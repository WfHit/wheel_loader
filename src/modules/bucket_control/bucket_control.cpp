#include "bucket_control.hpp"
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

using matrix::Vector3f;

BucketControl::BucketControl() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
    // Initialize motion planning components
    _velocity_smoother.setMaxAccel(200.0f);  // mm/s²
    _velocity_smoother.setMaxVel(100.0f);    // mm/s
    _velocity_smoother.setMaxJerk(1000.0f);  // mm/s³

    // Initialize position smoother - using Z-axis for 1D motion
    _position_smoother.setMaxAccelerationZ(200.0f); // mm/s²
    _position_smoother.setMaxVelocityZ(100.0f);     // mm/s
    _position_smoother.setMaxJerkZ(1000.0f);        // mm/s³
}

bool BucketControl::init()
{
    // Initialize PID controller
    _position_pid.setGains(_param_pid_p.get(), _param_pid_i.get(), _param_pid_d.get());

    // Configure motion planning with parameters
    _velocity_smoother.setMaxAccel(_param_max_acceleration.get());
    _velocity_smoother.setMaxVel(_param_max_velocity.get());
    _velocity_smoother.setMaxJerk(_param_jerk_limit.get());

    _position_smoother.setMaxAccelerationZ(_param_max_acceleration.get());
    _position_smoother.setMaxVelocityZ(_param_max_velocity.get());
    _position_smoother.setMaxJerkZ(_param_jerk_limit.get());

    // Load kinematic parameters
    updateKinematicParameters();

    // Get motor and sensor indices
    _motor_index = _param_motor_index.get();

    // Get limit sensor instance IDs
    uint8_t coarse_instance = _param_limit_coarse_idx.get();
    uint8_t fine_instance = _param_limit_fine_idx.get();

    PX4_INFO("Using limit sensors: coarse=%d, fine=%d", coarse_instance, fine_instance);

    // Validate geometry
    if (_kinematics.bellcrank_length <= 0 || _kinematics.coupler_length <= 0) {
        PX4_ERR("Invalid linkage lengths");
        return false;
    }

    // The DRV8701 driver should already be running as a separate module
    // The quad encoder driver should already be running
    // We just subscribe to their outputs and publish motor commands

    // Schedule work item
    ScheduleOnInterval(10_ms); // 100Hz update rate

    PX4_INFO("Bucket control initialized");
    PX4_INFO("Bellcrank: %.1fmm, Coupler: %.1fmm, Boom: %.1fmm",
             (double)_kinematics.bellcrank_length, (double)_kinematics.coupler_length, (double)_kinematics.boom_length);
    PX4_INFO("Bellcrank internal angle: %.2f rad, Bucket arm: %.1fmm",
             (double)_kinematics.bellcrank_internal_angle, (double)_kinematics.bucket_arm_length);

    return true;
}

void BucketControl::updateKinematicParameters()
{
    // Load geometry from parameters
    _kinematics.actuator_base_x = _param_actuator_base_x.get();
    _kinematics.actuator_base_y = _param_actuator_base_y.get();
    _kinematics.bellcrank_boom_x = _param_bellcrank_boom_x.get();
    _kinematics.bellcrank_boom_y = _param_bellcrank_boom_y.get();
    _kinematics.bucket_boom_pivot_x = _param_bucket_boom_pivot_x.get();
    _kinematics.bucket_boom_pivot_y = _param_bucket_boom_pivot_y.get();

    _kinematics.bellcrank_length = _param_bellcrank_length.get();
    _kinematics.coupler_length = _param_coupler_length.get();
    _kinematics.actuator_offset = _param_actuator_offset.get();
    _kinematics.bucket_arm_length = _param_bucket_arm_length.get();
    _kinematics.bellcrank_internal_angle = _param_bellcrank_internal_angle.get();
    _kinematics.bucket_offset = _param_bucket_offset.get();
    _kinematics.boom_length = _param_boom_length.get();

    _kinematics.actuator_min_length = _param_actuator_min.get();
    _kinematics.actuator_max_length = _param_actuator_max.get();
}

bool BucketControl::solveBucketLinkage(float actuator_length, float boom_angle,
                                       float &bucket_angle, float &bellcrank_angle, float &coupler_angle)
{
    // Transform coordinates to account for boom rotation
    float cos_boom = cosf(boom_angle);
    float sin_boom = sinf(boom_angle);

    // Actuator base is fixed to chassis, so it rotates relative to boom
    float act_base_x_boom = _kinematics.actuator_base_x * cos_boom + _kinematics.actuator_base_y * sin_boom;
    float act_base_y_boom = -_kinematics.actuator_base_x * sin_boom + _kinematics.actuator_base_y * cos_boom;

    // Bellcrank pivot is fixed to boom (no transformation needed)
    float bellcrank_pivot_x = _kinematics.bellcrank_boom_x;
    float bellcrank_pivot_y = _kinematics.bellcrank_boom_y;

    // Step 1: Solve for bellcrank angle using actuator triangle
    // Triangle: actuator_base -> bellcrank_pivot -> actuator_attachment_on_bellcrank
    float dx = bellcrank_pivot_x - act_base_x_boom;
    float dy = bellcrank_pivot_y - act_base_y_boom;
    float base_to_bellcrank_dist = sqrtf(dx*dx + dy*dy);

    // Law of cosines to find angle at bellcrank pivot
    float cos_bellcrank_angle = (base_to_bellcrank_dist*base_to_bellcrank_dist +
                                _kinematics.actuator_offset*_kinematics.actuator_offset -
                                actuator_length*actuator_length) /
                               (2.0f * base_to_bellcrank_dist * _kinematics.actuator_offset);

    if (fabsf(cos_bellcrank_angle) > 1.0f) {
        // No valid solution - actuator cannot reach
        return false;
    }

    float base_angle = atan2f(dy, dx);
    bellcrank_angle = base_angle - acosf(cos_bellcrank_angle); // Subtract because actuator pulls

    // Step 2: Find bellcrank coupler attachment point using internal angle
    // The coupler attaches at an angle relative to the actuator arm
    float coupler_arm_angle = bellcrank_angle + _kinematics.bellcrank_internal_angle;
    float bellcrank_end_x = bellcrank_pivot_x + _kinematics.bellcrank_length * cosf(coupler_arm_angle);
    float bellcrank_end_y = bellcrank_pivot_y + _kinematics.bellcrank_length * sinf(coupler_arm_angle);

    // Step 3: Calculate bucket coupler attachment point
    float bucket_pivot_x = _kinematics.bucket_boom_pivot_x;
    float bucket_pivot_y = _kinematics.bucket_boom_pivot_y;

    dx = bucket_pivot_x - bellcrank_end_x;
    dy = bucket_pivot_y - bellcrank_end_y;
    float coupler_required_length = sqrtf(dx*dx + dy*dy);

    // Check if coupler can span the distance
    if (fabsf(coupler_required_length - _kinematics.coupler_length) > 1.0f) { // 1mm tolerance
        // Coupler cannot reach - invalid geometry
        return false;
    }

    coupler_angle = atan2f(dy, dx);

    // Step 4: Calculate bucket angle using bucket arm geometry
    // The bucket arm extends from the bucket pivot at bucket_offset angle relative to coupler
    float bucket_arm_angle = coupler_angle + (float)M_PI + _kinematics.bucket_offset; // +PI because coupler pulls bucket
    bucket_angle = bucket_arm_angle; // This is the bucket's orientation

    return true;
}

float BucketControl::actuatorLengthToBucketAngle(float actuator_length, float boom_angle)
{
    float bucket_angle, bellcrank_angle, coupler_angle;

    if (solveBucketLinkage(actuator_length, boom_angle, bucket_angle, bellcrank_angle, coupler_angle)) {
        return bucket_angle;
    }

    // Fallback to linear approximation if solver fails
    float normalized = (actuator_length - _kinematics.actuator_min_length) /
                      (_kinematics.actuator_max_length - _kinematics.actuator_min_length);
    return _param_angle_min.get() + normalized * (_param_angle_max.get() - _param_angle_min.get());
}

float BucketControl::bucketAngleToActuatorLength(float bucket_angle, float boom_angle)
{
    // Use iterative solver (binary search) to find actuator length for desired bucket angle
    float length_min = _kinematics.actuator_min_length;
    float length_max = _kinematics.actuator_max_length;
    float tolerance = 0.001f; // 0.001 rad tolerance
    int max_iterations = 30;

    for (int i = 0; i < max_iterations; i++) {
        float length_mid = (length_min + length_max) / 2.0f;
        float current_angle = actuatorLengthToBucketAngle(length_mid, boom_angle);
        float error = bucket_angle - current_angle;

        if (fabsf(error) < tolerance) {
            return length_mid;
        }

        if (error > 0) {
            // Need longer actuator
            length_min = length_mid;
        } else {
            // Need shorter actuator
            length_max = length_mid;
        }
    }

    return (length_min + length_max) / 2.0f;
}

float BucketControl::compensateBoomAngle(float target_ground_angle, float boom_angle)
{
    // Target ground angle is what we want relative to horizontal ground
    // We need to convert this to bucket angle relative to boom
    //
    // If boom rotates up by boom_angle, and we want bucket at same ground angle,
    // then bucket must rotate down relative to boom by boom_angle

    return target_ground_angle - boom_angle;
}

void BucketControl::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // Update parameters if changed
    parameter_update_s param_update;
    if (_parameter_update_sub.update(&param_update)) {
        updateParams();
        updateKinematicParameters();
        _motor_index = _param_motor_index.get();
        _control_mode = static_cast<ControlMode>(_param_control_mode.get());
    }

    // Update AHRS data if enabled
    if (_param_ahrs_enabled.get()) {
        updateAHRSData();
    }

    // Read sensors
    readEncoderFeedback();
    checkLimitSwitches();

    // Get current boom angle for all calculations
    boom_status_s boom_status;
    _current_boom_angle = 0.0f;
    if (_boom_status_sub.copy(&boom_status)) {
        _current_boom_angle = boom_status.angle;
    }

    // Update current angles for status
    _current_bucket_angle = actuatorLengthToBucketAngle(_current_actuator_length, _current_boom_angle);
    _current_ground_angle = calculateGroundRelativeAngle(_current_bucket_angle);

    // Process bucket commands
    bucket_command_s cmd;
    if (_bucket_cmd_sub.update(&cmd)) {
        _target_ground_angle = cmd.target_angle; // Command is in ground coordinates

        // Update control mode from command if specified
        if (cmd.control_mode != static_cast<uint8_t>(_control_mode)) {
            _control_mode = static_cast<ControlMode>(cmd.control_mode);
            PX4_INFO("Control mode changed to: %d", static_cast<int>(_control_mode));
        }

        // Override parameters with command values if provided (not NaN)
        if (!std::isnan(cmd.grading_angle)) {
            _grading_angle = cmd.grading_angle;
        } else {
            _grading_angle = _param_grading_angle.get();
        }

        if (!std::isnan(cmd.transport_angle)) {
            _transport_angle = cmd.transport_angle;
        } else {
            _transport_angle = _param_transport_angle.get();
        }

        // Apply control mode logic
        switch (_control_mode) {
        case ControlMode::AUTO_LEVEL:
            performAutoLevel();
            break;

        case ControlMode::GRADING:
            performGradingControl();
            break;

        case ControlMode::TRANSPORT:
            applyAntiSpillControl();
            break;

        case ControlMode::SLOPE_COMPENSATION:
            {
                float compensated_angle = compensateForSlope(_target_ground_angle);
                float target_bucket_angle = compensated_angle - _current_boom_angle;
                _target_actuator_length = bucketAngleToActuatorLength(target_bucket_angle, _current_boom_angle);
            }
            break;

        case ControlMode::MANUAL:
        default:
            // Use existing manual control
            float target_bucket_angle = compensateBoomAngle(_target_ground_angle, _current_boom_angle);
            _target_actuator_length = bucketAngleToActuatorLength(target_bucket_angle, _current_boom_angle);
            break;
        }

	PX4_DEBUG("Cmd: ground=%.2f°, boom=%.2f°, bucket=%.2f°, actuator=%.1fmm, mode=%d",
		 static_cast<double>(math::degrees(_target_ground_angle)),
		 static_cast<double>(math::degrees(_current_boom_angle)),
		 static_cast<double>(math::degrees(_current_bucket_angle)),
		 static_cast<double>(_target_actuator_length),
		 static_cast<int>(_control_mode));
    }

    // Update state machine
    updateStateMachine();

    // Apply stability limiting if AHRS is enabled
    if (_param_ahrs_enabled.get()) {
        updateStabilityFactor();
        _control_output = limitMovementForStability(_control_output);
    }

    // Publish status
    publishStatus();
}

void BucketControl::updateStateMachine()
{
    switch (_state) {
        case State::UNINITIALIZED:
            _state = State::ZEROING;
            _zeroing_state = ZeroingState::MOVE_TO_COARSE_LIMIT;
            _zeroing_start_time = hrt_absolute_time();
            _zeroing_state_start_time = hrt_absolute_time();
            PX4_INFO("Starting bucket zeroing operation");
            break;

        case State::ZEROING:
            performZeroing();
            if (_zeroing_complete) {
                _state = State::READY;
                PX4_INFO("Bucket control ready");
            }
            break;

        case State::READY:
        case State::MOVING:
            updateMotionControl();
            break;

        case State::ERROR:
            // Stop actuator
            setMotorCommand(0.0f);
            break;
    }
}

void BucketControl::performZeroing()
{
    float fast_speed = _param_zeroing_fast_speed.get();
    float slow_speed = _param_zeroing_slow_speed.get();

    switch (_zeroing_state) {
        case ZeroingState::MOVE_TO_COARSE_LIMIT:
        {
            // Move down to coarse limit switch at moderate speed
            if (!_limit_switch_coarse) {
                setMotorCommand(-fast_speed * 0.5f); // 50% of fast speed downward
            } else {
                // Coarse limit reached, start deceleration
                _zeroing_state = ZeroingState::SETTLE_AT_COARSE;
                _zeroing_state_start_time = hrt_absolute_time();
                PX4_INFO("Coarse limit reached, settling");
            }
            break;
        }

        case ZeroingState::SETTLE_AT_COARSE:
        {
            // Gradual stop to avoid overshoot
            float settle_time = hrt_elapsed_time(&_zeroing_state_start_time) * 1e-6f;
            float speed = -fast_speed * 0.5f * fmaxf(0.0f, 1.0f - settle_time / 0.5f); // 0.5s deceleration

            if (settle_time > 0.5f) {
                setMotorCommand(0.0f);
                // Set target for fast move (90% of actuator range)
                _zeroing_target = _current_actuator_length +
                                 (_kinematics.actuator_max_length - _kinematics.actuator_min_length) * 0.9f;
                _zeroing_state = ZeroingState::FAST_MOVE_TO_FINE;
                _zeroing_state_start_time = hrt_absolute_time();
                PX4_INFO("Moving to fine limit, target: %.1fmm", static_cast<double>(_zeroing_target));
            } else {
                setMotorCommand(speed);
            }
            break;
        }

        case ZeroingState::FAST_MOVE_TO_FINE:
        {
            // Use PX4 motion planning library for smooth fast movement toward fine limit
            float dt = 0.01f; // 100Hz control loop

            // Configure motion planning for zeroing (more aggressive parameters)
            _position_smoother.setMaxAccelerationZ(_param_max_acceleration.get() * 2.0f); // Double acceleration for zeroing
            _position_smoother.setMaxVelocityZ(_param_max_velocity.get() * 1.5f);         // 1.5x velocity for zeroing
            _position_smoother.setMaxJerkZ(_param_jerk_limit.get() * 2.0f);               // Double jerk for faster response

            // Use position smoother for trajectory generation (1D motion using Z-axis)
            matrix::Vector3f current_pos{0.0f, 0.0f, _current_actuator_length};
            matrix::Vector3f target_pos{0.0f, 0.0f, _zeroing_target};
            matrix::Vector3f feedforward_velocity{0.0f, 0.0f, 0.0f};

            PositionSmoothing::PositionSmoothingSetpoints setpoints;
            _position_smoother.generateSetpoints(current_pos, target_pos, feedforward_velocity,
                                               dt, false, setpoints);

            // Get smoothed position and velocity setpoints from Z component
            float position_setpoint = setpoints.position(2);
            float velocity_setpoint = setpoints.velocity(2);

            // Simple P control for zeroing with velocity feedforward
            float position_error = position_setpoint - _current_actuator_length;
            float control = math::constrain(position_error * 0.01f + velocity_setpoint * 0.005f, -fast_speed, fast_speed);

            // Check if we're getting close to target or if fine limit is reached
            if (_limit_switch_fine || fabsf(_zeroing_target - _current_actuator_length) < 10.0f) { // Within 10mm
                _zeroing_state = ZeroingState::SLOW_APPROACH_FINE;
                _zeroing_state_start_time = hrt_absolute_time();
                PX4_INFO("Slow approach to fine limit");
            } else {
                setMotorCommand(control);
            }
            break;
        }

        case ZeroingState::SLOW_APPROACH_FINE:
        {
            // Very slow movement to touch fine limit switch
            if (!_limit_switch_fine) {
                setMotorCommand(slow_speed); // Slow upward movement
            } else {
                // Fine limit reached - stop immediately and set zero
                setMotorCommand(0.0f);

                // Record this as our maximum position
                sensor_quad_encoder_s encoder_data;
                if (_sensor_quad_encoder_sub.copy(&encoder_data)) {
                    uint8_t encoder_idx = _param_encoder_index.get();
                    if (encoder_idx < encoder_data.count && encoder_data.valid[encoder_idx]) {
                        _encoder_zero_offset = encoder_data.position[encoder_idx];
                        _encoder_count = 0;
                        _current_actuator_length = _kinematics.actuator_max_length;
                        _zeroing_state = ZeroingState::COMPLETE;
                        _zeroing_complete = true;
                        PX4_INFO("Zeroing complete at fine limit, offset: %lld", (long long)_encoder_zero_offset);
                    }
                }
            }

            // Timeout protection for slow approach
            if (hrt_elapsed_time(&_zeroing_state_start_time) > 15_s) {
                PX4_ERR("Fine limit approach timeout");
                _state = State::ERROR;
                setMotorCommand(0.0f);
            }
            break;
        }

        case ZeroingState::COMPLETE:
            // Zeroing done
            break;
    }

    // Overall timeout protection
    if (hrt_elapsed_time(&_zeroing_start_time) > 60_s) {
        PX4_ERR("Zeroing operation timeout");
        _state = State::ERROR;
        setMotorCommand(0.0f);
    }
}

void BucketControl::updateMotionControl()
{
    // Check limits
    if ((_limit_switch_coarse && _control_output < 0) ||
        (_limit_switch_fine && _control_output > 0)) {
        setMotorCommand(0.0f);
        return;
    }

    // Update trajectory setpoint using motion planning
    float dt = 0.01f; // 100Hz control loop
    updateTrajectorySetpoint(dt);

    // PID control on position
    _position_pid.setSetpoint(_target_actuator_length);
    _control_output = _position_pid.update(_current_actuator_length, dt);

    // Apply control output
    setMotorCommand(_control_output);

    // Update state
    float position_error = _target_actuator_length - _current_actuator_length;
    _state = (fabsf(position_error) < 2.0f) ? State::READY : State::MOVING; // 2mm tolerance
}

void BucketControl::updateTrajectorySetpoint(float dt)
{
    // Configure motion planning constraints
    _velocity_smoother.setMaxAccel(_param_max_acceleration.get());
    _velocity_smoother.setMaxVel(_param_max_velocity.get());
    _velocity_smoother.setMaxJerk(_param_jerk_limit.get());

    // Set current state for velocity smoother
    _velocity_smoother.reset(0.0f, _current_velocity, _current_actuator_length);

    // Calculate velocity setpoint based on position error
    float position_error = _target_actuator_length - _current_actuator_length;
    float velocity_setpoint = 0.0f;

    if (fabsf(position_error) > 1.0f) {  // 1mm deadband
        // Use position smoother for trajectory generation (1D motion using Z-axis)
        matrix::Vector3f current_pos{0.0f, 0.0f, _current_actuator_length};
        matrix::Vector3f target_pos{0.0f, 0.0f, _target_actuator_length};
        matrix::Vector3f feedforward_velocity{0.0f, 0.0f, 0.0f};

        PositionSmoothing::PositionSmoothingSetpoints setpoints;
        _position_smoother.generateSetpoints(current_pos, target_pos, feedforward_velocity,
                                           dt, false, setpoints);

        // Get smoothed velocity setpoint from Z component
        velocity_setpoint = setpoints.velocity(2);

        // Apply velocity constraints
        velocity_setpoint = math::constrain(velocity_setpoint,
                                          -_param_max_velocity.get(),
                                          _param_max_velocity.get());
    }

    // Update internal velocity tracking
    _current_velocity = velocity_setpoint;
}

void BucketControl::setMotorCommand(float command)
{
    // Publish HBridge command for bucket motor
    hbridge_command_s cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.channel = _motor_index;
    cmd.duty_cycle = command;  // Use command directly as duty_cycle (-1.0 to 1.0)
    cmd.enable = true;

    _hbridge_command_pub.publish(cmd);
}

void BucketControl::readEncoderFeedback()
{
    // Read from sensor_quad_encoder topic published by quad encoder driver
    sensor_quad_encoder_s encoder_data;
    if (_sensor_quad_encoder_sub.update(&encoder_data)) {
        uint8_t encoder_idx = _param_encoder_index.get();

        if (encoder_idx < encoder_data.count && encoder_data.valid[encoder_idx]) {
            int32_t current_position = encoder_data.position[encoder_idx];
            _encoder_count = current_position - _encoder_zero_offset;

            // Calculate velocity from encoder changes
            if (_last_encoder_time > 0) {
                float dt = hrt_elapsed_time(&_last_encoder_time) * 1e-6f;
                if (dt > 0.001f) { // Avoid division by very small numbers
                    int64_t delta_count = _encoder_count - _last_encoder_count;
                    float delta_length = delta_count * _param_encoder_scale.get();
                    _current_velocity = delta_length / dt;
                }
            }

            _last_encoder_count = _encoder_count;
            _last_encoder_time = encoder_data.timestamp;

            // Convert encoder counts to actuator length
            _current_actuator_length = _encoder_count * _param_encoder_scale.get() + _kinematics.actuator_min_length;
        }
    }
}

bool BucketControl::checkLimitSwitches()
{
    // Read limit sensor states from limit_sensor topic
    limit_sensor_s limit_sensor_data;

    uint8_t coarse_instance = _param_limit_coarse_idx.get();
    uint8_t fine_instance = _param_limit_fine_idx.get();

    // Check for updated limit sensor data
    while (_limit_sensor_sub.update(&limit_sensor_data)) {
        if (limit_sensor_data.instance == coarse_instance) {
            _limit_switch_coarse = limit_sensor_data.state;
        } else if (limit_sensor_data.instance == fine_instance) {
            _limit_switch_fine = limit_sensor_data.state;
        }
    }

    return _limit_switch_coarse || _limit_switch_fine;
}

// AHRS Integration Methods

void BucketControl::updateAHRSData()
{
    // Get vehicle attitude
    vehicle_attitude_s attitude;
    if (_vehicle_attitude_sub.update(&attitude)) {
        // Convert quaternion to Euler angles
        matrix::Quatf q(attitude.q);
        matrix::Eulerf euler(q);

        _machine_roll = euler.phi();
        _machine_pitch = euler.theta();
        _machine_yaw = euler.psi();
    }

    // Get angular velocity
    vehicle_angular_velocity_s angular_vel;
    if (_vehicle_angular_velocity_sub.update(&angular_vel)) {
        _angular_rate_x = angular_vel.xyz[0];
        _angular_rate_y = angular_vel.xyz[1];
        _angular_rate_z = angular_vel.xyz[2];
    }

    // Get acceleration
    vehicle_acceleration_s acceleration;
    if (_vehicle_acceleration_sub.update(&acceleration)) {
        _acceleration_x = acceleration.xyz[0];
        _acceleration_y = acceleration.xyz[1];
        _acceleration_z = acceleration.xyz[2];
    }
}

float BucketControl::calculateGroundRelativeAngle(float bucket_boom_angle)
{
    // Calculate bucket angle relative to ground, accounting for machine tilt
    // bucket_boom_angle is relative to boom
    // _current_boom_angle is boom angle relative to machine
    // _machine_pitch is machine pitch relative to ground

    if (!_param_ahrs_enabled.get()) {
        // Without AHRS, just use simple boom compensation
        return bucket_boom_angle + _current_boom_angle;
    }

    float bucket_machine_angle = bucket_boom_angle + _current_boom_angle;
    float bucket_ground_angle = bucket_machine_angle - _machine_pitch;

    return bucket_ground_angle;
}

float BucketControl::compensateForSlope(float target_angle)
{
    // Compensate for machine pitch to maintain desired ground angle
    if (!_param_ahrs_enabled.get()) {
        return target_angle;
    }

    float compensation = _param_slope_compensation.get() * _machine_pitch;
    return target_angle + compensation;
}

void BucketControl::updateStabilityFactor()
{
    // Calculate stability based on machine attitude and motion
    float pitch_factor = fabsf(_machine_pitch) / _param_stability_threshold.get();
    float roll_factor = fabsf(_machine_roll) / _param_stability_threshold.get();

    // Consider angular rates for dynamic stability
    float pitch_rate_factor = fabsf(_angular_rate_y) / 1.0f; // 1 rad/s threshold
    float roll_rate_factor = fabsf(_angular_rate_x) / 1.0f;

    // Combined stability factor (0.0 = very unstable, 1.0 = stable)
    float static_stability = 1.0f - fmaxf(pitch_factor, roll_factor);
    float dynamic_stability = 1.0f - fmaxf(pitch_rate_factor, roll_rate_factor);

    _stability_factor = fminf(static_stability, dynamic_stability);
    _stability_factor = math::constrain(_stability_factor, 0.1f, 1.0f);

    // Set warning flag
    _stability_warning = (_stability_factor < 0.5f);
}

float BucketControl::limitMovementForStability(float command)
{
    // Reduce command based on stability factor
    if (_stability_warning) {
        // More aggressive limiting when unstable
        return command * _stability_factor * 0.5f;
    }
    return command * _stability_factor;
}

void BucketControl::performAutoLevel()
{
    // Auto-level bucket to maintain horizontal orientation
    float current_ground_angle = calculateGroundRelativeAngle(_current_bucket_angle);
    float angle_error = _target_ground_angle - current_ground_angle;

    // PD control for smooth leveling
    float p_term = _param_level_p_gain.get() * angle_error;
    float d_term = _param_level_d_gain.get() * (-_angular_rate_y); // Damping

    float level_command = p_term + d_term;

    // Convert to actuator length
    float target_bucket_angle = _current_bucket_angle + level_command;
    _target_actuator_length = bucketAngleToActuatorLength(target_bucket_angle, _current_boom_angle);
}

void BucketControl::performGradingControl()
{
    // Maintain consistent cutting angle for grading operations
    _grading_angle = _param_grading_angle.get();

    // Account for machine pitch and forward motion
    float adjusted_angle = _grading_angle - _machine_pitch;

    // Add feed-forward based on machine velocity (if available)
    if (fabsf(_acceleration_x) > 0.1f) {
        // Slight angle adjustment based on forward acceleration
        adjusted_angle += 0.1f * _acceleration_x;
    }

    float target_bucket_angle = adjusted_angle - _current_boom_angle;
    _target_actuator_length = bucketAngleToActuatorLength(target_bucket_angle, _current_boom_angle);
}

void BucketControl::applyAntiSpillControl()
{
    // Prevent material spillage during transport
    float spill_risk = 0.0f;

    // Check lateral acceleration (turning)
    float lateral_g = fabsf(_acceleration_y) / 9.81f;
    spill_risk = fmaxf(spill_risk, lateral_g / _param_spill_threshold.get());

    // Check pitch changes
    float pitch_rate_risk = fabsf(_angular_rate_y) / 0.5f; // 0.5 rad/s threshold
    spill_risk = fmaxf(spill_risk, pitch_rate_risk);

    // Check sudden stops (longitudinal acceleration)
    float brake_g = fabsf(_acceleration_x) / 9.81f;
    spill_risk = fmaxf(spill_risk, brake_g / _param_spill_threshold.get());

    _anti_spill_active = (spill_risk > 0.5f);

    if (_anti_spill_active) {
        // Tilt bucket back to prevent spillage
        float spill_compensation = math::constrain(spill_risk * 0.2f, 0.0f, 0.3f); // Max 0.3 rad
        _transport_angle = _param_transport_angle.get() + spill_compensation;

        float target_bucket_angle = _transport_angle - _current_boom_angle - _machine_pitch;
        _target_actuator_length = bucketAngleToActuatorLength(target_bucket_angle, _current_boom_angle);
    } else {
        // Normal transport angle
        _transport_angle = _param_transport_angle.get();
        float target_bucket_angle = _transport_angle - _current_boom_angle;
        _target_actuator_length = bucketAngleToActuatorLength(target_bucket_angle, _current_boom_angle);
    }
}

void BucketControl::publishStatus()
{
    bucket_status_s status{};
    status.timestamp = hrt_absolute_time();

    // Basic status
    status.state = static_cast<uint8_t>(_state);
    status.actuator_length = _current_actuator_length;
    status.target_actuator_length = _target_actuator_length;
    status.bucket_angle = _current_bucket_angle;
    status.ground_angle = _current_ground_angle;
    status.velocity = _current_velocity;
    status.control_output = _control_output;
    status.limit_switch_coarse = _limit_switch_coarse;
    status.limit_switch_fine = _limit_switch_fine;
    status.zeroing_complete = _zeroing_complete;

    // AHRS-related status
    if (_param_ahrs_enabled.get()) {
        status.control_mode = static_cast<uint8_t>(_control_mode);
        status.stability_factor = _stability_factor;
        status.anti_spill_active = _anti_spill_active;
        status.stability_warning = _stability_warning;
        status.machine_pitch = _machine_pitch;
        status.machine_roll = _machine_roll;
        status.target_ground_angle = _target_ground_angle;
    }

    _bucket_status_pub.publish(status);
}

// Module interface implementation
int BucketControl::task_spawn(int argc, char *argv[])
{
    BucketControl *instance = new BucketControl();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }

    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int BucketControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int BucketControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Bucket control module for wheel loader.

Manages bucket angle control through linear actuator with boom angle compensation.
Supports zeroing procedure with coarse and fine limit switches.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("bucket_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int bucket_control_main(int argc, char *argv[])
{
    return BucketControl::main(argc, argv);
}
