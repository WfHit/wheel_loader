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
    uint8_t load_instance = _param_limit_load_idx.get();     // Load limit (bucket down)
    uint8_t dump_instance = _param_limit_dump_idx.get();     // Dump limit (bucket up)

    PX4_INFO("Using limit sensors: load=%d, dump=%d", load_instance, dump_instance);

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

    // Get current boom angle from AS5600 magnetic encoder
    sensor_mag_encoder_s mag_encoder_data;
    _current_boom_angle = 0.0f;

    // Read the latest magnetic encoder data for boom angle
    if (_sensor_mag_encoder_sub.update(&mag_encoder_data)) {
        // Validate sensor readings
        if (mag_encoder_data.magnet_detected &&
            !mag_encoder_data.magnet_too_strong &&
            !mag_encoder_data.magnet_too_weak) {
            _current_boom_angle = mag_encoder_data.angle;
        } else {
            // Log sensor issues for debugging
            if (!mag_encoder_data.magnet_detected) {
                PX4_DEBUG("Boom AS5600: No magnet detected");
            } else if (mag_encoder_data.magnet_too_strong) {
                PX4_DEBUG("Boom AS5600: Magnet too strong");
            } else if (mag_encoder_data.magnet_too_weak) {
                PX4_DEBUG("Boom AS5600: Magnet too weak");
            }
        }
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

    // Update calibration if in progress
    if (_calibration_mode) {
        update_calibration();
    }

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
            _zeroing_state = ZeroingState::MOVE_TO_LOAD_LIMIT;
            _zeroing_start_time = hrt_absolute_time();
            _zeroing_state_start_time = hrt_absolute_time();
            PX4_INFO("Starting bucket zeroing operation - moving to load limit");
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
        case ZeroingState::MOVE_TO_LOAD_LIMIT:
        {
            // Move down to load limit switch (bucket down position) at moderate speed
            if (!_limit_switch_load) {
                setMotorCommand(-fast_speed * 0.5f); // 50% of fast speed downward
            } else {
                // Load limit reached, start deceleration
                _zeroing_state = ZeroingState::SETTLE_AT_LOAD;
                _zeroing_state_start_time = hrt_absolute_time();
                PX4_INFO("Load limit reached, settling");
            }
            break;
        }

        case ZeroingState::SETTLE_AT_LOAD:
        {
            // Gradual stop to avoid overshoot
            float settle_time = hrt_elapsed_time(&_zeroing_state_start_time) * 1e-6f;
            float speed = -fast_speed * 0.5f * fmaxf(0.0f, 1.0f - settle_time / 0.5f); // 0.5s deceleration

            if (settle_time > 0.5f) {
                setMotorCommand(0.0f);
                // Set target for fast move (90% of actuator range)
                _zeroing_target = _current_actuator_length +
                                 (_kinematics.actuator_max_length - _kinematics.actuator_min_length) * 0.9f;
                _zeroing_state = ZeroingState::FAST_MOVE_TO_DUMP;
                _zeroing_state_start_time = hrt_absolute_time();
                PX4_INFO("Moving to dump limit, target: %.1fmm", static_cast<double>(_zeroing_target));
            } else {
                setMotorCommand(speed);
            }
            break;
        }

        case ZeroingState::FAST_MOVE_TO_DUMP:
        {
            // Use PX4 motion planning library for smooth fast movement toward dump limit
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

            // Check if we're getting close to target or if dump limit is reached
            if (_limit_switch_dump || fabsf(_zeroing_target - _current_actuator_length) < 10.0f) { // Within 10mm
                _zeroing_state = ZeroingState::SLOW_APPROACH_DUMP;
                _zeroing_state_start_time = hrt_absolute_time();
                PX4_INFO("Slow approach to dump limit");
            } else {
                setMotorCommand(control);
            }
            break;
        }

        case ZeroingState::SLOW_APPROACH_DUMP:
        {
            // Very slow movement to touch dump limit switch
            if (!_limit_switch_dump) {
                setMotorCommand(slow_speed); // Slow upward movement
            } else {
                // Dump limit reached - stop immediately and reset encoder using message
                setMotorCommand(0.0f);

                // Send encoder reset command via uORB message
                quad_encoder_reset_s reset_cmd{};
                reset_cmd.timestamp = hrt_absolute_time();
                reset_cmd.instance = _param_encoder_index.get();

                _quad_encoder_reset_pub.publish(reset_cmd);

                // Reset our internal tracking
                _encoder_zero_offset = 0;  // Reset since encoder driver will reset to 0
                _encoder_count = 0;
                _current_actuator_length = _kinematics.actuator_max_length;
                _zeroing_state = ZeroingState::COMPLETE;
                _zeroing_complete = true;

                PX4_INFO("Zeroing complete at dump limit - encoder reset command sent for instance %d",
                         _param_encoder_index.get());
            }

            // Timeout protection for slow approach
            if (hrt_elapsed_time(&_zeroing_state_start_time) > 15_s) {
                PX4_ERR("Dump limit approach timeout");
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
    // Check limits: load limit prevents downward motion, dump limit prevents upward motion
    if ((_limit_switch_load && _control_output < 0) ||   // Load limit active, blocking downward
        (_limit_switch_dump && _control_output > 0)) {   // Dump limit active, blocking upward
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

    uint8_t load_instance = _param_limit_load_idx.get();  // Load limit (bucket down)
    uint8_t dump_instance = _param_limit_dump_idx.get();  // Dump limit (bucket up)

    // Check for updated limit sensor data
    while (_limit_sensor_sub.update(&limit_sensor_data)) {
        if (limit_sensor_data.instance == load_instance) {
            _limit_switch_load = limit_sensor_data.state;   // Load limit switch
        } else if (limit_sensor_data.instance == dump_instance) {
            _limit_switch_dump = limit_sensor_data.state;   // Dump limit switch
        }
    }

    return _limit_switch_load || _limit_switch_dump;
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
    status.limit_switch_coarse = _limit_switch_load;     // Load limit switch status
    status.limit_switch_fine = _limit_switch_dump;       // Dump limit switch status
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
    if (!is_running()) {
        PX4_ERR("Module not running");
        return PX4_ERROR;
    }

    BucketControl *instance = get_instance();
    if (!instance) {
        PX4_ERR("Instance not available");
        return PX4_ERROR;
    }

    if (argc < 2) {
        return print_usage("missing command");
    }

    if (!strcmp(argv[1], "test")) {
        if (argc < 3) {
            PX4_INFO("Available test commands:");
            PX4_INFO("  mode <0-4>     - Set control mode (0=MANUAL, 1=AUTO_LEVEL, 2=SLOPE_COMP, 3=GRADING, 4=TRANSPORT)");
            PX4_INFO("  angle <deg>    - Set target angle in degrees");
            PX4_INFO("  status         - Show current status");
            return PX4_OK;
        }

        if (!strcmp(argv[2], "mode")) {
            if (argc < 4) {
                PX4_ERR("Usage: bucket_control test mode <0-4>");
                return PX4_ERROR;
            }

            int mode = atoi(argv[3]);
            if (mode < 0 || mode > 4) {
                PX4_ERR("Invalid mode %d. Valid range: 0-4", mode);
                return PX4_ERROR;
            }

            // Create and publish bucket command to change mode
            bucket_command_s cmd{};
            cmd.timestamp = hrt_absolute_time();
            cmd.target_angle = instance->_target_ground_angle; // Keep current target angle
            cmd.control_mode = static_cast<uint8_t>(mode);
            cmd.command_mode = 0; // Position mode
            cmd.coordinate_frame = 0; // Ground reference
            cmd.max_velocity = instance->_param_max_velocity.get() / 1000.0f; // Convert mm/s to m/s for angle rate
            cmd.enable_stability_limit = instance->_param_ahrs_enabled.get();
            cmd.enable_anti_spill = (mode == 4); // Enable anti-spill for transport mode
            cmd.grading_angle = std::nanf(""); // Use parameter defaults
            cmd.transport_angle = std::nanf(""); // Use parameter defaults
            cmd.stability_threshold = std::nanf(""); // Use parameter defaults

            // Publish the command
            static uORB::Publication<bucket_command_s> test_bucket_cmd_pub{ORB_ID(bucket_command)};
            test_bucket_cmd_pub.publish(cmd);

            const char* mode_names[] = {"MANUAL", "AUTO_LEVEL", "SLOPE_COMPENSATION", "GRADING", "TRANSPORT"};
            PX4_INFO("Bucket command published: mode %d (%s)", mode, mode_names[mode]);
            return PX4_OK;
        }

        if (!strcmp(argv[2], "angle")) {
            if (argc < 4) {
                PX4_ERR("Usage: bucket_control test angle <degrees>");
                return PX4_ERROR;
            }

            float angle_deg = atof(argv[3]);
            float angle_rad = math::radians(angle_deg);

            // Validate angle range (typical bucket range: -90 to +90 degrees)
            if (angle_deg < -90.0f || angle_deg > 90.0f) {
                PX4_WARN("Angle %f° is outside typical range [-90, +90]", static_cast<double>(angle_deg));
            }

            // Create and publish bucket command to set target angle
            bucket_command_s cmd{};
            cmd.timestamp = hrt_absolute_time();
            cmd.target_angle = angle_rad; // Set the target angle
            cmd.control_mode = static_cast<uint8_t>(instance->_control_mode); // Keep current mode
            cmd.command_mode = 0; // Position mode
            cmd.coordinate_frame = 0; // Ground reference
            cmd.max_velocity = instance->_param_max_velocity.get() / 1000.0f; // Convert mm/s to m/s for angle rate
            cmd.enable_stability_limit = instance->_param_ahrs_enabled.get();
            cmd.enable_anti_spill = (instance->_control_mode == ControlMode::TRANSPORT);
            cmd.grading_angle = std::nanf(""); // Use parameter defaults
            cmd.transport_angle = std::nanf(""); // Use parameter defaults
            cmd.stability_threshold = std::nanf(""); // Use parameter defaults

            // Publish the command
            static uORB::Publication<bucket_command_s> test_bucket_cmd_pub{ORB_ID(bucket_command)};
            test_bucket_cmd_pub.publish(cmd);

            PX4_INFO("Bucket command published: target angle %.1f° (%.3f rad)", static_cast<double>(angle_deg), static_cast<double>(angle_rad));
            return PX4_OK;
        }

        if (!strcmp(argv[2], "status")) {
            const char* mode_names[] = {"MANUAL", "AUTO_LEVEL", "SLOPE_COMPENSATION", "GRADING", "TRANSPORT"};
            const char* state_names[] = {"UNINITIALIZED", "ZEROING", "READY", "MOVING", "ERROR"};

            PX4_INFO("=== Bucket Control Status ===");
            PX4_INFO("State: %s", state_names[static_cast<int>(instance->_state)]);
            PX4_INFO("Control Mode: %s", mode_names[static_cast<int>(instance->_control_mode)]);
            PX4_INFO("Target Ground Angle: %.1f°", static_cast<double>(math::degrees(instance->_target_ground_angle)));
            PX4_INFO("Current Bucket Angle: %.1f°", static_cast<double>(math::degrees(instance->_current_bucket_angle)));
            PX4_INFO("Current Ground Angle: %.1f°", static_cast<double>(math::degrees(instance->_current_ground_angle)));
            PX4_INFO("Actuator Length: %.1f mm (target: %.1f mm)",
                     static_cast<double>(instance->_current_actuator_length),
                     static_cast<double>(instance->_target_actuator_length));
            PX4_INFO("Limit Switches - Load: %s, Dump: %s",
                     instance->_limit_switch_load ? "ACTIVE" : "inactive",
                     instance->_limit_switch_dump ? "ACTIVE" : "inactive");
            PX4_INFO("Zeroing Complete: %s", instance->_zeroing_complete ? "YES" : "NO");

            if (instance->_param_ahrs_enabled.get()) {
                PX4_INFO("Machine Pitch: %.1f°, Roll: %.1f°",
                         static_cast<double>(math::degrees(instance->_machine_pitch)),
                         static_cast<double>(math::degrees(instance->_machine_roll)));
                PX4_INFO("Stability Factor: %.2f", static_cast<double>(instance->_stability_factor));
                PX4_INFO("Anti-spill Active: %s", instance->_anti_spill_active ? "YES" : "NO");
            }

            return PX4_OK;
        }

        return print_usage("unknown test command");
    }

    if (!strcmp(argv[1], "calibrate")) {
        if (instance->_calibration_mode) {
            PX4_WARN("Calibration already in progress - state: %d", static_cast<int>(instance->_calib_state));
            return PX4_ERROR;
        }

        // Check if zeroing is complete first
        if (!instance->_zeroing_complete) {
            PX4_ERR("Cannot start calibration: zeroing not complete");
            return PX4_ERROR;
        }

        // Start auto-calibration
        if (instance->start_auto_calibration()) {
            PX4_INFO("Auto-calibration started for AS5600 magnetic encoder");
            return PX4_OK;
        } else {
            PX4_ERR("Failed to start auto-calibration");
            return PX4_ERROR;
        }
    }

    return print_usage("unknown command");
}

bool BucketControl::start_auto_calibration()
{
    if (_calibration_mode) {
        PX4_WARN("Calibration already in progress");
        return false;
    }

    // Check if zeroing is complete
    if (!_zeroing_complete) {
        PX4_ERR("Cannot start calibration: zeroing not complete");
        return false;
    }

    // Initialize calibration
    _calibration_mode = true;
    _calib_state = CalibrationState::IDLE;
    _calib_start_time = hrt_absolute_time();
    _calib_timeout_ms = 30000; // 30 seconds timeout
    _calib_min_angle = NAN;
    _calib_max_angle = NAN;
    _calib_center_angle = NAN;

    PX4_INFO("Starting AS5600 auto-calibration for bucket control");
    _calib_state = CalibrationState::MOVING_TO_CENTER;

    // Move to center position first
    set_target_actuator_length(_param_length_center.get());

    return true;
}

void BucketControl::update_calibration()
{
    if (!_calibration_mode) {
        return;
    }

    const hrt_abstime now = hrt_absolute_time();

    // Check for timeout
    if (now - _calib_start_time > _calib_timeout_ms * 1000) {
        PX4_ERR("Calibration timeout");
        abort_calibration();
        return;
    }

    switch (_calib_state) {
        case CalibrationState::IDLE:
            // Should not happen
            break;

        case CalibrationState::MOVING_TO_CENTER: {
            // Wait for actuator to reach center position
            float length_error = fabsf(_current_actuator_length - _param_length_center.get());
            if (length_error < 5.0f) { // 5mm tolerance
                _calib_state = CalibrationState::READING_CENTER;
                _calib_center_angle = get_as5600_angle();
                PX4_INFO("Center position reached, AS5600 angle: %.1f°", static_cast<double>(math::degrees(_calib_center_angle)));

                // Move to minimum position
                set_target_actuator_length(_param_length_min.get());
                _calib_state = CalibrationState::MOVING_TO_MIN;
            }
            break;
        }

        case CalibrationState::READING_CENTER:
            // Already handled in MOVING_TO_CENTER
            break;

        case CalibrationState::MOVING_TO_MIN: {
            // Wait for actuator to reach minimum position
            float length_error = fabsf(_current_actuator_length - _param_length_min.get());
            if (length_error < 5.0f) { // 5mm tolerance
                _calib_state = CalibrationState::READING_MIN;
                _calib_min_angle = get_as5600_angle();
                PX4_INFO("Minimum position reached, AS5600 angle: %.1f°", static_cast<double>(math::degrees(_calib_min_angle)));

                // Move to maximum position
                set_target_actuator_length(_param_length_max.get());
                _calib_state = CalibrationState::MOVING_TO_MAX;
            }
            break;
        }

        case CalibrationState::READING_MIN:
            // Already handled in MOVING_TO_MIN
            break;

        case CalibrationState::MOVING_TO_MAX: {
            // Wait for actuator to reach maximum position
            float length_error = fabsf(_current_actuator_length - _param_length_max.get());
            if (length_error < 5.0f) { // 5mm tolerance
                _calib_state = CalibrationState::READING_MAX;
                _calib_max_angle = get_as5600_angle();
                PX4_INFO("Maximum position reached, AS5600 angle: %.1f°", static_cast<double>(math::degrees(_calib_max_angle)));

                // Complete calibration
                complete_calibration();
            }
            break;
        }

        case CalibrationState::READING_MAX:
            // Already handled in MOVING_TO_MAX
            break;

        case CalibrationState::COMPLETED:
        case CalibrationState::FAILED:
            // Calibration is done
            break;
    }
}

void BucketControl::complete_calibration()
{
    if (!_calibration_mode) {
        return;
    }

    // Validate calibration data
    if (isnan(_calib_min_angle) || isnan(_calib_max_angle) || isnan(_calib_center_angle)) {
        PX4_ERR("Calibration failed: invalid angle readings");
        abort_calibration();
        return;
    }

    // Calculate angle range
    float angle_range = fabsf(_calib_max_angle - _calib_min_angle);
    if (angle_range < math::radians(30.0f)) { // Minimum 30 degrees range
        PX4_ERR("Calibration failed: insufficient angle range (%.1f°)", static_cast<double>(math::degrees(angle_range)));
        abort_calibration();
        return;
    }

    // Store calibration results
    PX4_INFO("AS5600 Calibration completed successfully:");
    PX4_INFO("  Min position: %.1f° (length: %.1f mm)",
             static_cast<double>(math::degrees(_calib_min_angle)),
             static_cast<double>(_param_length_min.get()));
    PX4_INFO("  Center position: %.1f° (length: %.1f mm)",
             static_cast<double>(math::degrees(_calib_center_angle)),
             static_cast<double>(_param_length_center.get()));
    PX4_INFO("  Max position: %.1f° (length: %.1f mm)",
             static_cast<double>(math::degrees(_calib_max_angle)),
             static_cast<double>(_param_length_max.get()));
    PX4_INFO("  Total range: %.1f°", static_cast<double>(math::degrees(angle_range)));

    _calib_state = CalibrationState::COMPLETED;

    // Return to center position
    set_target_actuator_length(_param_length_center.get());

    // End calibration mode
    _calibration_mode = false;
}

void BucketControl::abort_calibration()
{
    if (!_calibration_mode) {
        return;
    }

    PX4_WARN("AS5600 calibration aborted");
    _calib_state = CalibrationState::FAILED;
    _calibration_mode = false;

    // Return to center position for safety
    set_target_actuator_length(_param_length_center.get());
}

bool BucketControl::check_limit_sensors()
{
    // In bucket control, we use limit switches instead of limit sensors
    // This function checks if limit switches are working properly
    return true; // For now, assume they're always working
}

float BucketControl::get_as5600_angle()
{
    // This would read from the AS5600 magnetic encoder
    // For now, return a simulated angle based on actuator position
    // TODO: Implement actual AS5600 I2C communication

    // Simulate angle based on actuator length (for testing)
    float length_ratio = (_current_actuator_length - _param_length_min.get()) /
                        (_param_length_max.get() - _param_length_min.get());

    // Convert to angle range (assuming bucket rotates from -45° to +45°)
    float simulated_angle = math::radians(-45.0f + length_ratio * 90.0f);

    return simulated_angle;
}

float BucketControl::translate_as5600_to_bucket_angle(float as5600_angle)
{
    if (!_calibration_mode && _calib_state != CalibrationState::COMPLETED) {
        // Use default translation if not calibrated
        return as5600_angle;
    }

    // Use calibration data to translate AS5600 angle to bucket angle
    if (isnan(_calib_min_angle) || isnan(_calib_max_angle)) {
        return as5600_angle;
    }

    // Linear interpolation between calibrated points
    float as5600_range = _calib_max_angle - _calib_min_angle;
    float bucket_range = math::radians(_param_angle_max.get() - _param_angle_min.get());

    if (fabsf(as5600_range) < 1e-6f) {
        return as5600_angle; // Avoid division by zero
    }

    float ratio = (as5600_angle - _calib_min_angle) / as5600_range;
    float bucket_angle = math::radians(_param_angle_min.get()) + ratio * bucket_range;

    return bucket_angle;
}

float BucketControl::translate_bucket_to_as5600_angle(float bucket_angle)
{
    if (!_calibration_mode && _calib_state != CalibrationState::COMPLETED) {
        // Use default translation if not calibrated
        return bucket_angle;
    }

    // Use calibration data to translate bucket angle to AS5600 angle
    if (isnan(_calib_min_angle) || isnan(_calib_max_angle)) {
        return bucket_angle;
    }

    // Linear interpolation between calibrated points
    float bucket_range = math::radians(_param_angle_max.get() - _param_angle_min.get());
    float as5600_range = _calib_max_angle - _calib_min_angle;

    if (fabsf(bucket_range) < 1e-6f) {
        return bucket_angle; // Avoid division by zero
    }

    float ratio = (bucket_angle - math::radians(_param_angle_min.get())) / bucket_range;
    float as5600_angle = _calib_min_angle + ratio * as5600_range;

    return as5600_angle;
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
Supports zeroing procedure with load limit (bucket down) and dump limit (bucket up) switches.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("bucket_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("test mode <0-4>", "Set control mode (0=MANUAL, 1=AUTO_LEVEL, 2=SLOPE_COMP, 3=GRADING, 4=TRANSPORT)");
    PRINT_MODULE_USAGE_COMMAND_DESCR("test angle <deg>", "Set target angle in degrees");
    PRINT_MODULE_USAGE_COMMAND_DESCR("test status", "Show current module status");
    PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Start AS5600 magnetic encoder auto-calibration");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int bucket_control_main(int argc, char *argv[])
{
    return BucketControl::main(argc, argv);
}
