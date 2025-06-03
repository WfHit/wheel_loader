#include "bucket_control.hpp"
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

BucketControl::BucketControl() :
    ModuleParams(nullptr),
    WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
    _position_pid.set_dt(0.01f); // 100Hz control loop
}

bool BucketControl::init()
{
    // Initialize PID controller
    _position_pid.set_parameters(_param_pid_p.get(), _param_pid_i.get(), _param_pid_d.get(),
                                 0.0f, 1.0f); // integral limit, output limit

    // Load kinematic parameters
    updateKinematicParameters();

    // Get motor and sensor indices
    _motor_index = _param_motor_index.get();

    // Validate geometry
    if (_kinematics.bellcrank_length <= 0 || _kinematics.linkage_length <= 0) {
        PX4_ERR("Invalid linkage lengths");
        return false;
    }

    // The DRV8701 driver should already be running as a separate module
    // The quad encoder driver should already be running
    // We just subscribe to their outputs and publish motor commands

    // Schedule work item
    ScheduleOnInterval(10_ms); // 100Hz update rate

    PX4_INFO("Bucket control initialized");
    PX4_INFO("Bellcrank: %.1fmm, Linkage: %.1fmm, Boom: %.1fmm",
             _kinematics.bellcrank_length, _kinematics.linkage_length, _kinematics.boom_length);

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
    _kinematics.linkage_length = _param_linkage_length.get();
    _kinematics.actuator_offset = _param_actuator_offset.get();
    _kinematics.bucket_offset = _param_bucket_offset.get();
    _kinematics.boom_length = _param_boom_length.get();

    _kinematics.actuator_min_length = _param_actuator_min.get();
    _kinematics.actuator_max_length = _param_actuator_max.get();
}

bool BucketControl::solveBucketLinkage(float actuator_length, float boom_angle,
                                       float &bucket_angle, float &bellcrank_angle, float &linkage_angle)
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

    // Step 2: Find bellcrank endpoint (where linkage attaches)
    float bellcrank_end_x = bellcrank_pivot_x + _kinematics.bellcrank_length * cosf(bellcrank_angle);
    float bellcrank_end_y = bellcrank_pivot_y + _kinematics.bellcrank_length * sinf(bellcrank_angle);

    // Step 3: Solve for linkage angle
    // Linkage connects bellcrank end to bucket pivot
    float bucket_pivot_x = _kinematics.bucket_boom_pivot_x;
    float bucket_pivot_y = _kinematics.bucket_boom_pivot_y;

    dx = bucket_pivot_x - bellcrank_end_x;
    dy = bucket_pivot_y - bellcrank_end_y;
    float linkage_required_length = sqrtf(dx*dx + dy*dy);

    // Check if linkage can span the distance
    if (fabsf(linkage_required_length - _kinematics.linkage_length) > 1.0f) { // 1mm tolerance
        // Linkage cannot reach - invalid geometry
        return false;
    }

    linkage_angle = atan2f(dy, dx);

    // Step 4: Calculate bucket angle
    // Bucket angle is determined by linkage orientation plus offset
    bucket_angle = linkage_angle + _kinematics.bucket_offset;

    return true;
}

float BucketControl::actuatorLengthToBucketAngle(float actuator_length, float boom_angle)
{
    float bucket_angle, bellcrank_angle, linkage_angle;

    if (solveBucketLinkage(actuator_length, boom_angle, bucket_angle, bellcrank_angle, linkage_angle)) {
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
    }

    // Read sensors
    readEncoderFeedback();
    checkLimitSwitches();

    // Process bucket commands
    bucket_command_s cmd;
    if (_bucket_cmd_sub.update(&cmd)) {
        float target_ground_angle = cmd.target_angle; // Command is in ground coordinates

        // Get current boom angle for compensation
        boom_status_s boom_status;
        float boom_angle = 0.0f;
        if (_boom_status_sub.copy(&boom_status)) {
            boom_angle = boom_status.angle;
        }

        // Convert ground angle to bucket angle relative to boom
        float target_bucket_angle = compensateBoomAngle(target_ground_angle, boom_angle);

        // Convert bucket angle to actuator length
        _target_actuator_length = bucketAngleToActuatorLength(target_bucket_angle, boom_angle);

        PX4_DEBUG("Cmd: ground=%.2f°, boom=%.2f°, bucket=%.2f°, actuator=%.1fmm",
                 math::degrees(target_ground_angle), math::degrees(boom_angle),
                 math::degrees(target_bucket_angle), _target_actuator_length);
    }

    // Update current angles for status
    boom_status_s boom_status;
    float boom_angle = 0.0f;
    if (_boom_status_sub.copy(&boom_status)) {
        boom_angle = boom_status.angle;
    }

    _current_bucket_angle = actuatorLengthToBucketAngle(_current_actuator_length, boom_angle);
    _current_ground_angle = _current_bucket_angle + boom_angle;

    // Update state machine
    updateStateMachine();

    // Publish status
    bucket_status_s status{};
    status.timestamp = hrt_absolute_time();
    status.current_angle = _current_ground_angle;        // Ground angle for user
    status.bucket_relative_angle = _current_bucket_angle; // Boom-relative angle
    status.current_velocity = _current_velocity;
    status.actuator_length = _current_actuator_length;
    status.limit_switch_min = _limit_switch_coarse;
    status.limit_switch_max = _limit_switch_fine;
    status.state = static_cast<uint8_t>(_state);
    _bucket_status_pub.publish(status);
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
                PX4_INFO("Moving to fine limit, target: %.1fmm", _zeroing_target);
            } else {
                setMotorCommand(speed);
            }
            break;
        }

        case ZeroingState::FAST_MOVE_TO_FINE:
        {
            // Use S-curve planner for smooth fast movement toward fine limit
            float velocity_setpoint, acceleration;
            float position_setpoint = generateSCurveSetpoint(_current_actuator_length, _zeroing_target,
                                                            velocity_setpoint, acceleration);

            // Simple P control for zeroing
            float position_error = position_setpoint - _current_actuator_length;
            float control = math::constrain(position_error * 0.01f, -fast_speed, fast_speed);

            // Check if we're getting close to target or if fine limit is reached
            if (_limit_switch_fine || fabsf(position_error) < 10.0f) { // Within 10mm
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
                wheel_encoders_s encoders;
                if (_wheel_encoders_sub.copy(&encoders)) {
                    uint8_t encoder_idx = _param_encoder_index.get();
                    _encoder_zero_offset = encoders.wheel_angle[encoder_idx];
                    _encoder_count = 0;
                    _current_actuator_length = _kinematics.actuator_max_length;
                    _zeroing_state = ZeroingState::COMPLETE;
                    _zeroing_complete = true;
                    PX4_INFO("Zeroing complete at fine limit, offset: %lld", _encoder_zero_offset);
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

    // S-curve trajectory generation
    float velocity_setpoint, acceleration;
    float position_setpoint = generateSCurveSetpoint(_current_actuator_length, _target_actuator_length,
                                                     velocity_setpoint, acceleration);

    // PID control
    float position_error = position_setpoint - _current_actuator_length;
    _control_output = _position_pid.control_error(position_error);

    // Apply control output
    setMotorCommand(_control_output);

    // Update state
    _state = (fabsf(position_error) < 2.0f) ? State::READY : State::MOVING; // 2mm tolerance
}

float BucketControl::generateSCurveSetpoint(float current, float target, float &velocity, float &acceleration)
{
    // Simple S-curve implementation
    float error = target - current;
    float max_vel = _param_max_velocity.get();
    float max_acc = _param_max_acceleration.get();

    // Calculate required deceleration distance
    float decel_dist = (_current_velocity * _current_velocity) / (2.0f * max_acc);

    if (fabsf(error) < decel_dist) {
        // Deceleration phase
        velocity = _current_velocity * 0.95f; // Gradual deceleration
    } else if (fabsf(_current_velocity) < max_vel) {
        // Acceleration phase
        velocity = _current_velocity + (error > 0 ? max_acc : -max_acc) * 0.01f; // dt = 0.01s
        velocity = math::constrain(velocity, -max_vel, max_vel);
    } else {
        // Constant velocity phase
        velocity = error > 0 ? max_vel : -max_vel;
    }

    acceleration = (velocity - _current_velocity) / 0.01f;

    return current + velocity * 0.01f;
}

void BucketControl::setMotorCommand(float command)
{
    // Publish to actuator_motors topic that DRV8701 driver subscribes to
    actuator_motors_s motors{};
    motors.timestamp = hrt_absolute_time();

    // Initialize all motors to disarmed
    for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
        motors.control[i] = NAN;
    }

    // Set our motor command (-1 to 1)
    motors.control[_motor_index] = math::constrain(command, -1.0f, 1.0f);

    _actuator_motors_pub.publish(motors);
}

void BucketControl::readEncoderFeedback()
{
    // Read from wheel_encoders topic published by quad encoder driver
    wheel_encoders_s encoders;
    if (_wheel_encoders_sub.copy(&encoders)) {
        uint8_t encoder_idx = _param_encoder_index.get();

        if (encoder_idx < wheel_encoders_s::WHEEL_COUNT) {
            _encoder_count = encoders.wheel_angle[encoder_idx] - _encoder_zero_offset;

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
            _last_encoder_time = hrt_absolute_time();

            // Convert encoder counts to actuator length
            _current_actuator_length = _encoder_count * _param_encoder_scale.get() + _kinematics.actuator_min_length;
        }
    }
}

bool BucketControl::checkLimitSwitches()
{
    // Read GPIO states from sensor_gpio topic
    sensor_gpio_s gpio_state;
    if (_sensor_gpio_sub.copy(&gpio_state)) {
        uint8_t gpio_coarse_idx = _param_gpio_coarse_idx.get();  // Coarse limit (down)
        uint8_t gpio_fine_idx = _param_gpio_fine_idx.get();      // Fine limit (up)

        if (gpio_coarse_idx < sensor_gpio_s::GPIO_COUNT) {
            _limit_switch_coarse = gpio_state.levels & (1 << gpio_coarse_idx);
        }

        if (gpio_fine_idx < sensor_gpio_s::GPIO_COUNT) {
            _limit_switch_fine = gpio_state.levels & (1 << gpio_fine_idx);
        }
    }

    return _limit_switch_coarse || _limit_switch_fine;
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
