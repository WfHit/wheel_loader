/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

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
    // Initialize PID controllers
    _position_pid.setGains(_param_pid_p.get(), _param_pid_i.get(), _param_pid_d.get());
    _velocity_pid.setGains(_param_vel_pid_p.get(), _param_vel_pid_i.get(), _param_vel_pid_d.get());

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

    // Initialize boom angle monitoring
    _current_boom_angle = 0.0f;
    _target_absolute_bucket_angle = 0.0f;  // Start with horizontal bucket
    _boom_angle_changed = false;

    // Get limit sensor instance IDs
    uint8_t load_instance = _param_limit_load_idx.get();     // Load limit (bucket down)
    uint8_t dump_instance = _param_limit_dump_idx.get();     // Dump limit (bucket up)

    PX4_INFO("Using limit sensors: load=%d, dump=%d", load_instance, dump_instance);
    PX4_INFO("Boom compensation mode enabled - bucket will maintain absolute angle");

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

bool BucketControl::solveBucketLinkage(
    float actuator_length,
    float boom_angle,
    float &bucket_angle,
    float &bellcrank_angle,
    float &coupler_angle)
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
        // Control mode is fixed to BOOM_COMPENSATED
    }

    // Read sensors using EKF2-style instance discovery
    updateEncoderData();
    updateHBridgeStatus();
    checkLimitSwitches();

    // Monitor boom angle changes and update bucket compensation
    updateBoomAngleMonitoring();

    // Update current bucket angle for status
    _current_bucket_angle = actuatorLengthToBucketAngle(_current_actuator_length, _current_boom_angle);

    // Clean control architecture - 3 separate functions
    monitorCommandTarget();     // Function 1: Monitor command target changes
    monitorBoomChanges();       // Function 2: Monitor boom angle changes
    updateActuatorTarget();     // Function 3: Calculate actuator target from both sources

    // Update state machine
    updateStateMachine();

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
                _encoder_zero_offset = 0.0f;  // Reset since encoder driver will reset to 0
                _current_actuator_length = _kinematics.actuator_max_length;
                _zeroing_state = ZeroingState::COMPLETE;
                _zeroing_complete = true;

                PX4_INFO("Zeroing complete at dump limit - encoder reset command sent for instance %ld",
                         static_cast<long>(_param_encoder_index.get()));
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

// Clean Control Architecture Implementation

void BucketControl::monitorCommandTarget()
{
    // Function 1: Monitor bucket command target changes
    bucket_command_s cmd;
    if (_bucket_command_sub.update(&cmd)) {
        // New command received - update target absolute bucket angle
        _target_absolute_bucket_angle = cmd.target_angle;

        PX4_DEBUG("New bucket command: target=%.2f°",
                 static_cast<double>(math::degrees(_target_absolute_bucket_angle)));
    }
}

void BucketControl::monitorBoomChanges()
{
    // Function 2: Monitor boom angle changes (already done in updateBoomAngleMonitoring)
    // The boom angle monitoring is handled by updateBoomAngleMonitoring()
    // which sets _boom_angle_changed flag when boom moves significantly

    if (_boom_angle_changed) {
        PX4_DEBUG("Boom angle changed to %.2f°",
                 static_cast<double>(math::degrees(_current_boom_angle)));
    }
}

void BucketControl::updateActuatorTarget()
{
    // Function 3: Calculate actuator target from command target and boom angle
    // This function consolidates both command changes and boom compensation

    // Calculate required actuator length for current absolute bucket angle and boom position
    float target_bucket_relative_angle = _target_absolute_bucket_angle - _current_boom_angle;
    float new_target_actuator_length = bucketAngleToActuatorLength(target_bucket_relative_angle, _current_boom_angle);

    // Only update if the change is significant (avoid small oscillations)
    float actuator_delta = fabsf(new_target_actuator_length - _target_actuator_length);
    if (actuator_delta > 0.5f) { // 0.5mm threshold
        _target_actuator_length = new_target_actuator_length;

        PX4_DEBUG("Actuator target updated: absolute=%.2f°, relative=%.2f°, actuator=%.1fmm",
                 static_cast<double>(math::degrees(_target_absolute_bucket_angle)),
                 static_cast<double>(math::degrees(target_bucket_relative_angle)),
                 static_cast<double>(_target_actuator_length));
    }

    // Reset boom change flag after processing
    _boom_angle_changed = false;
}

void BucketControl::updateMotionControl()
{
    // Check limits: load limit prevents downward motion, dump limit prevents upward motion
    if ((_limit_switch_load && _control_output < 0) ||   // Load limit active, blocking downward
        (_limit_switch_dump && _control_output > 0)) {   // Dump limit active, blocking upward
        setMotorCommand(0.0f);
        return;
    }

    float dt = 0.01f; // 100Hz control loop

    // Step 1: Use smoother to generate smooth trajectory
    // Configure motion planning constraints
    _position_smoother.setMaxAccelerationZ(_param_max_acceleration.get());
    _position_smoother.setMaxVelocityZ(_param_max_velocity.get());
    _position_smoother.setMaxJerkZ(_param_jerk_limit.get());

    // Generate smooth trajectory using position smoother (1D motion using Z-axis)
    matrix::Vector3f current_pos{0.0f, 0.0f, _current_actuator_length};
    matrix::Vector3f target_pos{0.0f, 0.0f, _target_actuator_length};
    matrix::Vector3f feedforward_velocity{0.0f, 0.0f, 0.0f};

    PositionSmoothing::PositionSmoothingSetpoints setpoints;
    _position_smoother.generateSetpoints(current_pos, target_pos, feedforward_velocity,
                                       dt, false, setpoints);

    // Get smoothed trajectory setpoints from Z component
    float smooth_position_setpoint = setpoints.position(2);
    float smooth_velocity_setpoint = setpoints.velocity(2);

    // Step 2: Cascade PID control to follow the trajectory
    // Position PID tracks the smooth position setpoint and generates velocity command
    _position_pid.setSetpoint(smooth_position_setpoint);
    float velocity_command = _position_pid.update(_current_actuator_length, dt);

    // Add velocity feedforward from smoother
    velocity_command += smooth_velocity_setpoint;

    // Constrain velocity command to limits
    velocity_command = math::constrain(velocity_command,
                                     -_param_max_velocity.get(),
                                     _param_max_velocity.get());

    // Step 3: Velocity PID follows velocity command and generates motor output
    _velocity_pid.setSetpoint(velocity_command);
    _control_output = _velocity_pid.update(_current_velocity, dt);

    // Apply control output
    setMotorCommand(_control_output);

    // Update state
    float position_error = _target_actuator_length - _current_actuator_length;
    _state = (fabsf(position_error) < 2.0f) ? State::READY : State::MOVING; // 2mm tolerance
}

void BucketControl::setMotorCommand(float command)
{
    // Publish HBridge command for bucket motor (EKF2 pattern)
    hbridge_command_s cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.instance = _motor_index;  // Instance info embedded in message data
    cmd.duty_cycle = command;     // Use command directly as duty_cycle (-1.0 to 1.0)
    cmd.enable = true;

    // Publish using EKF2 pattern - no instance parameter needed
    _hbridge_command_pub.publish(cmd);
}

void BucketControl::readEncoderFeedback()
{
    // Read from sensor_quad_encoder topic published by quad encoder driver
    uint8_t encoder_idx = _param_encoder_index.get();

    // Check if we have a valid encoder instance
    if (encoder_idx < _sensor_quad_encoder_sub.size()) {
        sensor_quad_encoder_s encoder_data;
        if (_sensor_quad_encoder_sub[encoder_idx].update(&encoder_data)) {
            // Verify this is the correct instance
            if (encoder_data.instance == encoder_idx) {
                // Get actuator length directly from encoder position
                // Position is in 1/million rad, convert to length using scale
                float position_rad = static_cast<float>(encoder_data.position) * 1e-6f;
                _current_actuator_length = (position_rad - _encoder_zero_offset) * _param_encoder_scale.get() + _kinematics.actuator_min_length;

                // Get velocity directly from encoder velocity
                // Velocity is in 1/million rad/s, convert to mm/s
                float velocity_rad_s = static_cast<float>(encoder_data.velocity) * 1e-6f;
                _current_velocity = velocity_rad_s * _param_encoder_scale.get();

                _last_encoder_time = encoder_data.timestamp;
            }
        }
    }
}

void BucketControl::updateEncoderData()
{
    sensor_quad_encoder_s encoder_data;

    // If no specific instance selected, find our encoder's instance
    if (_encoder_selected < 0) {
        const hrt_abstime timestamp_stale = math::max(hrt_absolute_time(), 100_ms) - 100_ms;
        uint8_t target_encoder_idx = _param_encoder_index.get();

        if (_sensor_quad_encoder_sub.advertised()) {
            for (unsigned i = 0; i < _sensor_quad_encoder_sub.size(); i++) {
                if (_sensor_quad_encoder_sub[i].update(&encoder_data)) {
                    // Check if this is our encoder's data
                    if ((encoder_data.timestamp != 0) &&
                        (encoder_data.timestamp > timestamp_stale) &&
                        (encoder_data.instance == target_encoder_idx)) {

                        int nencoder = orb_group_count(ORB_ID(sensor_quad_encoder));
                        if (nencoder > 1) {
                            PX4_INFO("Bucket control selected encoder:%d (instance %d, %d advertised)",
                                     i, target_encoder_idx, nencoder);
                        }

                        _encoder_selected = i;
                        _last_encoder_update = encoder_data.timestamp;
                        break;
                    }
                }
            }
        }
    }

    // Use the selected instance
    if (_encoder_selected >= 0 &&
        _sensor_quad_encoder_sub[_encoder_selected].update(&encoder_data)) {

        if (encoder_data.instance == _param_encoder_index.get()) {
            // Process our encoder's data
            _last_encoder_update = encoder_data.timestamp;

            // Get actuator length directly from encoder position
            // Position is in 1/million rad, convert to length using scale
            float position_rad = static_cast<float>(encoder_data.position) * 1e-6f;
            _current_actuator_length = (position_rad - _encoder_zero_offset) * _param_encoder_scale.get() + _kinematics.actuator_min_length;

            // Get velocity directly from encoder velocity
            // Velocity is in 1/million rad/s, convert to mm/s
            float velocity_rad_s = static_cast<float>(encoder_data.velocity) * 1e-6f;
            _current_velocity = velocity_rad_s * _param_encoder_scale.get();

            _last_encoder_time = encoder_data.timestamp;
        }
    }
}

bool BucketControl::checkLimitSwitches()
{
    limit_sensor_s limit_sensor_data;

    // If no specific instances selected, find our limit sensor instances
    if (_limit_load_selected < 0 || _limit_dump_selected < 0) {
        const hrt_abstime timestamp_stale = math::max(hrt_absolute_time(), 100_ms) - 100_ms;
        uint8_t target_load_idx = _param_limit_load_idx.get();
        uint8_t target_dump_idx = _param_limit_dump_idx.get();

        if (_limit_sensor_sub.advertised()) {
            for (unsigned i = 0; i < _limit_sensor_sub.size(); i++) {
                if (_limit_sensor_sub[i].update(&limit_sensor_data)) {
                    // Check if this is our load limit sensor's data
                    if ((limit_sensor_data.timestamp != 0) &&
                        (limit_sensor_data.timestamp > timestamp_stale) &&
                        (limit_sensor_data.instance == target_load_idx) &&
                        (_limit_load_selected < 0)) {

                        int nlimit = orb_group_count(ORB_ID(limit_sensor));
                        if (nlimit > 1) {
                            PX4_INFO("Bucket control selected load limit sensor:%d (instance %d, %d advertised)",
                                     i, target_load_idx, nlimit);
                        }

                        _limit_load_selected = i;
                        _last_limit_load_update = limit_sensor_data.timestamp;
                    }

                    // Check if this is our dump limit sensor's data
                    if ((limit_sensor_data.timestamp != 0) &&
                        (limit_sensor_data.timestamp > timestamp_stale) &&
                        (limit_sensor_data.instance == target_dump_idx) &&
                        (_limit_dump_selected < 0)) {

                        int nlimit = orb_group_count(ORB_ID(limit_sensor));
                        if (nlimit > 1) {
                            PX4_INFO("Bucket control selected dump limit sensor:%d (instance %d, %d advertised)",
                                     i, target_dump_idx, nlimit);
                        }

                        _limit_dump_selected = i;
                        _last_limit_dump_update = limit_sensor_data.timestamp;
                    }
                }
            }
        }
    }

    // Use the selected load limit instance
    if (_limit_load_selected >= 0 &&
        _limit_sensor_sub[_limit_load_selected].update(&limit_sensor_data)) {

        if (limit_sensor_data.instance == _param_limit_load_idx.get()) {
            // Process our load limit sensor's data
            _last_limit_load_update = limit_sensor_data.timestamp;
            _limit_switch_load = limit_sensor_data.state;
        }
    }

    // Use the selected dump limit instance
    if (_limit_dump_selected >= 0 &&
        _limit_sensor_sub[_limit_dump_selected].update(&limit_sensor_data)) {

        if (limit_sensor_data.instance == _param_limit_dump_idx.get()) {
            // Process our dump limit sensor's data
            _last_limit_dump_update = limit_sensor_data.timestamp;
            _limit_switch_dump = limit_sensor_data.state;
        }
    }

    return _limit_switch_load || _limit_switch_dump;
}

bool BucketControl::checkHBridgeStatus()
{
    // Check hbridge status for our motor instance using EKF2-style SubscriptionMultiArray
    hbridge_status_s hbridge_status;
    uint8_t hbridge_channel = static_cast<uint8_t>(_param_motor_index.get());

    if (hbridge_channel < _hbridge_status_sub.size()) {
        if (_hbridge_status_sub[hbridge_channel].update(&hbridge_status)) {
            // Verify this is the correct instance
            if (hbridge_status.instance == hbridge_channel) {
                // Update our status based on hbridge feedback
                // Could use this for fault detection, current monitoring, etc.
                return hbridge_status.enabled;
            }
        }
    }
    return false;  // No status received or not enabled
}

void BucketControl::updateHBridgeStatus()
{
    hbridge_status_s hbridge_status;

    // If no specific instance selected, find our motor's instance
    if (_hbridge_status_selected < 0) {
        const hrt_abstime timestamp_stale = math::max(hrt_absolute_time(), 100_ms) - 100_ms;

        if (_hbridge_status_sub.advertised()) {
            for (unsigned i = 0; i < _hbridge_status_sub.size(); i++) {
                if (_hbridge_status_sub[i].update(&hbridge_status)) {
                    // Check if this is our motor's status
                    if ((hbridge_status.timestamp != 0) &&
                        (hbridge_status.timestamp > timestamp_stale) &&
                        (hbridge_status.instance == _motor_index)) {

                        int nstatus = orb_group_count(ORB_ID(hbridge_status));
                        if (nstatus > 1) {
                            PX4_INFO("Bucket control selected hbridge_status:%d (motor %d, %d advertised)",
                                     i, _motor_index, nstatus);
                        }

                        _hbridge_status_selected = i;
                        _last_hbridge_status_update = hbridge_status.timestamp;
                        break;
                    }
                }
            }
        }
    }

    // Use the selected instance
    if (_hbridge_status_selected >= 0 &&
        _hbridge_status_sub[_hbridge_status_selected].update(&hbridge_status)) {

        if (hbridge_status.instance == _motor_index) {
            // Process our motor's status
            _last_hbridge_status_update = hbridge_status.timestamp;

            // Check if hbridge is disabled - this could indicate a problem
            if (!hbridge_status.enabled) {
                PX4_WARN("HBridge motor %d is disabled", _motor_index);
            }

            // Update internal state if needed
            // _motor_current = hbridge_status.current;
            // _motor_voltage = hbridge_status.voltage;
        }
    }
}

// Boom Angle Monitoring and Compensation Methods

void BucketControl::updateBoomAngleMonitoring()
{
    // Get current boom angle from AS5600 magnetic encoder
    sensor_mag_encoder_s mag_encoder_data;
    float new_boom_angle = _current_boom_angle; // Default to previous value

    // Read the latest magnetic encoder data for boom angle
    if (_sensor_mag_encoder_sub.update(&mag_encoder_data)) {
        // Validate sensor readings
        if (mag_encoder_data.magnet_detected &&
            !mag_encoder_data.magnet_too_strong &&
            !mag_encoder_data.magnet_too_weak) {
            new_boom_angle = mag_encoder_data.angle;
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

    // Check if boom angle has changed significantly
    float boom_angle_delta = fabsf(new_boom_angle - _previous_boom_angle);
    _boom_angle_changed = (boom_angle_delta > _boom_angle_threshold);

    if (_boom_angle_changed) {
        PX4_DEBUG("Boom movement detected: %.3f° -> %.3f° (delta: %.3f°)",
                 static_cast<double>(math::degrees(_previous_boom_angle)),
                 static_cast<double>(math::degrees(new_boom_angle)),
                 static_cast<double>(math::degrees(boom_angle_delta)));
    }

    // Update boom angle tracking
    _previous_boom_angle = _current_boom_angle;
    _current_boom_angle = new_boom_angle;
}

void BucketControl::compensateForBoomMovement()
{
    // When boom moves, adjust bucket actuator to maintain the same absolute bucket angle
    // _target_absolute_bucket_angle remains constant
    // We need to recalculate the required actuator length for the new boom position

    float target_bucket_relative_angle = _target_absolute_bucket_angle - _current_boom_angle;
    float new_target_actuator_length = bucketAngleToActuatorLength(target_bucket_relative_angle, _current_boom_angle);

    // Only update if the change is significant (avoid small oscillations)
    float actuator_delta = fabsf(new_target_actuator_length - _target_actuator_length);
    if (actuator_delta > 1.0f) { // 1mm threshold
        _target_actuator_length = new_target_actuator_length;

        PX4_DEBUG("Boom compensation: absolute=%.2f°, relative=%.2f°, actuator=%.1fmm",
                 static_cast<double>(math::degrees(_target_absolute_bucket_angle)),
                 static_cast<double>(math::degrees(target_bucket_relative_angle)),
                 static_cast<double>(_target_actuator_length));
    }

    // Reset the boom change flag
    _boom_angle_changed = false;
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
    status.velocity = _current_velocity;
    status.control_output = _control_output;
    status.limit_switch_coarse = _limit_switch_load;     // Load limit switch status
    status.limit_switch_fine = _limit_switch_dump;       // Dump limit switch status
    status.zeroing_complete = _zeroing_complete;

    // Control mode status (simplified) - fixed to boom compensation mode
    status.control_mode = 0;  // Always boom compensation mode
    status.target_ground_angle = _target_absolute_bucket_angle;

    // Boom tracking status (reuse existing fields for boom angle monitoring)
    status.machine_pitch = _current_boom_angle;  // Use machine_pitch field to report boom angle
    status.anti_spill_active = _boom_angle_changed;  // Use anti_spill_active to report boom movement

    _bucket_status_pub.publish(status);
}
