/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "traction_control.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

TractionControl::TractionControl() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
    // Initialize PID controllers for slip control
    _slip_controller_front.setGains(1.0f, 0.2f, 0.1f);
    _slip_controller_front.setOutputLimit(1.0f);
    _slip_controller_front.setIntegralLimit(1.0f);

    _slip_controller_rear.setGains(1.0f, 0.2f, 0.1f);
    _slip_controller_rear.setOutputLimit(1.0f);
    _slip_controller_rear.setIntegralLimit(1.0f);

    _yaw_rate_controller.setGains(1.2f, 0.1f, 0.2f);
    _yaw_rate_controller.setOutputLimit(1.0f);
    _yaw_rate_controller.setIntegralLimit(1.0f);
}

TractionControl::~TractionControl()
{
    perf_free(_loop_perf);
    perf_free(_slip_detect_perf);
    perf_free(_stall_detect_perf);
}

bool TractionControl::init()
{
    ScheduleOnInterval(1000000 / UPDATE_RATE_HZ); // 50Hz
    return true;
}

void TractionControl::Run()
{
    if (should_exit()) {
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);

    // Update all state information
    update_vehicle_state();
    update_motor_feedback();

    // Check if traction control should be active
    vehicle_control_mode_s control_mode;
    if (_vehicle_control_mode_sub.copy(&control_mode)) {
        _traction_control_active = control_mode.flag_control_velocity_enabled &&
                                   _param_enable.get();
    }

    if (!_traction_control_active) {
        perf_end(_loop_perf);
        return;
    }

    // Get desired motion from manual control or autonomous planner
    // For now, use default values or implement alternative input method
    // TODO: Replace with appropriate input source (manual_control_setpoint, trajectory_setpoint, etc.)
    _desired_velocity = 0.0f;  // Default to stationary
    _desired_yaw_rate = 0.0f;  // Default to no turning
    _desired_force = 0.0f;     // Default to no force

    // Core traction control pipeline
    estimate_axle_slip();
    estimate_ground_conditions();

    // Compute velocity commands based on slip control
    compute_velocity_commands();

    // Compute force distribution between axles
    compute_force_distribution();

    // Compute steering compensation for slip
    compute_steering_compensation();

    // Check for wheel stall conditions
    detect_wheel_stall();

    // Handle excessive slip events
    if (_front_axle.is_slipping || _rear_axle.is_slipping) {
        handle_excessive_slip();
    }

    // Publish commands to wheel and steering controllers
    publish_wheel_commands();
    publish_steering_command();

    _last_update = hrt_absolute_time();

    perf_end(_loop_perf);
}

void TractionControl::update_vehicle_state()
{
    // Get velocity from EKF
    vehicle_local_position_s local_pos;
    if (_vehicle_local_position_sub.copy(&local_pos)) {
        _vehicle.velocity_body(0) = local_pos.vx;
        _vehicle.velocity_body(1) = local_pos.vy;
        _vehicle.velocity_body(2) = local_pos.vz;

        _vehicle.ground_speed = sqrtf(local_pos.vx * local_pos.vx +
                                      local_pos.vy * local_pos.vy);
    }

    // Get acceleration
    vehicle_acceleration_s accel;
    if (_vehicle_acceleration_sub.copy(&accel)) {
        _vehicle.acceleration_body(0) = accel.xyz[0];
        _vehicle.acceleration_body(1) = accel.xyz[1];
        _vehicle.acceleration_body(2) = accel.xyz[2];
    }

    // Get angular velocity
    vehicle_angular_velocity_s angular_vel;
    if (_vehicle_angular_velocity_sub.copy(&angular_vel)) {
        _vehicle.yaw_rate = angular_vel.xyz[2];
    }

    // Calculate sideslip angle
    if (_vehicle.ground_speed > MIN_GROUND_SPEED) {
        _vehicle.sideslip_angle = atan2f(_vehicle.velocity_body(1),
                                         _vehicle.velocity_body(0));
    }
}

void TractionControl::update_motor_feedback()
{
    // Get motor speeds from encoders
    actuator_motors_s motors;
    if (_actuator_motors_sub.copy(&motors)) {
        // Convert RPM to rad/s
        _front_axle.motor_speed = motors.rpm[0] * (2.0f * M_PI_F / 60.0f);
        _rear_axle.motor_speed = motors.rpm[1] * (2.0f * M_PI_F / 60.0f);

        // Convert to wheel speed
        float gear_ratio = _param_gear_ratio.get();
        _front_axle.wheel_speed = _front_axle.motor_speed / gear_ratio;
        _rear_axle.wheel_speed = _rear_axle.motor_speed / gear_ratio;
    }
}

void TractionControl::estimate_axle_slip()
{
    // Estimate slip for each axle
    _front_axle.slip = calculate_axle_slip(
        _front_axle.wheel_speed,
        _vehicle.ground_speed,
        _vehicle.articulation_angle,
        true
    );

    _rear_axle.slip = calculate_axle_slip(
        _rear_axle.wheel_speed,
        _vehicle.ground_speed,
        _vehicle.articulation_angle,
        false
    );

    // Determine if slipping
    float slip_threshold = _param_max_slip.get();

    if (fabsf(_front_axle.slip.longitudinal) > slip_threshold) {
        _front_axle.is_slipping = true;
        perf_count(_slip_detect_perf);
        _last_slip_event = hrt_absolute_time();
    } else {
        _front_axle.is_slipping = false;
    }

    if (fabsf(_rear_axle.slip.longitudinal) > slip_threshold) {
        _rear_axle.is_slipping = true;
        perf_count(_slip_detect_perf);
        _last_slip_event = hrt_absolute_time();
    } else {
        _rear_axle.is_slipping = false;
    }
}

TractionControl::SlipEstimate TractionControl::calculate_axle_slip(
    float wheel_speed, float vehicle_speed, float articulation_angle, bool is_front_axle)
{
    SlipEstimate slip;
    slip.timestamp = hrt_absolute_time();

    // Calculate wheel linear velocity
    float wheel_linear_speed = wheel_speed * _param_wheel_radius.get();

    // Adjust for articulated steering kinematics
    float effective_speed = vehicle_speed;
    if (fabsf(articulation_angle) > 0.01f && vehicle_speed > MIN_GROUND_SPEED) {
        float wheelbase = _param_wheelbase.get();
        float turn_radius = wheelbase / tanf(articulation_angle);

        if (is_front_axle) {
            // Front axle outer radius adjustment
            effective_speed *= (1.0f + wheelbase / (2.0f * fabsf(turn_radius)));
        } else {
            // Rear axle inner radius adjustment
            effective_speed *= (1.0f - wheelbase / (2.0f * fabsf(turn_radius)));
        }
    }

    // Calculate longitudinal slip
    if (effective_speed > MIN_GROUND_SPEED) {
        slip.longitudinal = (wheel_linear_speed - effective_speed) / effective_speed;
    } else if (wheel_linear_speed > MIN_GROUND_SPEED) {
        slip.longitudinal = 1.0f; // Pure spinning
    } else {
        slip.longitudinal = 0.0f;
    }

    // Constrain slip
    slip.longitudinal = math::constrain(slip.longitudinal, -1.0f, 1.0f);

    // Estimate lateral slip
    if (is_front_axle) {
        slip.lateral = _vehicle.sideslip_angle + articulation_angle / 2.0f;
    } else {
        slip.lateral = _vehicle.sideslip_angle - articulation_angle / 2.0f;
    }

    // Confidence based on speed
    slip.confidence = math::constrain(vehicle_speed / 1.0f, 0.1f, 1.0f);

    return slip;
}

void TractionControl::estimate_ground_conditions()
{
    // Estimate surface type from slip behavior
    float avg_slip = (fabsf(_front_axle.slip.longitudinal) +
                     fabsf(_rear_axle.slip.longitudinal)) / 2.0f;

    // Surface estimation based on slip characteristics
    if (avg_slip < 0.05f) {
        _surface_type = SurfaceType::HARD_ROCK;
        _estimated_friction = 0.9f;
    } else if (avg_slip < 0.15f) {
        _surface_type = SurfaceType::GRAVEL;
        _estimated_friction = 0.7f;
    } else if (avg_slip < 0.3f) {
        _surface_type = SurfaceType::LOOSE_SOIL;
        _estimated_friction = 0.5f;
    } else {
        _surface_type = SurfaceType::MUD;
        _estimated_friction = 0.3f;
    }
}

void TractionControl::compute_velocity_commands()
{
    // Base velocity from desired speed
    float base_velocity = _desired_velocity;

    // Target slip from parameters
    float target_slip = _param_target_slip.get();

    // Compute slip errors
    float front_slip_error = target_slip - _front_axle.slip.longitudinal;
    float rear_slip_error = target_slip - _rear_axle.slip.longitudinal;

    // PID control for slip regulation
    float dt = (_last_update > 0) ? (hrt_absolute_time() - _last_update) * 1e-6f : 0.02f;

    float front_correction = _slip_controller_front.update(front_slip_error, dt);
    float rear_correction = _slip_controller_rear.update(rear_slip_error, dt);

    // Apply corrections to velocity commands
    _front_velocity_cmd = base_velocity * (1.0f + front_correction);
    _rear_velocity_cmd = base_velocity * (1.0f + rear_correction);

    // Limit velocity difference between axles for stability
    float max_diff = base_velocity * 0.2f; // 20% maximum difference
    float diff = _front_velocity_cmd - _rear_velocity_cmd;
    if (fabsf(diff) > max_diff) {
        float avg = (_front_velocity_cmd + _rear_velocity_cmd) / 2.0f;
        _front_velocity_cmd = avg + max_diff * (diff > 0 ? 0.5f : -0.5f);
        _rear_velocity_cmd = avg - max_diff * (diff > 0 ? 0.5f : -0.5f);
    }
}

void TractionControl::compute_force_distribution()
{
    // Start with default distribution
    float front_ratio = _param_force_distribution.get();

    // Adjust for slip conditions
    if (_front_axle.is_slipping && !_rear_axle.is_slipping) {
        // Reduce front force
        front_ratio = fmaxf(0.2f, front_ratio - 0.2f);
    } else if (_rear_axle.is_slipping && !_front_axle.is_slipping) {
        // Reduce rear force
        front_ratio = fminf(0.8f, front_ratio + 0.2f);
    }

    // Apply distribution
    _front_force_cmd = _desired_force * front_ratio;
    _rear_force_cmd = _desired_force * (1.0f - front_ratio);

    // Apply maximum force limits
    limit_wheel_forces();
}

void TractionControl::limit_wheel_forces()
{
    // Apply maximum force limits from parameters
    float max_force = _param_max_force.get();
    _front_force_cmd = math::constrain(_front_force_cmd, -max_force, max_force);
    _rear_force_cmd = math::constrain(_rear_force_cmd, -max_force, max_force);
}

void TractionControl::compute_steering_compensation()
{
    // Calculate desired articulation for trajectory following
    float desired_articulation = calculate_kinematic_steering_rate(_desired_yaw_rate);

    // Use yaw rate controller to maintain desired yaw rate
    float dt = (_last_update > 0) ? (hrt_absolute_time() - _last_update) * 1e-6f : 0.02f;
    float yaw_rate_error = _desired_yaw_rate - _vehicle.yaw_rate;
    float yaw_compensation = _yaw_rate_controller.update(yaw_rate_error, dt);

    // Combine compensations
    _articulation_cmd = desired_articulation + yaw_compensation;

    // Apply limits
    _articulation_cmd = math::constrain(_articulation_cmd,
                                        -_param_max_articulation.get(),
                                        _param_max_articulation.get());

    // Rate limiting for smooth operation
    static float last_cmd = 0.0f;
    float rate = (_articulation_cmd - last_cmd) / dt;
    if (fabsf(rate) > ARTICULATION_RATE_LIMIT) {
        _articulation_cmd = last_cmd + ARTICULATION_RATE_LIMIT * dt * (rate > 0 ? 1.0f : -1.0f);
    }
    last_cmd = _articulation_cmd;
}

float TractionControl::calculate_kinematic_steering_rate(float desired_yaw_rate)
{
    float wheelbase = _param_wheelbase.get();

    if (_vehicle.ground_speed > MIN_GROUND_SPEED) {
        return atanf(wheelbase * desired_yaw_rate / _vehicle.ground_speed);
    } else {
        return 0.0f;
    }
}

void TractionControl::detect_wheel_stall()
{
    // Detect if wheels are stalled (not rotating despite commands)
    bool front_stalled = (_front_velocity_cmd > MIN_GROUND_SPEED &&
                          _front_axle.wheel_speed < _param_stall_threshold.get());
    bool rear_stalled = (_rear_velocity_cmd > MIN_GROUND_SPEED &&
                         _rear_axle.wheel_speed < _param_stall_threshold.get());

    if (front_stalled || rear_stalled) {
        perf_count(_stall_detect_perf);
        _last_stall_event = hrt_absolute_time();

        // Reduce force commands if stalled
        if (front_stalled) {
            _front_force_cmd *= 0.5f;
            PX4_WARN("Front wheel stall detected");
        }
        if (rear_stalled) {
            _rear_force_cmd *= 0.5f;
            PX4_WARN("Rear wheel stall detected");
        }
    }
}

void TractionControl::handle_excessive_slip()
{
    // Handle excessive slip events

    if (_front_axle.is_slipping) {
        // Reduce front velocity and force
        _front_velocity_cmd *= 0.8f;
        _front_force_cmd *= 0.7f;
    }

    if (_rear_axle.is_slipping) {
        // Reduce rear velocity and force
        _rear_velocity_cmd *= 0.8f;
        _rear_force_cmd *= 0.7f;
    }

    // If both slipping, reduce overall commands
    if (_front_axle.is_slipping && _rear_axle.is_slipping) {
        _front_velocity_cmd *= 0.7f;
        _rear_velocity_cmd *= 0.7f;
        _front_force_cmd *= 0.6f;
        _rear_force_cmd *= 0.6f;
    }
}

void TractionControl::publish_wheel_commands()
{
    // Publish wheel setpoints for front and rear wheels

    // Front wheel setpoint
    wheel_setpoint_s front_setpoint{};
    front_setpoint.timestamp = hrt_absolute_time();
    front_setpoint.wheel_speed_rad_s = _front_velocity_cmd / _param_wheel_radius.get(); // Convert m/s to rad/s
    front_setpoint.wheel_acceleration_rad_s2 = 2.0f; // Default acceleration limit
    front_setpoint.wheel_torque_nm = _front_force_cmd * _param_wheel_radius.get(); // Convert force to torque
    front_setpoint.control_mode = wheel_setpoint_s::MODE_SPEED_CONTROL;
    front_setpoint.enable_traction_control = _traction_control_active;
    front_setpoint.emergency_stop = false;
    front_setpoint.speed_limit_rad_s = _param_max_speed.get();
    front_setpoint.torque_limit_nm = _param_max_force.get() * _param_wheel_radius.get();

    // Rear wheel setpoint
    wheel_setpoint_s rear_setpoint{};
    rear_setpoint.timestamp = hrt_absolute_time();
    rear_setpoint.wheel_speed_rad_s = _rear_velocity_cmd / _param_wheel_radius.get(); // Convert m/s to rad/s
    rear_setpoint.wheel_acceleration_rad_s2 = 2.0f; // Default acceleration limit
    rear_setpoint.wheel_torque_nm = _rear_force_cmd * _param_wheel_radius.get(); // Convert force to torque
    rear_setpoint.control_mode = wheel_setpoint_s::MODE_SPEED_CONTROL;
    rear_setpoint.enable_traction_control = _traction_control_active;
    rear_setpoint.emergency_stop = false;
    rear_setpoint.speed_limit_rad_s = _param_max_speed.get();
    rear_setpoint.torque_limit_nm = _param_max_force.get() * _param_wheel_radius.get();

    // Publish setpoints
    _wheel_setpoint_front_pub.publish(front_setpoint);
    _wheel_setpoint_rear_pub.publish(rear_setpoint);
}

void TractionControl::publish_steering_command()
{
    // Publish steering command
    steering_setpoint_s steering_sp{};
    steering_sp.timestamp = hrt_absolute_time();
    steering_sp.steering_angle_rad = _articulation_cmd;
    steering_sp.steering_rate_rad_s = _desired_yaw_rate; // Feed-forward

    _steering_setpoint_pub.publish(steering_sp);
}

float TractionControl::estimate_friction_coefficient()
{
    return _estimated_friction;
}

int TractionControl::task_spawn(int argc, char *argv[])
{
    TractionControl *instance = new TractionControl();

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

int TractionControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int TractionControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Traction Control Module for Articulated Wheel Loader

This module provides slip-based traction control optimized for mining operations:
- Estimates slip for front/rear axles using motor encoders and EKF velocity
- Provides steering compensation for articulated vehicle
- Optimizes traction and stability for maximum performance

The module acts as an intermediary between control inputs and actuators,
optimizing traction based on slip estimation and ground conditions.

### Implementation
Currently configured to work with manual control or autonomous planner inputs.
Publishes steering commands to steering controller.
Note: Direct wheel control implementation needs to be added based on specific actuator setup.

### Examples
Start with default parameters:
$ traction_control start

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("traction_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int traction_control_main(int argc, char *argv[])
{
    return TractionControl::main(argc, argv);
}
