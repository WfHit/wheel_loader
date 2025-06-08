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

#include "WheelLoaderController.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

WheelLoaderController::WheelLoaderController() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
    // Initialize performance counters
    _loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME ": loop");
    _control_latency_perf = perf_alloc(PC_ELAPSED, MODULE_NAME ": control latency");

    // Initialize system status
    _vehicle_status.timestamp = hrt_absolute_time();
    _vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
    _vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
    _vehicle_status.system_type = vehicle_status_s::VEHICLE_TYPE_GROUND_ROVER;

    // Initialize module status
    _module_status.timestamp = hrt_absolute_time();
    _module_status.module_id = 0; // Wheel loader controller ID
    _module_status.health = module_status_s::HEALTH_OK;
    _module_status.arming_state = module_status_s::ARMING_STATE_DISARMED;
    _module_status.operational_state = module_status_s::STATE_STANDBY;

    // Initialize control outputs
    _actuator_motors = {};
    _actuator_servos = {};
    _boom_command = {};
    _bucket_command = {};
}

WheelLoaderController::~WheelLoaderController()
{
    perf_free(_loop_perf);
    perf_free(_control_latency_perf);
}

bool WheelLoaderController::init()
{
    if (!_vehicle_command_sub.registerCallback()) {
        PX4_ERR("vehicle_command registration failed");
        return false;
    }

    if (!_manual_control_sub.registerCallback()) {
        PX4_ERR("manual_control registration failed");
        return false;
    }

    // Schedule the first iteration
    ScheduleOnInterval(MAIN_LOOP_INTERVAL_US);

    _system_state = SystemState::STANDBY;

    PX4_INFO("Wheel Loader Controller initialized");
    return true;
}

void WheelLoaderController::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);

    // Update all subscriptions
    updateVehicleCommands();
    updateManualControls();
    updateChassisStatus();
    updateBoomBucketStatus();
    updateSafetyStatus();
    updateParameters();

    // Update AHRS-enhanced bucket control monitoring
    updateBucketControlForTerrain();

    // Perform safety checks
    performSafetyChecks();

    // Execute control logic based on current mode
    switch (_control_mode) {
    case ControlMode::MANUAL:
        processManualControl();
        break;
    case ControlMode::SEMI_AUTO:
        processSemiAutoControl();
        break;
    case ControlMode::AUTO:
        processAutoControl();
        break;
    case ControlMode::EMERGENCY_STOP:
        processEmergencyStop();
        break;
    }

    // Log AHRS status periodically
    logAHRSStatus();

    // Publish system status periodically
    if (hrt_elapsed_time(&_module_status.timestamp) > STATUS_PUBLISH_INTERVAL_US) {
        publishSystemStatus();
    }

    perf_end(_loop_perf);
}

void WheelLoaderController::updateVehicleCommands()
{
    vehicle_command_s vehicle_command;

    if (_vehicle_command_sub.update(&vehicle_command)) {
        vehicle_command_ack_s ack{};
        ack.timestamp = hrt_absolute_time();
        ack.command = vehicle_command.command;
        ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;

        switch (vehicle_command.command) {
        case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if (vehicle_command.param1 > 0.5f) {
                // Arm request
                if (allSubsystemsReady()) {
                    _armed = true;
                    _vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
                    _module_status.arming_state = module_status_s::ARMING_STATE_ARMED;
                    PX4_INFO("Wheel loader armed");
                } else {
                    ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
                    PX4_WARN("Arm denied - subsystems not ready");
                }
            } else {
                // Disarm request
                _armed = false;
                _vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
                _module_status.arming_state = module_status_s::ARMING_STATE_DISARMED;
                resetControllers();
                PX4_INFO("Wheel loader disarmed");
            }
            break;

        case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
            // Handle mode changes
            if (vehicle_command.param1 == 1.0f) { // Manual mode
                _control_mode = ControlMode::MANUAL;
                _vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
            } else if (vehicle_command.param1 == 2.0f) { // Auto mode
                if (_param_auto_enabled.get()) {
                    _control_mode = ControlMode::AUTO;
                    _vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
                    _work_cycle_state = WorkCycleState::IDLE;
                } else {
                    ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
                }
            }
            break;

        case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
            // Emergency stop
            _control_mode = ControlMode::EMERGENCY_STOP;
            _emergency_stop_active = true;
            handleEmergencyStop();
            break;

        default:
            ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
            break;
        }

        _vehicle_command_ack_pub.publish(ack);
    }
}

void WheelLoaderController::updateManualControls()
{
    manual_control_setpoint_s manual_control;

    if (_manual_control_sub.update(&manual_control)) {
        _last_manual_control = hrt_absolute_time();

        if (_armed && _control_mode == ControlMode::MANUAL) {
            controlChassis(manual_control);
            controlBoomBucket(manual_control);
        }
    }
}

void WheelLoaderController::updateChassisStatus()
{
    wheel_encoder_s wheel_encoder;

    if (_wheel_encoder_sub.update(&wheel_encoder)) {
        _chassis_state.front_wheel_velocity = wheel_encoder.front_velocity;
        _chassis_state.rear_wheel_velocity = wheel_encoder.rear_velocity;
        _chassis_state.last_update = hrt_absolute_time();
    }
}

void WheelLoaderController::updateBoomBucketStatus()
{
    boom_status_s boom_status;
    bucket_status_s bucket_status;

    if (_boom_status_sub.update(&boom_status)) {
        _boom_bucket_state.boom_angle = boom_status.angle;
        _boom_bucket_state.boom_velocity = boom_status.velocity;
        _boom_bucket_state.boom_load = boom_status.load;
        _boom_bucket_state.boom_ready = (boom_status.state == 2); // Ready state
        _boom_bucket_state.last_update = hrt_absolute_time();
    }

    if (_bucket_status_sub.update(&bucket_status)) {
        _boom_bucket_state.bucket_angle = bucket_status.current_angle;
        _boom_bucket_state.bucket_velocity = bucket_status.current_velocity;
        _boom_bucket_state.bucket_relative_angle = bucket_status.bucket_relative_angle;
        _boom_bucket_state.bucket_ready = (bucket_status.state == 2); // Ready state
        _boom_bucket_state.last_update = hrt_absolute_time();
    }
}

void WheelLoaderController::updateSafetyStatus()
{
    safety_s safety;

    if (_safety_sub.update(&safety)) {
        if (!safety.safety_switch_available || !safety.safety_off) {
            _emergency_stop_active = true;
            _control_mode = ControlMode::EMERGENCY_STOP;
        }
    }
}

void WheelLoaderController::updateParameters()
{
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();

        // Update work cycle parameters from module parameters
        _work_cycle_params.approach_speed = _param_work_cycle_speed.get() * 0.5f;
        _work_cycle_params.loading_speed = _param_work_cycle_speed.get() * 0.2f;
        _work_cycle_params.transport_speed = _param_work_cycle_speed.get();
        _work_cycle_params.load_detection_threshold = _param_load_threshold.get();
    }
}

void WheelLoaderController::controlChassis(const manual_control_setpoint_s &manual)
{
    perf_begin(_control_latency_perf);

    // Map manual control inputs to wheel velocities and steering
    float throttle = manual.throttle;
    float steering = manual.roll; // Use roll for steering

    // Apply velocity limits
    float max_velocity = _param_wheel_max_vel.get();
    float target_velocity = throttle * max_velocity;

    // Differential steering for articulated chassis
    float velocity_diff = steering * max_velocity * 0.3f;

    _actuator_motors.velocity[0] = math::constrain(target_velocity + velocity_diff, -max_velocity, max_velocity);
    _actuator_motors.velocity[1] = math::constrain(target_velocity - velocity_diff, -max_velocity, max_velocity);

    // Steering servo control
    float max_steering_angle = math::radians(_param_steer_max_angle.get());
    _actuator_servos.control[0] = math::constrain(steering * max_steering_angle, -max_steering_angle, max_steering_angle);

    publishChassisCommands();

    perf_end(_control_latency_perf);
}

void WheelLoaderController::controlBoomBucket(const manual_control_setpoint_s &manual)
{
    // Map manual control inputs to boom and bucket commands
    float boom_input = manual.pitch; // Use pitch for boom
    float bucket_input = manual.yaw;  // Use yaw for bucket

    // Boom control using boom_command messages
    _boom_command.timestamp = hrt_absolute_time();

    if (fabsf(boom_input) > 0.05f) { // Deadband to prevent noise
        // Velocity control mode for manual operation
        _boom_command.command_mode = 1; // Velocity mode
        _boom_command.target_angle = boom_input * _param_boom_max_vel.get(); // Using target_angle as velocity in velocity mode
        _boom_command.max_velocity = _param_boom_max_vel.get();
        _boom_command.max_load = 1.0f; // Maximum load capacity
        _boom_command.priority = 1; // Normal priority
        _boom_command.emergency_stop = false;
    } else {
        // Stop boom when no input
        _boom_command.command_mode = 1; // Velocity mode
        _boom_command.target_angle = 0.0f; // Zero velocity
        _boom_command.max_velocity = _param_boom_max_vel.get();
        _boom_command.max_load = 1.0f;
        _boom_command.priority = 1;
        _boom_command.emergency_stop = false;
    }

    // Enhanced bucket control with AHRS integration
    _bucket_command.timestamp = hrt_absolute_time();

    if (_control_mode == ControlMode::MANUAL) {
        // Manual mode - direct operator control
        _bucket_command.command_mode = 1; // Velocity mode for manual control
        _bucket_command.control_mode = bucket_command_s::MODE_MANUAL;
        _bucket_command.target_angle = bucket_input * _param_bucket_max_vel.get(); // Using as velocity
        _bucket_command.coordinate_frame = 0; // Ground reference
        _bucket_command.enable_stability_limit = true; // Always enable for safety
        _bucket_command.enable_anti_spill = false; // Not needed in manual mode
    } else {
        // Semi-auto or auto modes - use enhanced features
        _bucket_command.command_mode = 0; // Position mode for precise control
        _bucket_command.coordinate_frame = 0; // Ground reference
        _bucket_command.enable_stability_limit = true;
        _bucket_command.enable_anti_spill = (_work_cycle_state == WorkCycleState::TRANSPORT);

        // Set control mode based on work cycle state
        switch (_work_cycle_state) {
        case WorkCycleState::LOAD_MATERIAL:
            _bucket_command.control_mode = bucket_command_s::MODE_AUTO_LEVEL;
            _bucket_command.target_angle = _work_cycle_params.load_angle * M_PI / 180.0f;
            break;

        case WorkCycleState::TRANSPORT:
            _bucket_command.control_mode = bucket_command_s::MODE_TRANSPORT;
            _bucket_command.target_angle = _work_cycle_params.transport_angle * M_PI / 180.0f;
            _bucket_command.transport_angle = _work_cycle_params.carry_angle * M_PI / 180.0f;
            break;

        case WorkCycleState::DUMP_MATERIAL:
            _bucket_command.control_mode = bucket_command_s::MODE_GRADING;
            _bucket_command.target_angle = _work_cycle_params.dump_angle * M_PI / 180.0f;
            _bucket_command.grading_angle = _work_cycle_params.dump_angle * M_PI / 180.0f;
            break;

        default:
            _bucket_command.control_mode = bucket_command_s::MODE_MANUAL;
            _bucket_command.target_angle = bucket_input * _param_bucket_max_vel.get();
            break;
        }
    }

    _bucket_command.max_velocity = _param_bucket_max_vel.get();

    // Set NaN for parameters we want to use defaults for
    if (_bucket_command.control_mode != bucket_command_s::MODE_GRADING) {
        _bucket_command.grading_angle = NAN;
    }
    if (_bucket_command.control_mode != bucket_command_s::MODE_TRANSPORT) {
        _bucket_command.transport_angle = NAN;
    }
    _bucket_command.stability_threshold = NAN; // Use parameter default

    publishBoomBucketCommands();
}

void WheelLoaderController::processManualControl()
{
    // Check for control timeout
    if (hrt_elapsed_time(&_last_manual_control) > CONTROL_TIMEOUT_S * 1e6) {
        // Stop all motion on control timeout
        resetControllers();
        return;
    }

    _system_state = SystemState::OPERATION;
    _module_status.operational_state = module_status_s::STATE_ACTIVE;
}

void WheelLoaderController::processSemiAutoControl()
{
    // Semi-automatic mode with operator oversight and AHRS assistance
    // This includes features like:
    // - Automatic load leveling during loading operations
    // - Bucket shake for loading assistance
    // - Slope compensation during transport
    // - Anti-spill control when carrying loads

    manual_control_setpoint_s manual_control;
    if (_manual_control_sub.copy(&manual_control)) {
        // Check if operator is actively controlling bucket
        bool operator_bucket_control = fabsf(manual_control.yaw) > 0.05f;
        bool operator_boom_control = fabsf(manual_control.pitch) > 0.05f;

        if (operator_bucket_control || operator_boom_control) {
            // Operator override - use manual control with stability assist
            controlBoomBucket(manual_control);
        } else {
            // No operator input - enable automatic features based on context

            // Determine appropriate automatic mode based on boom position and load
            bucket_status_s bucket_status;
            bool has_bucket_status = _bucket_status_sub.copy(&bucket_status);

            boom_status_s boom_status;
            bool has_boom_status = _boom_status_sub.copy(&boom_status);

            if (has_bucket_status && has_boom_status) {
                // Auto-select control mode based on operational context
                if (boom_status.angle < _work_cycle_params.load_angle * M_PI / 180.0f + 0.2f) {
                    // Low boom position - likely loading
                    _bucket_command.control_mode = bucket_command_s::MODE_AUTO_LEVEL;
                    _bucket_command.target_angle = 0.0f; // Level bucket for loading

                } else if (boom_status.angle > _work_cycle_params.transport_angle * M_PI / 180.0f - 0.2f) {
                    // High boom position - likely transporting
                    _bucket_command.control_mode = bucket_command_s::MODE_TRANSPORT;
                    _bucket_command.target_angle = _work_cycle_params.carry_angle * M_PI / 180.0f;
                    _bucket_command.enable_anti_spill = true;

                } else {
                    // Mid position - use slope compensation
                    _bucket_command.control_mode = bucket_command_s::MODE_SLOPE_COMPENSATION;
                    _bucket_command.target_angle = bucket_status.ground_angle; // Maintain current angle
                }

                _bucket_command.timestamp = hrt_absolute_time();
                _bucket_command.command_mode = 0; // Position mode
                _bucket_command.coordinate_frame = 0; // Ground reference
                _bucket_command.max_velocity = _param_bucket_max_vel.get() * 0.5f; // Slower for auto mode
                _bucket_command.enable_stability_limit = true;

                // Set defaults for unused parameters
                _bucket_command.grading_angle = NAN;
                _bucket_command.transport_angle = NAN;
                _bucket_command.stability_threshold = NAN;

                // Keep boom control manual
                _boom_command.timestamp = hrt_absolute_time();
                _boom_command.command_mode = 1; // Velocity mode
                _boom_command.target_angle = 0.0f; // No automatic boom movement
                _boom_command.max_velocity = _param_boom_max_vel.get();
                _boom_command.max_load = 1.0f;
                _boom_command.priority = 1;
                _boom_command.emergency_stop = false;

                publishBoomBucketCommands();
            }
        }
    }

    _system_state = SystemState::OPERATION;
    _module_status.operational_state = module_status_s::STATE_ACTIVE;
}

void WheelLoaderController::processAutoControl()
{
    if (!_param_auto_enabled.get()) {
        _control_mode = ControlMode::MANUAL;
        return;
    }

    executeWorkCycle();
    _system_state = SystemState::OPERATION;
    _module_status.operational_state = module_status_s::STATE_AUTO;
}

void WheelLoaderController::executeWorkCycle()
{
    switch (_work_cycle_state) {
    case WorkCycleState::IDLE:
        // Wait for work command or transition to approach
        if (_armed) {
            transitionWorkCycleState(WorkCycleState::APPROACH_PILE);
        }
        break;

    case WorkCycleState::APPROACH_PILE:
        // Autonomous navigation to material pile
        // Set velocity for approach
        _actuator_motors.velocity[0] = _work_cycle_params.approach_speed;
        _actuator_motors.velocity[1] = _work_cycle_params.approach_speed;

        // Transition when close to pile (simplified)
        if (hrt_elapsed_time(&_state_transition_time) > 10_s) {
            transitionWorkCycleState(WorkCycleState::LOAD_MATERIAL);
        }
        break;

    case WorkCycleState::LOAD_MATERIAL:
        // Autonomous loading sequence with AHRS-assisted bucket control
        _actuator_motors.velocity[0] = _work_cycle_params.loading_speed;
        _actuator_motors.velocity[1] = _work_cycle_params.loading_speed;

        // Lower boom for loading
        _boom_command.timestamp = hrt_absolute_time();
        _boom_command.command_mode = 0; // Position mode
        _boom_command.target_angle = math::radians(_work_cycle_params.load_angle);
        _boom_command.max_velocity = _param_boom_max_vel.get();
        _boom_command.max_load = 1.0f;
        _boom_command.priority = 2; // High priority for autonomous operation
        _boom_command.emergency_stop = false;

        // Enhanced bucket control for loading with auto-leveling
        _bucket_command.timestamp = hrt_absolute_time();
        _bucket_command.command_mode = 0; // Position mode
        _bucket_command.control_mode = bucket_command_s::MODE_AUTO_LEVEL;
        _bucket_command.target_angle = 0.0f; // Level bucket for optimal loading
        _bucket_command.coordinate_frame = 0; // Ground reference
        _bucket_command.max_velocity = _param_bucket_max_vel.get() * 0.8f; // Slower for precision
        _bucket_command.enable_stability_limit = true;
        _bucket_command.enable_anti_spill = false; // Not needed during loading
        _bucket_command.grading_angle = NAN;
        _bucket_command.transport_angle = NAN;
        _bucket_command.stability_threshold = NAN;

        // Check load level
        if (_boom_bucket_state.boom_load > _work_cycle_params.load_detection_threshold) {
            transitionWorkCycleState(WorkCycleState::TRANSPORT);
        }
        break;

    case WorkCycleState::TRANSPORT:
        // Move to dump location with enhanced transport control
        _actuator_motors.velocity[0] = _work_cycle_params.transport_speed;
        _actuator_motors.velocity[1] = _work_cycle_params.transport_speed;

        // Raise boom to transport position
        _boom_command.timestamp = hrt_absolute_time();
        _boom_command.command_mode = 0; // Position mode
        _boom_command.target_angle = math::radians(_work_cycle_params.transport_angle);
        _boom_command.max_velocity = _param_boom_max_vel.get();
        _boom_command.max_load = 1.0f;
        _boom_command.priority = 2; // High priority for autonomous operation
        _boom_command.emergency_stop = false;

        // Enhanced bucket control for transport with anti-spill and stability
        _bucket_command.timestamp = hrt_absolute_time();
        _bucket_command.command_mode = 0; // Position mode
        _bucket_command.control_mode = bucket_command_s::MODE_TRANSPORT;
        _bucket_command.target_angle = math::radians(_work_cycle_params.carry_angle);
        _bucket_command.coordinate_frame = 0; // Ground reference
        _bucket_command.max_velocity = _param_bucket_max_vel.get() * 0.6f; // Slower for stability
        _bucket_command.enable_stability_limit = true;
        _bucket_command.enable_anti_spill = true; // Critical for transport
        _bucket_command.grading_angle = NAN;
        _bucket_command.transport_angle = math::radians(_work_cycle_params.carry_angle);
        _bucket_command.stability_threshold = NAN;

        // Transition when at dump location (simplified)
        if (hrt_elapsed_time(&_state_transition_time) > 15_s) {
            transitionWorkCycleState(WorkCycleState::DUMP_MATERIAL);
        }
        break;

    case WorkCycleState::DUMP_MATERIAL:
        // Autonomous dump sequence with precision grading control
        _actuator_motors.velocity[0] = 0.0f;
        _actuator_motors.velocity[1] = 0.0f;

        // Raise boom to dump position
        _boom_command.timestamp = hrt_absolute_time();
        _boom_command.command_mode = 0; // Position mode
        _boom_command.target_angle = math::radians(_work_cycle_params.dump_height);
        _boom_command.max_velocity = _param_boom_max_vel.get();
        _boom_command.max_load = 1.0f;
        _boom_command.priority = 2; // High priority for autonomous operation
        _boom_command.emergency_stop = false;

        // Enhanced dump sequence with grading control for precise material placement
        _bucket_command.timestamp = hrt_absolute_time();
        _bucket_command.command_mode = 0; // Position mode
        _bucket_command.control_mode = bucket_command_s::MODE_GRADING;
        _bucket_command.target_angle = math::radians(_work_cycle_params.dump_angle);
        _bucket_command.coordinate_frame = 0; // Ground reference
        _bucket_command.max_velocity = _param_bucket_max_vel.get() * 0.7f; // Controlled dumping speed
        _bucket_command.enable_stability_limit = true;
        _bucket_command.enable_anti_spill = false; // Not needed during dumping
        _bucket_command.grading_angle = math::radians(_work_cycle_params.dump_angle);
        _bucket_command.transport_angle = NAN;
        _bucket_command.stability_threshold = NAN;

        // Transition when dump complete
        if (hrt_elapsed_time(&_state_transition_time) > 8_s) {
            transitionWorkCycleState(WorkCycleState::RETURN);
        }
        break;

    case WorkCycleState::RETURN:
        // Return to starting position with safe transport settings
        _actuator_motors.velocity[0] = -_work_cycle_params.transport_speed * 0.8f;
        _actuator_motors.velocity[1] = -_work_cycle_params.transport_speed * 0.8f;

        // Lower boom to carry position
        _boom_command.timestamp = hrt_absolute_time();
        _boom_command.command_mode = 0; // Position mode
        _boom_command.target_angle = math::radians(_work_cycle_params.carry_angle);
        _boom_command.max_velocity = _param_boom_max_vel.get();
        _boom_command.max_load = 1.0f;
        _boom_command.priority = 2; // High priority for autonomous operation
        _boom_command.emergency_stop = false;

        // Return bucket to safe transport position with slope compensation
        _bucket_command.timestamp = hrt_absolute_time();
        _bucket_command.command_mode = 0; // Position mode
        _bucket_command.control_mode = bucket_command_s::MODE_SLOPE_COMPENSATION;
        _bucket_command.target_angle = 0.0f; // Neutral/level position
        _bucket_command.coordinate_frame = 0; // Ground reference
        _bucket_command.max_velocity = _param_bucket_max_vel.get();
        _bucket_command.enable_stability_limit = true;
        _bucket_command.enable_anti_spill = false; // No load to spill
        _bucket_command.grading_angle = NAN;
        _bucket_command.transport_angle = NAN;
        _bucket_command.stability_threshold = NAN;

        // Transition back to idle when complete
        if (hrt_elapsed_time(&_state_transition_time) > 12_s) {
            transitionWorkCycleState(WorkCycleState::IDLE);
        }
        break;
    }

    publishChassisCommands();
    publishBoomBucketCommands();
}

void WheelLoaderController::transitionWorkCycleState(WorkCycleState new_state)
{
    _work_cycle_state = new_state;
    _state_transition_time = hrt_absolute_time();

    PX4_INFO("Work cycle state transition to: %d", static_cast<int>(new_state));
}

void WheelLoaderController::processEmergencyStop()
{
    handleEmergencyStop();
    _system_state = SystemState::EMERGENCY;
    _module_status.operational_state = module_status_s::STATE_EMERGENCY;
}

void WheelLoaderController::handleEmergencyStop()
{
    // Immediately stop all motion
    resetControllers();

    // Disarm system
    _armed = false;
    _vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
    _module_status.arming_state = module_status_s::ARMING_STATE_DISARMED;

    PX4_WARN("Emergency stop activated");
}

void WheelLoaderController::resetControllers()
{
    // Reset all control outputs to safe values
    _actuator_motors = {};
    _actuator_servos = {};
    _boom_command = {};
    _bucket_command = {};

    // Publish zero commands
    publishChassisCommands();
    publishBoomBucketCommands();
}

void WheelLoaderController::publishChassisCommands()
{
    _actuator_motors.timestamp = hrt_absolute_time();
    _actuator_motors_pub.publish(_actuator_motors);

    _actuator_servos.timestamp = hrt_absolute_time();
    _actuator_servos_pub.publish(_actuator_servos);
}

void WheelLoaderController::publishBoomBucketCommands()
{
    _boom_command_pub.publish(_boom_command);
    _bucket_command_pub.publish(_bucket_command);
}

void WheelLoaderController::performSafetyChecks()
{
    if (hrt_elapsed_time(&_last_safety_check) < SAFETY_CHECK_INTERVAL_US) {
        return;
    }

    _last_safety_check = hrt_absolute_time();

    // Check subsystem health
    bool chassis_healthy = (hrt_elapsed_time(&_chassis_state.last_update) < 1_s);
    bool boom_bucket_healthy = (hrt_elapsed_time(&_boom_bucket_state.last_update) < 1_s);

    if (!chassis_healthy || !boom_bucket_healthy) {
        _module_status.health = module_status_s::HEALTH_ERROR;
        if (_armed) {
            PX4_WARN("Subsystem health check failed");
        }
    } else {
        _module_status.health = module_status_s::HEALTH_OK;
    }

    // Check emergency stop timeout
    if (_emergency_stop_active &&
        hrt_elapsed_time(&_last_manual_control) > EMERGENCY_STOP_TIMEOUT_S * 1e6) {
        // Clear emergency stop after timeout if conditions are safe
        if (allSubsystemsReady()) {
            _emergency_stop_active = false;
            _control_mode = ControlMode::MANUAL;
            PX4_INFO("Emergency stop cleared");
        }
    }
}

bool WheelLoaderController::allSubsystemsReady()
{
    return _boom_bucket_state.boom_ready &&
           _boom_bucket_state.bucket_ready &&
           !_emergency_stop_active &&
           _module_status.health == module_status_s::HEALTH_OK;
}

void WheelLoaderController::publishSystemStatus()
{
    _vehicle_status.timestamp = hrt_absolute_time();
    _vehicle_status_pub.publish(_vehicle_status);

    _module_status.timestamp = hrt_absolute_time();
    _module_status_pub.publish(_module_status);
}

void WheelLoaderController::configureAHRSBucketControl(uint8_t mode, float target_angle)
{
    // Configure the bucket control system for AHRS-enhanced operation
    _bucket_command.control_mode = mode;
    _bucket_command.coordinate_frame = 0; // Ground reference frame for AHRS
    _bucket_command.enable_stability_limit = true;
    _bucket_command.stability_threshold = NAN; // Use parameter default

    if (!isnan(target_angle)) {
        _bucket_command.target_angle = target_angle;
    }

    switch (mode) {
    case bucket_command_s::MODE_AUTO_LEVEL:
        _bucket_command.enable_anti_spill = false;
        _bucket_command.grading_angle = NAN;
        _bucket_command.transport_angle = NAN;
        break;

    case bucket_command_s::MODE_SLOPE_COMPENSATION:
        _bucket_command.enable_anti_spill = false;
        _bucket_command.grading_angle = NAN;
        _bucket_command.transport_angle = NAN;
        break;

    case bucket_command_s::MODE_GRADING:
        _bucket_command.enable_anti_spill = false;
        _bucket_command.grading_angle = isnan(target_angle) ?
            (_work_cycle_params.load_angle * M_PI / 180.0f) : target_angle;
        _bucket_command.transport_angle = NAN;
        break;

    case bucket_command_s::MODE_TRANSPORT:
        _bucket_command.enable_anti_spill = true;
        _bucket_command.grading_angle = NAN;
        _bucket_command.transport_angle = isnan(target_angle) ?
            (_work_cycle_params.transport_angle * M_PI / 180.0f) : target_angle;
        break;

    default:
        _bucket_command.control_mode = bucket_command_s::MODE_MANUAL;
        _bucket_command.enable_anti_spill = false;
        _bucket_command.grading_angle = NAN;
        _bucket_command.transport_angle = NAN;
        break;
    }
}

void WheelLoaderController::updateBucketControlForTerrain()
{
    // Update bucket control based on current terrain conditions
    bucket_status_s bucket_status;
    if (_bucket_status_sub.copy(&bucket_status)) {
        // Check if machine stability is compromised
        if (bucket_status.stability_factor < 0.8f) {
            // Reduce bucket movement aggressiveness
            _bucket_command.max_velocity = _param_bucket_max_vel.get() * bucket_status.stability_factor;

            // Enable stability limiting
            _bucket_command.enable_stability_limit = true;

            // If very unstable, switch to manual mode for operator control
            if (bucket_status.stability_factor < 0.5f) {
                configureAHRSBucketControl(bucket_command_s::MODE_MANUAL);
                PX4_WARN("Switching to manual bucket control due to low stability");
            }
        }

        // Check if we need anti-spill control
        if (bucket_status.spill_risk > 0.5f && _work_cycle_state == WorkCycleState::TRANSPORT) {
            _bucket_command.enable_anti_spill = true;
        }

        // Adjust control parameters based on machine pitch
        if (fabsf(bucket_status.machine_pitch) > 0.17f) { // 10 degrees
            // Enable slope compensation for steep terrain
            if (_bucket_command.control_mode == bucket_command_s::MODE_AUTO_LEVEL) {
                configureAHRSBucketControl(bucket_command_s::MODE_SLOPE_COMPENSATION);
            }
        }
    }
}

bool WheelLoaderController::isStabilityLimitRequired()
{
    bucket_status_s bucket_status;
    if (_bucket_status_sub.copy(&bucket_status)) {
        // Check multiple stability indicators
        bool low_stability = bucket_status.stability_factor < 0.8f;
        bool high_pitch = fabsf(bucket_status.machine_pitch) > 0.26f; // 15 degrees
        bool high_acceleration = bucket_status.spill_risk > 0.3f;

        return low_stability || high_pitch || high_acceleration;
    }

    return true; // Default to enabled for safety
}

void WheelLoaderController::logAHRSStatus()
{
    static uint64_t last_log_time = 0;

    if (hrt_elapsed_time(&last_log_time) > 2_s) {
        last_log_time = hrt_absolute_time();

        bucket_status_s bucket_status;
        if (_bucket_status_sub.copy(&bucket_status)) {
            PX4_INFO("AHRS Status - Mode: %d, Stability: %.2f, Pitch: %.1f°, Spill Risk: %.2f",
                    bucket_status.control_mode,
                    static_cast<double>(bucket_status.stability_factor),
                    static_cast<double>(bucket_status.machine_pitch * 180.0f / M_PI),
                    static_cast<double>(bucket_status.spill_risk));
        }
    }
}

void WheelLoaderController::logSystemPerformance()
{
    // Log performance metrics periodically
    static uint64_t last_log_time = 0;

    if (hrt_elapsed_time(&last_log_time) > 5_s) {
        last_log_time = hrt_absolute_time();

        PX4_INFO("Wheel Loader Performance - Loop: %.3fms, Control: %.3fms",
                 static_cast<double>(perf_mean(_loop_perf)) / 1000.0,
                 static_cast<double>(perf_mean(_control_latency_perf)) / 1000.0);
    }
}

int WheelLoaderController::task_spawn(int argc, char *argv[])
{
    WheelLoaderController *instance = new WheelLoaderController();

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

int WheelLoaderController::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Unified Wheel Loader Controller

This module provides comprehensive control for wheel loader vehicles including:
- Articulated chassis control (front/rear wheels, steering)
- Boom and bucket control with hydraulic actuators
- Manual, semi-automatic, and autonomous operation modes
- Safety monitoring and emergency stop functionality
- Autonomous work cycles for material handling

The controller integrates with:
- Wheel encoders for velocity feedback
- Hydraulic system controllers for boom/bucket
- Safety systems and emergency stops
- Manual control interfaces

### Implementation
The controller runs at 50Hz and provides:
- Real-time chassis and implement control
- Coordinated boom/bucket movements
- Autonomous work cycle execution
- Comprehensive safety monitoring

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("wheel_loader", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int WheelLoaderController::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("not running");
        return 1;
    }

    if (!strcmp(argv[0], "status")) {
        get_instance()->logSystemPerformance();
        return 0;
    }

    return print_usage("unknown command");
}

extern "C" __EXPORT int wheel_loader_main(int argc, char *argv[])
{
    return WheelLoaderController::main(argc, argv);
}
