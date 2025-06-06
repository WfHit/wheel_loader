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

    // Boom control (direct velocity control)
    float boom_max_vel = _param_boom_max_vel.get();
    float boom_velocity = boom_input * boom_max_vel;

    // For boom, we would send commands to the boom controller
    // This is a simplified approach - in reality, boom control might be more complex

    // Bucket control
    _bucket_command.timestamp = hrt_absolute_time();
    _bucket_command.command_mode = 1; // Velocity mode
    _bucket_command.max_velocity = _param_bucket_max_vel.get();
    _bucket_command.target_angle = bucket_input * _param_bucket_max_vel.get(); // Using as velocity
    _bucket_command.coordinate_frame = 0; // Ground reference

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
    // Semi-automatic mode with operator oversight
    // This would include features like:
    // - Automatic load leveling
    // - Bucket shake for loading
    // - Coordinated boom/bucket movements

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
        // Autonomous loading sequence
        _actuator_motors.velocity[0] = _work_cycle_params.loading_speed;
        _actuator_motors.velocity[1] = _work_cycle_params.loading_speed;

        // Check load level
        if (_boom_bucket_state.boom_load > _work_cycle_params.load_detection_threshold) {
            transitionWorkCycleState(WorkCycleState::TRANSPORT);
        }
        break;

    case WorkCycleState::TRANSPORT:
        // Move to dump location
        _actuator_motors.velocity[0] = _work_cycle_params.transport_speed;
        _actuator_motors.velocity[1] = _work_cycle_params.transport_speed;

        // Transition when at dump location (simplified)
        if (hrt_elapsed_time(&_state_transition_time) > 15_s) {
            transitionWorkCycleState(WorkCycleState::DUMP_MATERIAL);
        }
        break;

    case WorkCycleState::DUMP_MATERIAL:
        // Autonomous dump sequence
        _actuator_motors.velocity[0] = 0.0f;
        _actuator_motors.velocity[1] = 0.0f;

        // Execute dump sequence (raise boom, tip bucket)
        _bucket_command.timestamp = hrt_absolute_time();
        _bucket_command.command_mode = 0; // Position mode
        _bucket_command.target_angle = math::radians(_work_cycle_params.dump_angle);
        _bucket_command.coordinate_frame = 0;

        // Transition when dump complete
        if (hrt_elapsed_time(&_state_transition_time) > 8_s) {
            transitionWorkCycleState(WorkCycleState::RETURN);
        }
        break;

    case WorkCycleState::RETURN:
        // Return to starting position
        _actuator_motors.velocity[0] = -_work_cycle_params.transport_speed * 0.8f;
        _actuator_motors.velocity[1] = -_work_cycle_params.transport_speed * 0.8f;

        // Return bucket to transport position
        _bucket_command.timestamp = hrt_absolute_time();
        _bucket_command.command_mode = 0; // Position mode
        _bucket_command.target_angle = 0.0f;
        _bucket_command.coordinate_frame = 0;

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
