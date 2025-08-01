#include "steering_controller.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

#define MODULE_NAME "steering_controller"

SteeringController::SteeringController() :
    ModuleBase(MODULE_NAME),
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

bool SteeringController::init()
{
    // Update parameters from YAML configuration
    updateParams();

    // Schedule at configured rate
    ScheduleOnInterval(CONTROL_INTERVAL_US);

    PX4_INFO("Simple Steering Controller initialized");
    PX4_INFO("Max angle: ±%.1f°, ST3125 Servo ID: %d",
             (double)math::degrees(_max_steering_angle.get()),
             (int)_st3125_servo_id.get());

    return true;
}

void SteeringController::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // Check for parameter updates
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }

    // Process core functions
    process_steering_command();
    process_servo_feedback();
    process_limit_sensors();
    handle_abnormal_events();
}

void SteeringController::process_steering_command()
{
    steering_setpoint_s setpoint;
    if (_steering_setpoint_sub.update(&setpoint)) {
        _last_command_time = hrt_absolute_time();

        // Get target angle and constrain to limits
        _target_angle_rad = saturate_angle(setpoint.steering_angle_rad);

        // Check if command is safe based on limit sensors
        bool safe_to_move = true;
        
        if (_target_angle_rad < 0 && _left_limit_active) {
            safe_to_move = false;  // Don't move left if left limit is active
        }
        
        if (_target_angle_rad > 0 && _right_limit_active) {
            safe_to_move = false;  // Don't move right if right limit is active
        }

        if (safe_to_move && _servo_healthy && !_emergency_stop) {
            send_servo_command(_target_angle_rad);
        } else {
            // Emergency: command to center position
            send_servo_command(0.0f);
        }
    }
}

void SteeringController::send_servo_command(float position_rad)
{
    robotic_servo_command_s cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.id = _st3125_servo_id.get();
    cmd.command_type = 0; // position control
    cmd.goal_position = position_rad;
    cmd.goal_velocity = 0.0f; // Let ST3125 handle velocity
    cmd.goal_current = _st3125_current_limit.get();
    cmd.torque_enable = true;

    _servo_command_pub.publish(cmd);
}

void SteeringController::process_servo_feedback()
{
    robotic_servo_feedback_s feedback;
    if (_servo_feedback_sub.update(&feedback)) {
        if (feedback.id == _st3125_servo_id.get()) {
            _last_feedback_time = hrt_absolute_time();
            
            // Update servo state
            _current_angle_rad = feedback.position;
            _servo_error_flags = feedback.error_flags;
            _servo_current_a = feedback.current;
            _servo_temperature_c = feedback.temperature;
            
            // Check servo health
            _servo_healthy = (feedback.error_flags == 0) && feedback.torque_enabled;
        }
    }
}

void SteeringController::process_limit_sensors()
{
    limit_sensor_s sensor_data;
    while (_limit_sensor_sub.update(&sensor_data)) {
        // Update limit sensor states based on instance
        if (sensor_data.instance == _limit_left_instance.get()) {
            _left_limit_active = sensor_data.state;
        }
        else if (sensor_data.instance == _limit_right_instance.get()) {
            _right_limit_active = sensor_data.state;
        }
        
        // Update sensor health
        _limit_sensors_healthy = !sensor_data.redundancy_fault;
    }
}

void SteeringController::handle_abnormal_events()
{
    // Check for command timeout
    if (is_command_timeout()) {
        PX4_WARN("Steering command timeout - returning to center");
        _target_angle_rad = 0.0f;
        send_servo_command(0.0f);
    }

    // Check for feedback timeout
    if (is_feedback_timeout()) {
        PX4_WARN("Servo feedback timeout");
        _servo_healthy = false;
    }

    // Check servo error flags
    if (_servo_error_flags != 0) {
        PX4_WARN("Servo error flags: 0x%04x", _servo_error_flags);
        _servo_healthy = false;
        _emergency_stop = true;
        send_servo_command(0.0f); // Emergency stop - return to center
    }

    // Check limit sensor faults
    if (!_limit_sensors_healthy) {
        PX4_WARN("Limit sensor fault detected");
        // Continue operation but with caution
    }

    // Check for overcurrent
    if (_servo_current_a > _st3125_current_limit.get()) {
        PX4_WARN("Servo overcurrent: %.2fA", (double)_servo_current_a);
        _emergency_stop = true;
        send_servo_command(0.0f);
    }

    // Check for overtemperature
    if (_servo_temperature_c > 80.0f) { // ST3125 operating limit
        PX4_WARN("Servo overtemperature: %.1f°C", (double)_servo_temperature_c);
        _emergency_stop = true;
        send_servo_command(0.0f);
    }
}

float SteeringController::saturate_angle(float angle_rad)
{
    return math::constrain(angle_rad, -_max_steering_angle.get(), _max_steering_angle.get());
}

bool SteeringController::is_command_timeout()
{
    return (hrt_absolute_time() - _last_command_time) > (_command_timeout_ms.get() * 1000);
}

bool SteeringController::is_feedback_timeout()
{
    return (hrt_absolute_time() - _last_feedback_time) > (_feedback_timeout_ms.get() * 1000);
}

void SteeringController::update_parameters()
{
    // ModuleParams automatically handles the parameter updates from YAML
    updateParams();
}

int SteeringController::print_status()
{
    PX4_INFO("Simple Steering Controller Status:");
    PX4_INFO("  Target angle: %.1f°", (double)math::degrees(_target_angle_rad));
    PX4_INFO("  Current angle: %.1f°", (double)math::degrees(_current_angle_rad));
    PX4_INFO("  Servo healthy: %s", _servo_healthy ? "YES" : "NO");
    PX4_INFO("  Error flags: 0x%04x", _servo_error_flags);
    PX4_INFO("  Current: %.2fA", (double)_servo_current_a);
    PX4_INFO("  Temperature: %.1f°C", (double)_servo_temperature_c);
    PX4_INFO("  Left limit: %s", _left_limit_active ? "ACTIVE" : "inactive");
    PX4_INFO("  Right limit: %s", _right_limit_active ? "ACTIVE" : "inactive");
    PX4_INFO("  Emergency stop: %s", _emergency_stop ? "ACTIVE" : "inactive");

    return 0;
}

int SteeringController::task_spawn(int argc, char *argv[])
{
    SteeringController *instance = new SteeringController();

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

int SteeringController::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int SteeringController::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Simple steering controller for articulated wheel loader.

Essential functions only:
- Subscribe to steering target
- Convert to ST3125 servo command  
- Check limit sensors
- Monitor servo state for abnormal events

### Implementation
The controller runs at 50Hz and sends position commands to the ST3125 servo.
The servo handles its own PID control internally.

### Examples
$ steering_controller start
$ steering_controller status
$ steering_controller stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("steering_controller", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}
