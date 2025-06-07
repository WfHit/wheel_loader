#include "SafetyManager.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/vehicle_command.h>

SafetyManager::SafetyManager() :
    ModuleParams(nullptr)
{
    // Initialize safety state
    _safety_state.current_level = SAFETY_NORMAL;
    _safety_state.current_mode = SAFETY_MONITORING;
    _safety_state.active_faults = FAULT_NONE;

    // Initialize permits to denied for safety
    _safety_permits.motion_permitted = false;
    _safety_permits.steering_permitted = false;
    _safety_permits.autonomous_permitted = false;
    _safety_permits.load_operation_permitted = false;
    _safety_permits.high_speed_permitted = false;
}

int SafetyManager::task_spawn(int argc, char *argv[])
{
    SafetyManager *instance = new SafetyManager();

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

bool SafetyManager::init()
{
    // Initialize subscriptions
    _vehicle_status_sub.set_queue_size(1);
    _module_status_sub.set_queue_size(10);
    _imu_sub.set_queue_size(1);
    _gyro_sub.set_queue_size(1);

    // Initialize limit sensor subscriptions
    for (uint8_t i = 0; i < 8; i++) {
        _limit_sensor_sub[i] = uORB::Subscription{ORB_ID(limit_sensor), i};
        _limit_sensor_sub[i].set_queue_size(1);
    }

    // Initialize monitoring values
    _speed_monitor.speed_limit_ms = _max_speed_ms.get();
    _speed_monitor.max_safe_acceleration = _max_acceleration_ms2.get();
    _steering_monitor.max_safe_angle_rad = _max_steering_angle_rad.get();
    _steering_monitor.max_safe_rate_rads = _max_steering_rate_rads.get();
    _stability_monitor.stability_margin = _stability_margin_factor.get();
    _load_monitor.max_safe_payload_kg = _max_payload_kg.get();
    _load_monitor.max_cg_offset_m = _max_cg_offset_m.get();
    _terrain_monitor.max_safe_slope_rad = _max_slope_rad.get();
    _emergency_response.emergency_deceleration_rate = _emergency_decel_rate.get();

    ScheduleOnInterval(SAFETY_CHECK_INTERVAL_US);

    PX4_INFO("Safety Manager initialized successfully");
    return true;
}

void SafetyManager::run()
{
    if (should_exit()) {
        ScheduleClear();
        return;
    }

    perf_count_t safety_start = hrt_absolute_time();

    // Update parameters
    updateParams();

    // Perform all safety monitoring checks
    monitor_speed_limits();
    monitor_steering_limits();
    monitor_stability();
    monitor_load_conditions();
    monitor_communication();
    monitor_hardware_health();
    monitor_sensor_validity();
    monitor_terrain_conditions();
    monitor_electric_actuators();
    monitor_boom_operations();
    monitor_bucket_operations();
    monitor_articulation_system();
    monitor_power_systems();
    monitor_thermal_conditions();
    monitor_operator_interface();
    monitor_limit_sensors();

    // Update safety mode based on commands and conditions
    update_safety_mode();

    // Assess overall safety situation
    assess_overall_safety();
    calculate_risk_factors();
    update_safety_level();
    check_safety_interlocks();

    // Execute appropriate safety actions
    if (_safety_state.current_level >= SAFETY_WARNING) {
        execute_fail_safe_actions();
    }

    // Handle emergency conditions
    if (_safety_state.current_level == SAFETY_EMERGENCY) {
        handle_emergency_conditions();
    }

    // Attempt recovery if enabled
    if (_enable_auto_recovery.get() && _safety_state.active_faults != FAULT_NONE) {
        attempt_fault_recovery();
    }

    // Update performance metrics
    _safety_performance.safety_check_time_ms = (hrt_absolute_time() - safety_start) / 1000.0f;
    _safety_performance.total_safety_checks++;

    // Publish safety status
    module_status_s safety_status{};
    safety_status.timestamp = hrt_absolute_time();
    safety_status.module_id = 100; // Safety manager ID
    safety_status.health = (_safety_state.current_level <= SAFETY_CAUTION) ?
        module_status_s::HEALTH_OK : module_status_s::HEALTH_ERROR;
    safety_status.arming_state = _safety_permits.motion_permitted ?
        module_status_s::ARMING_STATE_ARMED : module_status_s::ARMING_STATE_DISARMED;
    safety_status.operational_state = static_cast<uint8_t>(_safety_state.current_mode);
    safety_status.error_count = _safety_state.safety_violation_count;

    _safety_status_pub.publish(safety_status);

    // Publish comprehensive system safety status
    system_safety_s system_safety{};
    system_safety.timestamp = hrt_absolute_time();

    // Overall safety status
    system_safety.safety_level = static_cast<uint8_t>(_safety_state.current_level);
    system_safety.safety_mode = static_cast<uint8_t>(_safety_state.current_mode);
    system_safety.overall_risk_factor = _safety_state.overall_risk_factor;
    system_safety.emergency_active = _emergency_response.emergency_stop_commanded;
    system_safety.safety_override_active = _safety_state.safety_override_active;

    // System fault status
    system_safety.active_faults = _safety_state.active_faults;
    system_safety.fault_history = _safety_state.fault_history;
    system_safety.last_fault_time = _safety_state.last_fault_time;
    system_safety.safety_violation_count = _safety_state.safety_violation_count;

    // Safety permits
    system_safety.motion_permitted = _safety_permits.motion_permitted;
    system_safety.steering_permitted = _safety_permits.steering_permitted;
    system_safety.autonomous_permitted = _safety_permits.autonomous_permitted;
    system_safety.electric_actuator_permitted = _safety_permits.electric_actuator_permitted;
    system_safety.boom_operation_permitted = _safety_permits.boom_operation_permitted;
    system_safety.bucket_operation_permitted = _safety_permits.bucket_operation_permitted;
    system_safety.articulation_permitted = _safety_permits.articulation_permitted;
    system_safety.engine_start_permitted = _safety_permits.engine_start_permitted;
    system_safety.emergency_override_active = _safety_permits.emergency_override_permitted;

    // System monitoring status
    system_safety.chassis_safe = (_safety_state.active_faults & (FAULT_SPEED_LIMIT | FAULT_STEERING_LIMIT | FAULT_STABILITY)) == 0;
    system_safety.hydraulic_safe = (_safety_state.active_faults & FAULT_HYDRAULIC) == 0; // Now represents electric actuator safety
    system_safety.sensor_safe = (_safety_state.active_faults & (FAULT_SENSOR | FAULT_LIMIT_SENSOR)) == 0;
    system_safety.communication_safe = (_safety_state.active_faults & FAULT_COMMUNICATION) == 0;
    system_safety.power_safe = (_safety_state.active_faults & FAULT_POWER) == 0;
    system_safety.thermal_safe = (_safety_state.active_faults & FAULT_THERMAL) == 0;

    // Emergency response
    system_safety.emergency_stop_commanded = _emergency_response.emergency_stop_commanded;
    system_safety.controlled_stop_active = _emergency_response.controlled_stop_active;
    system_safety.emergency_shutdown_active = _emergency_response.emergency_shutdown_active;
    system_safety.backup_systems_active = _emergency_response.backup_systems_active;

    // Performance metrics
    system_safety.safety_check_frequency_hz = SAFETY_CHECK_RATE_HZ;
    system_safety.intervention_response_time_ms = _safety_performance.safety_check_time_ms;
    system_safety.safety_system_availability = (_safety_state.current_level != SAFETY_EMERGENCY) ? 1.0f : 0.0f;
    system_safety.total_safety_checks = _safety_performance.total_safety_checks;
    system_safety.safety_interventions = _safety_performance.safety_interventions;

    _system_safety_pub.publish(system_safety);
}

void SafetyManager::monitor_speed_limits()
{
    wheel_speeds_setpoint_s wheel_speeds;

    if (_wheel_speeds_sub.update(&wheel_speeds)) {
        // Calculate current speed from wheel speeds
        float avg_rpm = (wheel_speeds.front_left_rpm + wheel_speeds.front_right_rpm +
                        wheel_speeds.rear_left_rpm + wheel_speeds.rear_right_rpm) / 4.0f;
        _speed_monitor.current_speed_ms = avg_rpm * (2.0f * M_PI * 0.5f) / 60.0f; // Assuming 0.5m wheel radius

        // Calculate acceleration
        static float last_speed = 0.0f;
        static uint64_t last_time = 0;
        uint64_t current_time = hrt_absolute_time();

        if (last_time > 0) {
            float dt = (current_time - last_time) / 1e6f;
            if (dt > 0.001f) { // Avoid division by zero
                _speed_monitor.acceleration_ms2 = (_speed_monitor.current_speed_ms - last_speed) / dt;
            }
        }

        last_speed = _speed_monitor.current_speed_ms;
        last_time = current_time;

        // Check speed limits
        if (_speed_monitor.current_speed_ms > _speed_monitor.speed_limit_ms) {
            _speed_monitor.speed_limit_exceeded = true;
            _speed_monitor.speed_violations++;
            _safety_state.active_faults |= FAULT_SPEED_LIMIT;
        } else {
            _speed_monitor.speed_limit_exceeded = false;
            _safety_state.active_faults &= ~FAULT_SPEED_LIMIT;
        }

        // Check for hard braking
        if (_speed_monitor.acceleration_ms2 < -_emergency_response.emergency_deceleration_rate) {
            _speed_monitor.hard_braking_detected = true;
        } else {
            _speed_monitor.hard_braking_detected = false;
        }

        // Update safe speed based on conditions
        float safety_factor = 1.0f;
        if (_stability_monitor.stability_warning) safety_factor *= 0.8f;
        if (_terrain_monitor.poor_traction) safety_factor *= 0.7f;
        if (_load_monitor.overload_detected) safety_factor *= 0.6f;

        _speed_monitor.max_safe_speed_ms = _speed_monitor.speed_limit_ms * safety_factor;
    }
}

void SafetyManager::monitor_steering_limits()
{
    steering_setpoint_s steering_setpoint;
    steering_status_s steering_status;

    if (_steering_setpoint_sub.update(&steering_setpoint)) {
        _steering_monitor.commanded_angle_rad = steering_setpoint.angle_rad;
    }

    if (_steering_status_sub.update(&steering_status)) {
        _steering_monitor.current_angle_rad = steering_status.current_angle_rad;

        // Calculate steering rate
        static float last_angle = 0.0f;
        static uint64_t last_time = 0;
        uint64_t current_time = hrt_absolute_time();

        if (last_time > 0) {
            float dt = (current_time - last_time) / 1e6f;
            if (dt > 0.001f) {
                _steering_monitor.steering_rate_rads = (_steering_monitor.current_angle_rad - last_angle) / dt;
            }
        }

        last_angle = _steering_monitor.current_angle_rad;
        last_time = current_time;

        // Check angle limits
        if (fabsf(_steering_monitor.current_angle_rad) > _steering_monitor.max_safe_angle_rad) {
            _steering_monitor.angle_limit_exceeded = true;
            _steering_monitor.steering_violations++;
            _safety_state.active_faults |= FAULT_STEERING_LIMIT;
        } else {
            _steering_monitor.angle_limit_exceeded = false;
            _safety_state.active_faults &= ~FAULT_STEERING_LIMIT;
        }

        // Check rate limits
        if (fabsf(_steering_monitor.steering_rate_rads) > _steering_monitor.max_safe_rate_rads) {
            _steering_monitor.rate_limit_exceeded = true;
        } else {
            _steering_monitor.rate_limit_exceeded = false;
        }

        // Check for steering error
        float steering_error = fabsf(_steering_monitor.current_angle_rad - _steering_monitor.commanded_angle_rad);
        _steering_monitor.steering_error_detected = (steering_error > 0.1f); // 5.7 degrees
    }
}

void SafetyManager::monitor_stability()
{
    sensor_accel_s accel_data;
    sensor_gyro_s gyro_data;

    if (_imu_sub.update(&accel_data)) {
        // Calculate roll and pitch from accelerometer
        _stability_monitor.roll_angle_rad = atan2f(accel_data.y, accel_data.z);
        _stability_monitor.pitch_angle_rad = atan2f(-accel_data.x,
            sqrtf(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        _stability_monitor.lateral_acceleration_ms2 = accel_data.y;
        _stability_monitor.longitudinal_acceleration_ms2 = accel_data.x;

        _sensor_monitor.imu_valid = true;
        _sensor_monitor.last_imu_time = hrt_absolute_time();
    }

    if (_gyro_sub.update(&gyro_data)) {
        _stability_monitor.roll_rate_rads = gyro_data.x;
        _stability_monitor.pitch_rate_rads = gyro_data.y;

        _sensor_monitor.gyro_valid = true;
        _sensor_monitor.last_gyro_time = hrt_absolute_time();
    }

    // Check stability limits
    if (fabsf(_stability_monitor.roll_angle_rad) > _max_roll_angle_rad.get() ||
        fabsf(_stability_monitor.pitch_angle_rad) > _max_pitch_angle_rad.get()) {
        _stability_monitor.stability_warning = true;
        _safety_state.active_faults |= FAULT_STABILITY;
    } else {
        _stability_monitor.stability_warning = false;
        _safety_state.active_faults &= ~FAULT_STABILITY;
    }

    // Calculate stability margin
    float roll_margin = 1.0f - fabsf(_stability_monitor.roll_angle_rad) / _max_roll_angle_rad.get();
    float pitch_margin = 1.0f - fabsf(_stability_monitor.pitch_angle_rad) / _max_pitch_angle_rad.get();
    _stability_monitor.stability_margin = math::min(roll_margin, pitch_margin);

    // Check for rollover risk
    if (_stability_monitor.stability_margin < 0.2f) {
        _stability_monitor.rollover_risk = true;
        _stability_monitor.stability_violations++;
    } else {
        _stability_monitor.rollover_risk = false;
    }

    // Detect tip-over conditions
    if (fabsf(_stability_monitor.roll_angle_rad) > _max_roll_angle_rad.get() * 0.9f) {
        _stability_monitor.tip_over_detected = true;
    } else {
        _stability_monitor.tip_over_detected = false;
    }
}

void SafetyManager::monitor_load_conditions()
{
    load_aware_torque_s load_status;

    if (_load_aware_torque_sub.update(&load_status)) {
        _load_monitor.payload_mass_kg = load_status.payload_mass_kg;
        _load_monitor.center_of_gravity_offset_m = load_status.cg_offset_m;

        // Check load limits
        if (_load_monitor.payload_mass_kg > _load_monitor.max_safe_payload_kg) {
            _load_monitor.overload_detected = true;
            _load_monitor.load_violations++;
            _safety_state.active_faults |= FAULT_LOAD;
        } else {
            _load_monitor.overload_detected = false;
            _safety_state.active_faults &= ~FAULT_LOAD;
        }

        // Check center of gravity offset
        if (fabsf(_load_monitor.center_of_gravity_offset_m) > _load_monitor.max_cg_offset_m) {
            _load_monitor.unsafe_load_distribution = true;
        } else {
            _load_monitor.unsafe_load_distribution = false;
        }

        // Detect load shift (rapid CG change)
        static float last_cg_offset = 0.0f;
        float cg_change = fabsf(_load_monitor.center_of_gravity_offset_m - last_cg_offset);
        _load_monitor.load_shift_detected = (cg_change > 0.5f); // 50cm shift threshold
        last_cg_offset = _load_monitor.center_of_gravity_offset_m;
    }
}

void SafetyManager::monitor_communication()
{
    manual_control_input_s manual_control;
    vehicle_command_s vehicle_command;
    module_status_s module_status;

    uint64_t current_time = hrt_absolute_time();
    float timeout_us = _communication_timeout_s.get() * 1e6f;

    // Check manual control timeout
    if (_manual_control_sub.update(&manual_control)) {
        _comm_monitor.last_manual_command_time = current_time;
        _comm_monitor.manual_control_timeout = false;
    } else if (current_time - _comm_monitor.last_manual_command_time > timeout_us) {
        _comm_monitor.manual_control_timeout = true;
        _comm_monitor.communication_failures++;
        _safety_state.active_faults |= FAULT_COMMUNICATION;
    }

    // Check vehicle command timeout
    if (_vehicle_command_sub.update(&vehicle_command)) {
        _comm_monitor.last_vehicle_command_time = current_time;
        _comm_monitor.vehicle_command_timeout = false;
    } else if (current_time - _comm_monitor.last_vehicle_command_time > timeout_us) {
        _comm_monitor.vehicle_command_timeout = true;
    }

    // Check module status timeout
    if (_module_status_sub.update(&module_status)) {
        _comm_monitor.last_module_status_time = current_time;
        _comm_monitor.module_status_timeout = false;
    } else if (current_time - _comm_monitor.last_module_status_time > timeout_us) {
        _comm_monitor.module_status_timeout = true;
    }

    // Calculate communication quality
    uint32_t timeout_count = 0;
    if (_comm_monitor.manual_control_timeout) timeout_count++;
    if (_comm_monitor.vehicle_command_timeout) timeout_count++;
    if (_comm_monitor.module_status_timeout) timeout_count++;

    _comm_monitor.communication_quality = 1.0f - (timeout_count / 3.0f);

    // Clear communication fault if no timeouts
    if (timeout_count == 0) {
        _safety_state.active_faults &= ~FAULT_COMMUNICATION;
    }
}

void SafetyManager::monitor_hardware_health()
{
    module_status_s module_status;

    // Reset hardware faults
    _hardware_monitor.front_wheel_fault = false;
    _hardware_monitor.rear_wheel_fault = false;
    _hardware_monitor.steering_fault = false;
    _hardware_monitor.sensor_fault = false;

    uint32_t healthy_modules = 0;
    uint32_t total_modules = 0;

    while (_module_status_sub.update(&module_status)) {
        total_modules++;

        switch (module_status.module_id) {
            case 1: // Front wheel module
                _hardware_monitor.front_wheel_fault = (module_status.health != module_status_s::HEALTH_OK);
                if (!_hardware_monitor.front_wheel_fault) healthy_modules++;
                break;

            case 2: // Rear wheel module
                _hardware_monitor.rear_wheel_fault = (module_status.health != module_status_s::HEALTH_OK);
                if (!_hardware_monitor.rear_wheel_fault) healthy_modules++;
                break;

            case 3: // Steering module
                _hardware_monitor.steering_fault = (module_status.health != module_status_s::HEALTH_OK);
                if (!_hardware_monitor.steering_fault) healthy_modules++;
                break;

            default:
                if (module_status.health == module_status_s::HEALTH_OK) healthy_modules++;
                break;
        }

        if (module_status.health == module_status_s::HEALTH_ERROR ||
            module_status.health == module_status_s::HEALTH_CRITICAL) {
            _hardware_monitor.hardware_failures++;

            if (module_status.health == module_status_s::HEALTH_CRITICAL) {
                _hardware_monitor.critical_failures++;
            }
        }
    }

    // Calculate hardware health score
    if (total_modules > 0) {
        _hardware_monitor.hardware_health_score = (float)healthy_modules / total_modules;
    }

    // Check for critical hardware faults
    if (_hardware_monitor.front_wheel_fault || _hardware_monitor.rear_wheel_fault ||
        _hardware_monitor.steering_fault || _hardware_monitor.critical_failures > 0) {
        _safety_state.active_faults |= FAULT_HARDWARE;
    } else {
        _safety_state.active_faults &= ~FAULT_HARDWARE;
    }
}

void SafetyManager::monitor_sensor_validity()
{
    uint64_t current_time = hrt_absolute_time();
    float sensor_timeout_us = _sensor_timeout_s.get() * 1e6f;

    // Check IMU validity
    if (current_time - _sensor_monitor.last_imu_time > sensor_timeout_us) {
        _sensor_monitor.imu_valid = false;
        _sensor_monitor.sensor_failures++;
    }

    // Check gyro validity
    if (current_time - _sensor_monitor.last_gyro_time > sensor_timeout_us) {
        _sensor_monitor.gyro_valid = false;
        _sensor_monitor.sensor_failures++;
    }

    // Calculate sensor confidence
    uint32_t valid_sensors = 0;
    uint32_t total_sensors = 5; // IMU, gyro, encoders, load, steering

    if (_sensor_monitor.imu_valid) valid_sensors++;
    if (_sensor_monitor.gyro_valid) valid_sensors++;
    if (_sensor_monitor.encoder_valid) valid_sensors++;
    if (_sensor_monitor.load_sensor_valid) valid_sensors++;
    if (_sensor_monitor.steering_sensor_valid) valid_sensors++;

    _sensor_monitor.sensor_confidence = (float)valid_sensors / total_sensors;

    // Set sensor fault if confidence is too low
    if (_sensor_monitor.sensor_confidence < 0.6f) {
        _safety_state.active_faults |= FAULT_SENSOR;
    } else {
        _safety_state.active_faults &= ~FAULT_SENSOR;
    }
}

void SafetyManager::monitor_terrain_conditions()
{
    terrain_adaptation_s terrain_status;

    if (_terrain_adaptation_sub.update(&terrain_status)) {
        _terrain_monitor.estimated_slope_rad = terrain_status.terrain_slope_rad;
        _terrain_monitor.surface_roughness = terrain_status.surface_roughness;
        _terrain_monitor.traction_coefficient = terrain_status.traction_coefficient;

        // Check slope limits
        if (fabsf(_terrain_monitor.estimated_slope_rad) > _terrain_monitor.max_safe_slope_rad) {
            _terrain_monitor.slope_limit_exceeded = true;
            _terrain_monitor.terrain_violations++;
            _safety_state.active_faults |= FAULT_TERRAIN;
        } else {
            _terrain_monitor.slope_limit_exceeded = false;
            _safety_state.active_faults &= ~FAULT_TERRAIN;
        }

        // Check traction conditions
        if (_terrain_monitor.traction_coefficient < 0.3f) {
            _terrain_monitor.poor_traction = true;
        } else {
            _terrain_monitor.poor_traction = false;
        }

        // Determine unsafe terrain
        _terrain_monitor.unsafe_terrain = _terrain_monitor.slope_limit_exceeded ||
                                         _terrain_monitor.poor_traction ||
                                         _terrain_monitor.surface_roughness > 0.8f;
    }
}

void SafetyManager::monitor_electric_actuators()
{
    boom_status_s boom_status;
    bucket_status_s bucket_status;
    wheel_loader_status_s loader_status;

    // Monitor boom electric actuator system
    if (_boom_status_sub.update(&boom_status)) {
        _electric_actuator_monitor.system_voltage_v = boom_status.motor_voltage;
        _electric_actuator_monitor.system_current_a = boom_status.motor_current;
        _electric_actuator_monitor.motor_temperature_c = boom_status.motor_temperature_c;

        // Check voltage limits
        if (_electric_actuator_monitor.system_voltage_v > _electric_actuator_monitor.max_safe_voltage_v ||
            _electric_actuator_monitor.system_voltage_v < _electric_actuator_monitor.min_safe_voltage_v) {
            _electric_actuator_monitor.voltage_fault = true;
            _electric_actuator_monitor.actuator_violations++;
            _safety_state.active_faults |= FAULT_HYDRAULIC; // Reusing FAULT_HYDRAULIC as FAULT_ELECTRIC_ACTUATOR
            PX4_WARN("Electric actuator voltage fault: %.1f V", (double)_electric_actuator_monitor.system_voltage_v);
        } else {
            _electric_actuator_monitor.voltage_fault = false;
        }

        // Check current limits
        if (_electric_actuator_monitor.system_current_a > _electric_actuator_monitor.max_safe_current_a) {
            _electric_actuator_monitor.current_fault = true;
            _electric_actuator_monitor.actuator_violations++;
            _safety_state.active_faults |= FAULT_HYDRAULIC;
            PX4_WARN("Electric actuator current fault: %.1f A", (double)_electric_actuator_monitor.system_current_a);
        } else {
            _electric_actuator_monitor.current_fault = false;
        }

        // Check temperature limits
        if (_electric_actuator_monitor.motor_temperature_c > _electric_actuator_monitor.max_safe_temperature_c) {
            _electric_actuator_monitor.temperature_fault = true;
            _electric_actuator_monitor.actuator_violations++;
            _safety_state.active_faults |= FAULT_THERMAL;
            PX4_WARN("Electric motor temperature fault: %.1f°C", (double)_electric_actuator_monitor.motor_temperature_c);
        } else {
            _electric_actuator_monitor.temperature_fault = false;
        }

        // Update electric actuator health score
        float voltage_factor = 1.0f - math::constrain(fabsf(_electric_actuator_monitor.system_voltage_v - 24.0f) / 12.0f, 0.0f, 1.0f); // Nominal 24V
        float current_factor = 1.0f - math::constrain(_electric_actuator_monitor.system_current_a / _electric_actuator_monitor.max_safe_current_a, 0.0f, 1.0f);
        float temp_factor = 1.0f - math::constrain(_electric_actuator_monitor.motor_temperature_c / _electric_actuator_monitor.max_safe_temperature_c, 0.0f, 1.0f);
        _electric_actuator_monitor.actuator_health_score = (voltage_factor + current_factor + temp_factor) / 3.0f;
    }

    // Check for motor controller and power system status
    if (_wheel_loader_status_sub.update(&loader_status)) {
        // Monitor motor controller and power system health
        _electric_actuator_monitor.controller_fault = (loader_status.system_health > 1); // WARNING or higher
        _electric_actuator_monitor.power_supply_fault = (loader_status.supply_voltage < 20.0f); // Low voltage indicator
    }
}

void SafetyManager::monitor_boom_operations()
{
    boom_status_s boom_status;

    if (_boom_status_sub.update(&boom_status)) {
        _boom_monitor.boom_angle_rad = boom_status.angle;
        _boom_monitor.boom_current_a = boom_status.motor_current;
        _boom_monitor.boom_voltage_v = boom_status.motor_voltage;
        _boom_monitor.boom_temperature_c = boom_status.motor_temperature_c;

        // Check boom angle limits
        if (fabsf(_boom_monitor.boom_angle_rad) > _boom_monitor.max_safe_boom_angle_rad) {
            _boom_monitor.boom_limit_exceeded = true;
            _boom_monitor.boom_violations++;
            _safety_state.active_faults |= FAULT_BOOM;
            PX4_WARN("Boom angle limit exceeded: %.2f rad", (double)_boom_monitor.boom_angle_rad);
        } else {
            _boom_monitor.boom_limit_exceeded = false;
            _safety_state.active_faults &= ~FAULT_BOOM;
        }

        // Check boom electric motor current limits
        if (_boom_monitor.boom_current_a > _boom_monitor.max_safe_current_a) {
            _boom_monitor.boom_motor_fault = true;
            _boom_monitor.boom_violations++;
            _safety_state.active_faults |= FAULT_BOOM;
            PX4_WARN("Boom motor current fault: %.1f A", (double)_boom_monitor.boom_current_a);
        } else {
            _boom_monitor.boom_motor_fault = false;
        }

        // Check boom motor temperature limits
        if (_boom_monitor.boom_temperature_c > _boom_monitor.max_safe_temperature_c) {
            _boom_monitor.boom_motor_fault = true;
            _boom_monitor.boom_violations++;
            _safety_state.active_faults |= FAULT_BOOM;
            PX4_WARN("Boom motor temperature fault: %.1f°C", (double)_boom_monitor.boom_temperature_c);
        }

        // Check boom motor and encoder health
        _boom_monitor.boom_motor_fault = boom_status.motor_fault;
        _boom_monitor.boom_encoder_fault = boom_status.encoder_fault;

        if (_boom_monitor.boom_motor_fault || _boom_monitor.boom_encoder_fault) {
            _safety_state.active_faults |= FAULT_BOOM;
        }
    }
}
}

void SafetyManager::monitor_bucket_operations()
{
    bucket_status_s bucket_status;

    if (_bucket_status_sub.update(&bucket_status)) {
        _bucket_monitor.bucket_angle_rad = bucket_status.current_angle;
        _bucket_monitor.bucket_current_a = bucket_status.motor_current;
        _bucket_monitor.bucket_voltage_v = bucket_status.motor_voltage;
        _bucket_monitor.bucket_temperature_c = bucket_status.motor_temperature_c;

        // Check bucket angle limits
        if (fabsf(_bucket_monitor.bucket_angle_rad) > _bucket_monitor.max_safe_bucket_angle_rad) {
            _bucket_monitor.bucket_limit_exceeded = true;
            _bucket_monitor.bucket_violations++;
            _safety_state.active_faults |= FAULT_BUCKET;
            PX4_WARN("Bucket angle limit exceeded: %.2f rad", (double)_bucket_monitor.bucket_angle_rad);
        } else {
            _bucket_monitor.bucket_limit_exceeded = false;
            _safety_state.active_faults &= ~FAULT_BUCKET;
        }

        // Check bucket electric motor current limits
        if (_bucket_monitor.bucket_current_a > _bucket_monitor.max_safe_current_a) {
            _bucket_monitor.bucket_motor_fault = true;
            _bucket_monitor.bucket_violations++;
            _safety_state.active_faults |= FAULT_BUCKET;
            PX4_WARN("Bucket motor current fault: %.1f A", (double)_bucket_monitor.bucket_current_a);
        } else {
            _bucket_monitor.bucket_motor_fault = false;
        }

        // Check bucket motor temperature limits
        if (_bucket_monitor.bucket_temperature_c > _bucket_monitor.max_safe_temperature_c) {
            _bucket_monitor.bucket_motor_fault = true;
            _bucket_monitor.bucket_violations++;
            _safety_state.active_faults |= FAULT_BUCKET;
            PX4_WARN("Bucket motor temperature fault: %.1f°C", (double)_bucket_monitor.bucket_temperature_c);
        }

        // Check bucket motor and encoder health
        _bucket_monitor.bucket_motor_fault = bucket_status.motor_fault;
        _bucket_monitor.bucket_encoder_fault = bucket_status.encoder_fault;

        if (_bucket_monitor.bucket_motor_fault || _bucket_monitor.bucket_encoder_fault) {
            _safety_state.active_faults |= FAULT_BUCKET;
        }
    }
}
}

void SafetyManager::monitor_articulation_system()
{
    wheel_loader_status_s loader_status;

    if (_wheel_loader_status_sub.update(&loader_status)) {
        _articulation_monitor.articulation_angle_rad = loader_status.articulation_angle_feedback;

        // Calculate articulation rate (simple derivative)
        static float prev_angle = 0.0f;
        static uint64_t prev_time = 0;
        uint64_t current_time = hrt_absolute_time();

        if (prev_time > 0) {
            float dt = (current_time - prev_time) / 1e6f;
            if (dt > 0.001f) { // Avoid division by very small numbers
                _articulation_monitor.articulation_rate_rads =
                    (_articulation_monitor.articulation_angle_rad - prev_angle) / dt;
            }
        }
        prev_angle = _articulation_monitor.articulation_angle_rad;
        prev_time = current_time;

        // Check articulation angle limits
        if (fabsf(_articulation_monitor.articulation_angle_rad) > _articulation_monitor.max_safe_articulation_angle_rad) {
            _articulation_monitor.angle_limit_exceeded = true;
            _articulation_monitor.articulation_violations++;
            _safety_state.active_faults |= FAULT_ARTICULATION;
            PX4_WARN("Articulation angle limit exceeded: %.2f rad", (double)_articulation_monitor.articulation_angle_rad);
        } else {
            _articulation_monitor.angle_limit_exceeded = false;
        }

        // Check articulation rate limits
        if (fabsf(_articulation_monitor.articulation_rate_rads) > _articulation_monitor.max_safe_articulation_rate_rads) {
            _articulation_monitor.rate_limit_exceeded = true;
            _articulation_monitor.articulation_violations++;
            _safety_state.active_faults |= FAULT_ARTICULATION;
            PX4_WARN("Articulation rate limit exceeded: %.2f rad/s", (double)_articulation_monitor.articulation_rate_rads);
        } else {
            _articulation_monitor.rate_limit_exceeded = false;
        }

        // Update overall articulation fault status
        if (!_articulation_monitor.angle_limit_exceeded && !_articulation_monitor.rate_limit_exceeded) {
            _safety_state.active_faults &= ~FAULT_ARTICULATION;
        }

        // Check for articulation system faults
        _articulation_monitor.articulation_fault = (loader_status.system_health > 2); // ERROR or CRITICAL
    }
}

void SafetyManager::monitor_power_systems()
{
    wheel_loader_status_s loader_status;

    if (_wheel_loader_status_sub.update(&loader_status)) {
        _power_monitor.battery_voltage_v = loader_status.supply_voltage;
        _power_monitor.system_current_a = loader_status.front_motor_current + loader_status.rear_motor_current;
        _power_monitor.power_consumption_w = loader_status.power_consumption;

        // Check voltage limits
        if (_power_monitor.battery_voltage_v < _power_monitor.min_safe_voltage_v) {
            _power_monitor.voltage_fault = true;
            _power_monitor.power_violations++;
            _safety_state.active_faults |= FAULT_POWER;
            PX4_WARN("Low battery voltage: %.1f V", (double)_power_monitor.battery_voltage_v);
        } else {
            _power_monitor.voltage_fault = false;
        }

        // Check current limits
        if (_power_monitor.system_current_a > _power_monitor.max_safe_current_a) {
            _power_monitor.current_fault = true;
            _power_monitor.power_violations++;
            _safety_state.active_faults |= FAULT_POWER;
            PX4_WARN("High system current: %.1f A", (double)_power_monitor.system_current_a);
        } else {
            _power_monitor.current_fault = false;
        }

        // Update overall power fault status
        _power_monitor.power_fault = _power_monitor.voltage_fault || _power_monitor.current_fault;

        if (!_power_monitor.power_fault) {
            _safety_state.active_faults &= ~FAULT_POWER;
        }
    }
}

void SafetyManager::monitor_thermal_conditions()
{
    wheel_loader_status_s loader_status;
    boom_status_s boom_status;
    bucket_status_s bucket_status;

    if (_wheel_loader_status_sub.update(&loader_status)) {
        _thermal_monitor.engine_temperature_c = loader_status.motor_temperature;
        _thermal_monitor.motor_temperature_c = loader_status.controller_temperature;
        // Note: Using controller temperature as proxy for motor controller temp
        // In real implementation, would have dedicated motor controller temp sensor

        // Check engine temperature
        if (_thermal_monitor.engine_temperature_c > _thermal_monitor.max_safe_engine_temp_c) {
            _thermal_monitor.engine_overheat = true;
            _thermal_monitor.thermal_violations++;
            _safety_state.active_faults |= FAULT_THERMAL;
            PX4_WARN("Engine overheat: %.1f°C", (double)_thermal_monitor.engine_temperature_c);
        } else {
            _thermal_monitor.engine_overheat = false;
        }

        // Check motor controller temperature
        if (_thermal_monitor.motor_temperature_c > _thermal_monitor.max_safe_motor_temp_c) {
            _thermal_monitor.motor_overheat = true;
            _thermal_monitor.thermal_violations++;
            _safety_state.active_faults |= FAULT_THERMAL;
            PX4_WARN("Motor controller overheat: %.1f°C", (double)_thermal_monitor.motor_temperature_c);
        } else {
            _thermal_monitor.motor_overheat = false;
        }
    }

    // Monitor boom motor temperature
    if (_boom_status_sub.update(&boom_status)) {
        if (boom_status.motor_temperature_c > _thermal_monitor.max_safe_motor_temp_c) {
            _thermal_monitor.motor_overheat = true;
            _thermal_monitor.thermal_violations++;
            _safety_state.active_faults |= FAULT_THERMAL;
            PX4_WARN("Boom motor overheat: %.1f°C", (double)boom_status.motor_temperature_c);
        }
    }

    // Monitor bucket motor temperature
    if (_bucket_status_sub.update(&bucket_status)) {
        if (bucket_status.motor_temperature_c > _thermal_monitor.max_safe_motor_temp_c) {
            _thermal_monitor.motor_overheat = true;
            _thermal_monitor.thermal_violations++;
            _safety_state.active_faults |= FAULT_THERMAL;
            PX4_WARN("Bucket motor overheat: %.1f°C", (double)bucket_status.motor_temperature_c);
        }
    }

    // Update cooling fault status
    _thermal_monitor.cooling_fault = _thermal_monitor.engine_overheat || _thermal_monitor.motor_overheat;

    if (!_thermal_monitor.cooling_fault) {
        _safety_state.active_faults &= ~FAULT_THERMAL;
    }
}
}

void SafetyManager::monitor_operator_interface()
{
    // Monitor manual control input for operator presence and validity
    manual_control_input_s manual_input;

    if (_manual_control_sub.update(&manual_input)) {
        static uint64_t last_manual_time = 0;
        uint64_t current_time = hrt_absolute_time();

        // Check for operator input timeout
        if (current_time - last_manual_time > 5_s) {
            // No operator input for 5 seconds
            _safety_permits.manual_override_active = false;
            PX4_DEBUG("Operator input timeout detected");
        } else {
            _safety_permits.manual_override_active = true;
        }

        last_manual_time = current_time;

        // Validate input signals
        bool inputs_valid = (manual_input.timestamp > 0) &&
                           (fabsf(manual_input.roll) <= 1.0f) &&
                           (fabsf(manual_input.pitch) <= 1.0f) &&
                           (fabsf(manual_input.yaw) <= 1.0f) &&
                           (fabsf(manual_input.throttle) <= 1.0f);

        if (!inputs_valid) {
            _safety_state.active_faults |= FAULT_COMMUNICATION;
            PX4_WARN("Invalid operator interface signals");
        } else {
            _safety_state.active_faults &= ~FAULT_COMMUNICATION;
        }
    }
}

void SafetyManager::monitor_limit_sensors()
{
    uint64_t current_time = hrt_absolute_time();
    uint32_t sensor_faults = 0;
    uint32_t redundancy_faults = 0;
    uint32_t timeout_faults = 0;
    bool all_healthy = true;

    // Check each limit sensor instance
    for (uint8_t i = 0; i < 8; i++) {
        limit_sensor_s sensor_msg;
        auto& sensor = _limit_sensor_monitor.sensors[i];

        if (_limit_sensor_sub[i].copy(&sensor_msg)) {
            sensor.last_update_time = current_time;
            sensor.healthy = sensor_msg.healthy;
            sensor.state = sensor_msg.state;
            sensor.redundancy_fault = sensor_msg.redundancy_fault;
            sensor.timeout = false;
            sensor.activation_count = sensor_msg.activation_count;
            sensor.sensor_type = sensor_msg.type;

            // Check for redundancy faults
            if (sensor_msg.redundancy_fault) {
                redundancy_faults++;
                PX4_WARN_THROTTLE(5000, "Limit sensor %d redundancy fault", i);
            }

            // Check sensor health
            if (!sensor_msg.healthy) {
                sensor_faults++;
                all_healthy = false;
                PX4_WARN_THROTTLE(5000, "Limit sensor %d unhealthy", i);
            }
        } else {
            // Check for timeout (no recent messages)
            if (current_time - sensor.last_update_time > 1_s && sensor.last_update_time > 0) {
                sensor.timeout = true;
                timeout_faults++;
                all_healthy = false;
                PX4_WARN_THROTTLE(10000, "Limit sensor %d timeout", i);
            }
        }
    }

    // Update monitoring state
    _limit_sensor_monitor.total_sensor_faults = sensor_faults;
    _limit_sensor_monitor.redundancy_faults = redundancy_faults;
    _limit_sensor_monitor.timeout_faults = timeout_faults;
    _limit_sensor_monitor.system_healthy = all_healthy;

    // Calculate health score
    float health_penalty = (float)(sensor_faults + redundancy_faults + timeout_faults) / 8.0f;
    _limit_sensor_monitor.sensor_health_score = math::max(0.0f, 1.0f - health_penalty);

    // Check for zeroing mode (safety override conditions)
    system_safety_s safety_msg;
    if (_system_safety_pub.get_state() != nullptr) {
        // Access published safety state to check mode
        _limit_sensor_monitor.zeroing_mode_active =
            (_safety_state.current_mode == SAFETY_OVERRIDE) ||
            (_safety_permits.emergency_override_permitted);

        if (_limit_sensor_monitor.zeroing_mode_active) {
            _limit_sensor_monitor.last_zeroing_time = current_time;
        }
    }

    // Set fault flags based on limit sensor issues
    if (sensor_faults > 2 || redundancy_faults > 1 || timeout_faults > 2) {
        _safety_state.active_faults |= FAULT_LIMIT_SENSOR;
        _safety_state.active_faults |= FAULT_SENSOR;
        PX4_WARN_THROTTLE(5000, "Limit sensor system degraded: faults=%d, redundancy=%d, timeouts=%d",
                         sensor_faults, redundancy_faults, timeout_faults);
    } else {
        _safety_state.active_faults &= ~FAULT_LIMIT_SENSOR;
    }

    // Log critical limit sensor events
    for (uint8_t i = 0; i < 8; i++) {
        auto& sensor = _limit_sensor_monitor.sensors[i];

        // Log limit activations for critical sensors (boom/bucket limits)
        if (sensor.state && sensor.healthy &&
           (sensor.sensor_type == 2 || sensor.sensor_type == 3)) { // boom min/max
            PX4_INFO_THROTTLE(2000, "Boom limit sensor %d activated (type %d)",
                             i, sensor.sensor_type);
        }
        if (sensor.state && sensor.healthy &&
           (sensor.sensor_type == 0 || sensor.sensor_type == 1)) { // bucket min/max
            PX4_INFO_THROTTLE(2000, "Bucket limit sensor %d activated (type %d)",
                             i, sensor.sensor_type);
        }
    }

    PX4_DEBUG("Limit sensor monitoring: healthy=%s, faults=%d, health_score=%.2f",
             all_healthy ? "yes" : "no", sensor_faults, _limit_sensor_monitor.sensor_health_score);
}

void SafetyManager::update_safety_mode()
{
    uint64_t current_time = hrt_absolute_time();
    bool mode_changed = false;

    // Check for zeroing mode activation via vehicle commands
    vehicle_command_s cmd;
    if (_vehicle_command_sub.copy(&cmd)) {
        // Process zeroing commands (bucket zeroing = command 1000, boom zeroing = command 1001)
        if (cmd.command == 1000 || cmd.command == 1001) {  // Custom zeroing commands
            if (cmd.param1 == 1.0f) {  // param1 = 1 means start zeroing
                if (_safety_state.current_mode != SAFETY_OVERRIDE) {
                    _safety_state.current_mode = SAFETY_OVERRIDE;
                    _safety_permits.emergency_override_permitted = true;
                    mode_changed = true;
                    PX4_INFO("Zeroing mode activated for %s (command %d)",
                           (cmd.command == 1000) ? "bucket" : "boom", (int)cmd.command);
                }
            } else if (cmd.param1 == 0.0f) {  // param1 = 0 means stop zeroing
                if (_safety_state.current_mode == SAFETY_OVERRIDE) {
                    _safety_state.current_mode = SAFETY_MONITORING;
                    _safety_permits.emergency_override_permitted = false;
                    mode_changed = true;
                    PX4_INFO("Zeroing mode deactivated for %s (command %d)",
                           (cmd.command == 1000) ? "bucket" : "boom", (int)cmd.command);
                }
            }
        }

        // Handle emergency override commands
        if (cmd.command == MAV_CMD_DO_SET_MODE && cmd.param1 == MAV_MODE_FLAG_SAFETY_ARMED) {
            if (_safety_state.current_mode != SAFETY_OVERRIDE) {
                _safety_state.current_mode = SAFETY_OVERRIDE;
                _safety_permits.emergency_override_permitted = true;
                mode_changed = true;
                PX4_WARN("Emergency safety override activated");
            }
        }

        // Handle return to normal mode
        if (cmd.command == MAV_CMD_DO_SET_MODE && cmd.param1 == MAV_MODE_FLAG_AUTO_ENABLED) {
            if (_safety_state.current_mode == SAFETY_OVERRIDE) {
                _safety_state.current_mode = SAFETY_MONITORING;
                _safety_permits.emergency_override_permitted = false;
                mode_changed = true;
                PX4_INFO("Returning to normal monitoring mode");
            }
        }
    }

    // Auto-timeout for zeroing mode (safety feature)
    if (_safety_state.current_mode == SAFETY_OVERRIDE &&
        _limit_sensor_monitor.last_zeroing_time > 0 &&
        current_time - _limit_sensor_monitor.last_zeroing_time > 30_s) {  // 30 second timeout

        _safety_state.current_mode = SAFETY_MONITORING;
        _safety_permits.emergency_override_permitted = false;
        mode_changed = true;
        PX4_WARN("Zeroing mode timed out after 30 seconds - returning to monitoring");
    }

    // Update safety override state based on current mode
    _safety_state.safety_override_active = (_safety_state.current_mode == SAFETY_OVERRIDE);

    // Update limit sensor monitoring state
    _limit_sensor_monitor.zeroing_mode_active = _safety_state.safety_override_active;

    // Log mode changes
    if (mode_changed) {
        const char* mode_str = (_safety_state.current_mode == SAFETY_OVERRIDE) ? "OVERRIDE" : "MONITORING";
        PX4_INFO("Safety mode changed to %s", mode_str);

        // Reset the zeroing time when entering override mode
        if (_safety_state.current_mode == SAFETY_OVERRIDE) {
            _limit_sensor_monitor.last_zeroing_time = current_time;
        }
    }

    PX4_DEBUG("Safety mode: %s, override_active: %s, zeroing_active: %s",
             (_safety_state.current_mode == SAFETY_OVERRIDE) ? "OVERRIDE" : "MONITORING",
             _safety_state.safety_override_active ? "yes" : "no",
             _limit_sensor_monitor.zeroing_mode_active ? "yes" : "no");
}

void SafetyManager::assess_overall_safety()
{
    // Calculate overall risk factor
    calculate_risk_factors();

    // Determine safety level based on active faults and risk
    if (_safety_state.active_faults & (FAULT_HARDWARE | FAULT_STABILITY)) {
        _safety_state.current_level = SAFETY_EMERGENCY;
    } else if (_safety_state.overall_risk_factor > _risk_threshold.get()) {
        _safety_state.current_level = SAFETY_CRITICAL;
    } else if (_safety_state.active_faults != FAULT_NONE) {
        if (_safety_state.overall_risk_factor > 0.5f) {
            _safety_state.current_level = SAFETY_WARNING;
        } else {
            _safety_state.current_level = SAFETY_CAUTION;
        }
    } else {
        _safety_state.current_level = SAFETY_NORMAL;
    }

    // Update safety mode based on level
    switch (_safety_state.current_level) {
        case SAFETY_NORMAL:
        case SAFETY_CAUTION:
            _safety_state.current_mode = SAFETY_MONITORING;
            break;

        case SAFETY_WARNING:
            _safety_state.current_mode = SAFETY_INTERVENTION;
            break;

        case SAFETY_CRITICAL:
            _safety_state.current_mode = SAFETY_OVERRIDE;
            break;

        case SAFETY_EMERGENCY:
            _safety_state.current_mode = SAFETY_SHUTDOWN;
            break;
    }
}

void SafetyManager::calculate_risk_factors()
{
    float risk = 0.0f;

    // Speed-related risk
    if (_speed_monitor.speed_limit_exceeded) risk += 0.3f;
    if (_speed_monitor.hard_braking_detected) risk += 0.2f;

    // Steering-related risk
    if (_steering_monitor.angle_limit_exceeded) risk += 0.2f;
    if (_steering_monitor.rate_limit_exceeded) risk += 0.1f;
    if (_steering_monitor.steering_error_detected) risk += 0.15f;

    // Stability-related risk
    if (_stability_monitor.stability_warning) risk += 0.4f;
    if (_stability_monitor.rollover_risk) risk += 0.6f;
    if (_stability_monitor.tip_over_detected) risk += 1.0f;

    // Load-related risk
    if (_load_monitor.overload_detected) risk += 0.3f;
    if (_load_monitor.unsafe_load_distribution) risk += 0.25f;
    if (_load_monitor.load_shift_detected) risk += 0.2f;

    // Communication-related risk
    if (_comm_monitor.manual_control_timeout) risk += 0.4f;
    if (_comm_monitor.communication_quality < 0.5f) risk += 0.3f;

    // Hardware-related risk
    if (_hardware_monitor.critical_failures > 0) risk += 0.8f;
    if (_hardware_monitor.hardware_health_score < 0.7f) risk += 0.3f;

    // Sensor-related risk
    if (_sensor_monitor.sensor_confidence < 0.6f) risk += 0.3f;

    // Terrain-related risk
    if (_terrain_monitor.slope_limit_exceeded) risk += 0.5f;
    if (_terrain_monitor.poor_traction) risk += 0.3f;
    if (_terrain_monitor.unsafe_terrain) risk += 0.2f;

    // Limit sensor-related risk
    if (_limit_sensor_monitor.sensor_health_score < 0.7f) risk += 0.3f;
    if (_limit_sensor_monitor.redundancy_faults > 1) risk += 0.4f;
    if (_limit_sensor_monitor.timeout_faults > 2) risk += 0.5f;
    if (!_limit_sensor_monitor.system_healthy) risk += 0.2f;

    _safety_state.overall_risk_factor = math::min(risk, 1.0f);
}

void SafetyManager::update_safety_level()
{
    static SafetyLevel last_level = SAFETY_NORMAL;

    if (_safety_state.current_level != last_level) {
        PX4_WARN("Safety level changed: %d -> %d (Risk: %.2f)",
                last_level, _safety_state.current_level, (double)_safety_state.overall_risk_factor);
        last_level = _safety_state.current_level;
        _safety_state.last_fault_time = hrt_absolute_time();
    }
}

void SafetyManager::check_safety_interlocks()
{
    // Update safety permits based on current conditions
    _safety_permits.motion_permitted = check_motion_permit();
    _safety_permits.steering_permitted = check_steering_permit();
    _safety_permits.autonomous_permitted = check_autonomous_permit();
    _safety_permits.load_operation_permitted = check_load_operation_permit();
    _safety_permits.high_speed_permitted = (_safety_state.current_level <= SAFETY_CAUTION);

    // Check system-level permits
    _safety_permits.hydraulic_operation_permitted = check_hydraulic_operation_permit();
    _safety_permits.boom_operation_permitted = check_boom_operation_permit();
    _safety_permits.bucket_operation_permitted = check_bucket_operation_permit();
    _safety_permits.articulation_permitted = check_articulation_permit();
    _safety_permits.engine_start_permitted = check_engine_start_permit();
    _safety_permits.emergency_override_permitted = check_emergency_override_permit();

    _safety_permits.permit_update_time = hrt_absolute_time();
}

bool SafetyManager::check_motion_permit()
{
    // Motion not permitted if:
    // - Emergency stop active
    // - Critical hardware faults
    // - Severe stability issues
    // - Communication timeout in autonomous mode

    if (_safety_state.current_level == SAFETY_EMERGENCY) return false;
    if (_hardware_monitor.critical_failures > 0) return false;
    if (_stability_monitor.tip_over_detected) return false;
    if (_comm_monitor.manual_control_timeout && _safety_permits.autonomous_permitted) return false;

    return true;
}

bool SafetyManager::check_steering_permit()
{
    // Steering not permitted if:
    // - Steering hardware fault
    // - Emergency stop active
    // - Severe steering error

    if (_hardware_monitor.steering_fault) return false;
    if (_safety_state.current_level == SAFETY_EMERGENCY) return false;
    if (_steering_monitor.steering_error_detected &&
        fabsf(_steering_monitor.current_angle_rad - _steering_monitor.commanded_angle_rad) > 0.2f) return false;

    return true;
}

bool SafetyManager::check_autonomous_permit()
{
    // Autonomous not permitted if:
    // - Any hardware faults
    // - Sensor confidence too low
    // - Communication issues
    // - Unsafe terrain

    if (_hardware_monitor.hardware_failures > 0) return false;
    if (_sensor_monitor.sensor_confidence < 0.8f) return false;
    if (_comm_monitor.communication_quality < 0.9f) return false;
    if (_terrain_monitor.unsafe_terrain) return false;
    if (_safety_state.current_level > SAFETY_CAUTION) return false;

    return true;
}

bool SafetyManager::check_load_operation_permit()
{
    // Load operations not permitted if:
    // - Overload detected
    // - Unsafe load distribution
    // - Unstable conditions

    if (_load_monitor.overload_detected) return false;
    if (_load_monitor.unsafe_load_distribution) return false;
    if (_stability_monitor.stability_warning) return false;
    if (_terrain_monitor.slope_limit_exceeded) return false;

    return true;
}

bool SafetyManager::check_electric_actuator_permit()
{
    // Electric actuator operations not permitted if:
    // - Electric actuator system faults
    // - Overheating conditions
    // - Voltage/current limit conditions
    // - Emergency conditions

    if (_electric_actuator_monitor.voltage_fault || _electric_actuator_monitor.current_fault) return false;
    if (_electric_actuator_monitor.temperature_fault) return false;
    if (_electric_actuator_monitor.controller_fault || _electric_actuator_monitor.power_supply_fault) return false;
    if (_safety_state.current_level >= SAFETY_CRITICAL) return false;
    if (_electric_actuator_monitor.actuator_health_score < 0.5f) return false;

    return true;
}

bool SafetyManager::check_boom_operation_permit()
{
    // Boom operations not permitted if:
    // - Boom system faults
    // - Boom angle limits exceeded
    // - Electric actuator system issues
    // - Unstable conditions

    if (_boom_monitor.boom_motor_fault || _boom_monitor.boom_encoder_fault) return false;
    if (_boom_monitor.boom_limit_exceeded) return false;
    if (!check_electric_actuator_permit()) return false;
    if (_stability_monitor.stability_warning) return false;
    if (_safety_state.current_level >= SAFETY_CRITICAL) return false;

    return true;
}

bool SafetyManager::check_bucket_operation_permit()
{
    // Bucket operations not permitted if:
    // - Bucket system faults
    // - Bucket angle limits exceeded
    // - Electric actuator system issues
    // - Unstable conditions

    if (_bucket_monitor.bucket_motor_fault || _bucket_monitor.bucket_encoder_fault) return false;
    if (_bucket_monitor.bucket_limit_exceeded) return false;
    if (!check_electric_actuator_permit()) return false;
    if (_stability_monitor.stability_warning) return false;
    if (_safety_state.current_level >= SAFETY_CRITICAL) return false;

    return true;
}

bool SafetyManager::check_articulation_permit()
{
    // Articulation not permitted if:
    // - Articulation system faults
    // - Angle or rate limits exceeded
    // - High speed conditions
    // - Unstable terrain

    if (_articulation_monitor.articulation_fault) return false;
    if (_articulation_monitor.angle_limit_exceeded || _articulation_monitor.rate_limit_exceeded) return false;
    if (_speed_monitor.current_speed_ms > 2.0f) return false; // No articulation at high speed
    if (_terrain_monitor.slope_limit_exceeded) return false;
    if (_safety_state.current_level >= SAFETY_CRITICAL) return false;

    return true;
}

bool SafetyManager::check_engine_start_permit()
{
    // Engine start not permitted if:
    // - Critical hardware faults
    // - Emergency conditions active
    // - Power system faults
    // - Operator interface issues

    if (_hardware_monitor.critical_failures > 0) return false;
    if (_safety_state.current_level == SAFETY_EMERGENCY) return false;
    if (_power_monitor.power_fault) return false;
    if (!_safety_permits.manual_override_active) return false; // Require operator presence

    return true;
}

bool SafetyManager::check_emergency_override_permit()
{
    // Emergency override permit always allowed if:
    // - Operator is actively present
    // - Communication is functional
    // - Manual control inputs are valid

    if (!_safety_permits.manual_override_active) return false;
    if (_comm_monitor.communication_quality < 0.5f) return false;

    return true;
}

void SafetyManager::execute_fail_safe_actions()
{
    switch (_safety_state.current_level) {
        case SAFETY_WARNING:
            apply_speed_limiting();
            _safety_performance.safety_interventions++;
            break;

        case SAFETY_CRITICAL:
            apply_speed_limiting();
            apply_steering_limiting();
            initiate_controlled_stop();
            _safety_performance.safety_interventions++;
            break;

        case SAFETY_EMERGENCY:
            activate_emergency_stop();
            _safety_performance.safety_interventions++;
            break;

        default:
            break;
    }
}

void SafetyManager::apply_speed_limiting()
{
    // Publish speed limit command
    vehicle_command_s safety_command{};
    safety_command.timestamp = hrt_absolute_time();
    safety_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_PARAMETER;
    safety_command.param1 = 1.0f; // Speed limit parameter
    safety_command.param2 = _speed_monitor.max_safe_speed_ms * 0.8f; // 80% of safe speed
    safety_command.source_system = 1;
    safety_command.source_component = 100; // Safety manager component ID
    safety_command.target_system = 1;
    safety_command.target_component = 0; // Chassis controller

    _safety_command_pub.publish(safety_command);
}

void SafetyManager::apply_steering_limiting()
{
    // Reduce steering authority
    vehicle_command_s safety_command{};
    safety_command.timestamp = hrt_absolute_time();
    safety_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_PARAMETER;
    safety_command.param1 = 2.0f; // Steering limit parameter
    safety_command.param2 = _steering_monitor.max_safe_angle_rad * 0.7f; // 70% of max angle
    safety_command.source_system = 1;
    safety_command.source_component = 100;
    safety_command.target_system = 1;
    safety_command.target_component = 0;

    _safety_command_pub.publish(safety_command);
}

void SafetyManager::initiate_controlled_stop()
{
    if (!_emergency_response.controlled_stop_active) {
        _emergency_response.controlled_stop_active = true;

        vehicle_command_s safety_command{};
        safety_command.timestamp = hrt_absolute_time();
        safety_command.command = vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM;
        safety_command.source_system = 1;
        safety_command.source_component = 100;
        safety_command.target_system = 1;
        safety_command.target_component = 0;

        _safety_command_pub.publish(safety_command);

        PX4_WARN("Controlled stop initiated by safety system");
    }
}

void SafetyManager::activate_emergency_stop()
{
    if (!_emergency_response.emergency_stop_commanded) {
        _emergency_response.emergency_stop_commanded = true;
        _emergency_response.emergency_start_time = hrt_absolute_time();
        _emergency_response.emergency_activations++;

        vehicle_command_s safety_command{};
        safety_command.timestamp = hrt_absolute_time();
        safety_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        safety_command.param1 = 3.0f; // Emergency mode
        safety_command.source_system = 1;
        safety_command.source_component = 100;
        safety_command.target_system = 1;
        safety_command.target_component = 0;

        _safety_command_pub.publish(safety_command);

        PX4_ERR("EMERGENCY STOP activated by safety system");
    }
}

void SafetyManager::handle_emergency_conditions()
{
    execute_emergency_shutdown();
    activate_backup_systems();
    send_emergency_alerts();
}

void SafetyManager::execute_emergency_shutdown()
{
    if (!_emergency_response.emergency_shutdown_active) {
        _emergency_response.emergency_shutdown_active = true;

        // Command immediate stop
        vehicle_command_s safety_command{};
        safety_command.timestamp = hrt_absolute_time();
        safety_command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        safety_command.param1 = 0.0f; // Disarm
        safety_command.param2 = 21196.0f; // Force disarm
        safety_command.source_system = 1;
        safety_command.source_component = 100;
        safety_command.target_system = 1;
        safety_command.target_component = 0;

        _safety_command_pub.publish(safety_command);

        PX4_ERR("Emergency shutdown executed");
    }
}

void SafetyManager::activate_backup_systems()
{
    // Activate backup systems if available
    _emergency_response.backup_systems_active = true;

    // In a real implementation, this would activate:
    // - Backup power systems
    // - Emergency lighting
    // - Backup communications
    // - Emergency beacon
}

void SafetyManager::send_emergency_alerts()
{
    // Send emergency alerts to operators
    static uint64_t last_alert_time = 0;
    uint64_t current_time = hrt_absolute_time();

    // Send alert every 5 seconds during emergency
    if (current_time - last_alert_time > 5e6) {
        PX4_ERR("EMERGENCY: Safety system active - Faults: 0x%X, Risk: %.2f",
                _safety_state.active_faults, (double)_safety_state.overall_risk_factor);
        last_alert_time = current_time;
    }
}

void SafetyManager::attempt_fault_recovery()
{
    // Attempt automatic recovery for non-critical faults
    static uint64_t last_recovery_attempt = 0;
    uint64_t current_time = hrt_absolute_time();

    // Only attempt recovery every 10 seconds
    if (current_time - last_recovery_attempt < 10e6) {
        return;
    }

    last_recovery_attempt = current_time;

    // Reset non-critical violations if conditions have improved
    if (_safety_state.overall_risk_factor < 0.3f) {
        if (_safety_state.active_faults & FAULT_SPEED_LIMIT && !_speed_monitor.speed_limit_exceeded) {
            _safety_state.active_faults &= ~FAULT_SPEED_LIMIT;
        }

        if (_safety_state.active_faults & FAULT_STEERING_LIMIT && !_steering_monitor.angle_limit_exceeded) {
            _safety_state.active_faults &= ~FAULT_STEERING_LIMIT;
        }

        if (_safety_state.active_faults & FAULT_COMMUNICATION && _comm_monitor.communication_quality > 0.8f) {
            _safety_state.active_faults &= ~FAULT_COMMUNICATION;
        }
    }
}

void SafetyManager::reset_safety_violations()
{
    _speed_monitor.speed_violations = 0;
    _steering_monitor.steering_violations = 0;
    _stability_monitor.stability_violations = 0;
    _load_monitor.load_violations = 0;
    _terrain_monitor.terrain_violations = 0;
    _comm_monitor.communication_failures = 0;
    _hardware_monitor.hardware_failures = 0;
    _sensor_monitor.sensor_failures = 0;
    _safety_state.safety_violation_count = 0;

    PX4_INFO("Safety violations reset");
}

void SafetyManager::validate_system_recovery()
{
    // Validate that system has recovered and is safe to resume operation
    bool recovery_valid = true;

    if (_safety_state.active_faults != FAULT_NONE) recovery_valid = false;
    if (_safety_state.overall_risk_factor > 0.2f) recovery_valid = false;
    if (_hardware_monitor.critical_failures > 0) recovery_valid = false;
    if (_sensor_monitor.sensor_confidence < 0.8f) recovery_valid = false;

    if (recovery_valid && _emergency_response.emergency_stop_commanded) {
        _emergency_response.emergency_stop_commanded = false;
        _emergency_response.controlled_stop_active = false;
        _emergency_response.emergency_shutdown_active = false;

        PX4_INFO("System recovery validated - emergency conditions cleared");
    }
}

int SafetyManager::print_status()
{
    PX4_INFO("=== Safety Manager Status ===");
    PX4_INFO("Safety Level: %d", _safety_state.current_level);
    PX4_INFO("Safety Mode: %d", _safety_state.current_mode);
    PX4_INFO("Active Faults: 0x%X", _safety_state.active_faults);
    PX4_INFO("Overall Risk: %.2f", (double)_safety_state.overall_risk_factor);
    PX4_INFO("Emergency Active: %s", _emergency_response.emergency_stop_commanded ? "YES" : "NO");

    PX4_INFO("\n=== Safety Permits ===");
    PX4_INFO("Motion: %s", _safety_permits.motion_permitted ? "GRANTED" : "DENIED");
    PX4_INFO("Steering: %s", _safety_permits.steering_permitted ? "GRANTED" : "DENIED");
    PX4_INFO("Autonomous: %s", _safety_permits.autonomous_permitted ? "GRANTED" : "DENIED");
    PX4_INFO("Load Ops: %s", _safety_permits.load_operation_permitted ? "GRANTED" : "DENIED");
    PX4_INFO("High Speed: %s", _safety_permits.high_speed_permitted ? "GRANTED" : "DENIED");

    PX4_INFO("\n=== Monitoring Status ===");
    PX4_INFO("Speed: %.2f m/s (Limit: %.2f, Exceeded: %s)",
             (double)_speed_monitor.current_speed_ms,
             (double)_speed_monitor.speed_limit_ms,
             _speed_monitor.speed_limit_exceeded ? "YES" : "NO");

    PX4_INFO("Steering: %.1f deg (Limit: %.1f, Exceeded: %s)",
             (double)(_steering_monitor.current_angle_rad * 180.0f / M_PI),
             (double)(_steering_monitor.max_safe_angle_rad * 180.0f / M_PI),
             _steering_monitor.angle_limit_exceeded ? "YES" : "NO");

    PX4_INFO("Stability Margin: %.2f (Warning: %s)",
             (double)_stability_monitor.stability_margin,
             _stability_monitor.stability_warning ? "YES" : "NO");

    PX4_INFO("Load: %.1f kg (Limit: %.1f, Overload: %s)",
             (double)_load_monitor.payload_mass_kg,
             (double)_load_monitor.max_safe_payload_kg,
             _load_monitor.overload_detected ? "YES" : "NO");

    PX4_INFO("Communication Quality: %.2f", (double)_comm_monitor.communication_quality);
    PX4_INFO("Hardware Health: %.2f", (double)_hardware_monitor.hardware_health_score);
    PX4_INFO("Sensor Confidence: %.2f", (double)_sensor_monitor.sensor_confidence);

    PX4_INFO("\n=== Performance ===");
    PX4_INFO("Safety Check Time: %.2f ms", (double)_safety_performance.safety_check_time_ms);
    PX4_INFO("Total Checks: %u", _safety_performance.total_safety_checks);
    PX4_INFO("Safety Interventions: %u", _safety_performance.safety_interventions);
    PX4_INFO("Emergency Activations: %u", _emergency_response.emergency_activations);

    return 0;
}

int SafetyManager::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        PX4_WARN("Safety Manager not running");
        return 1;
    }

    if (!strcmp(argv[0], "status")) {
        return get_instance()->print_status();
    }

    if (!strcmp(argv[0], "reset_violations")) {
        get_instance()->reset_safety_violations();
        return 0;
    }

    if (!strcmp(argv[0], "emergency")) {
        get_instance()->activate_emergency_stop();
        return 0;
    }

    if (!strcmp(argv[0], "recovery")) {
        get_instance()->validate_system_recovery();
        return 0;
    }

    return print_usage("unknown command");
}

int SafetyManager::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Safety Manager for chassis control system.

Provides comprehensive safety monitoring including:
- Speed and steering limit monitoring
- Stability and rollover protection
- Load condition monitoring
- Communication timeout detection
- Hardware health monitoring
- Sensor validity checking
- Terrain safety assessment
- Emergency response procedures

### Implementation
The safety manager operates independently and can override
any control commands to ensure safe operation.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("safety_manager", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print detailed safety status");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset_violations", "Reset safety violation counters");
    PRINT_MODULE_USAGE_COMMAND_DESCR("emergency", "Activate emergency stop");
    PRINT_MODULE_USAGE_COMMAND_DESCR("recovery", "Validate system recovery");
    PRINT_MODULE_USAGE_COMMAND("stop");

    return 0;
}

extern "C" __EXPORT int safety_manager_main(int argc, char *argv[])
{
    return SafetyManager::main(argc, argv);
}
