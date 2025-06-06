#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix.hpp>

// uORB message includes
#include <uORB/topics/WheelSpeedsSetpoint.h>
#include <uORB/topics/SteeringSetpoint.h>
#include <uORB/topics/TractionControl.h>
#include <uORB/topics/ModuleStatus.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/actuator_armed.h>

using namespace time_literals;

/**
 * @brief Safety Manager for chassis control system
 *
 * Advanced safety monitoring and fail-safe system that:
 * - Monitors all safety-critical parameters
 * - Implements fail-safe behaviors
 * - Provides independent safety override
 * - Manages safety interlocks and permits
 * - Performs continuous safety assessment
 * - Implements emergency response procedures
 */
class SafetyManager : public ModuleBase<SafetyManager>, public ModuleParams
{
public:
    SafetyManager();
    ~SafetyManager() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    bool init();

    int print_status() override;

private:
    static constexpr float SAFETY_CHECK_RATE_HZ = 50.0f;
    static constexpr uint64_t SAFETY_CHECK_INTERVAL_US = 1_s / SAFETY_CHECK_RATE_HZ;

    // Safety levels
    enum SafetyLevel {
        SAFETY_NORMAL = 0,
        SAFETY_CAUTION = 1,
        SAFETY_WARNING = 2,
        SAFETY_CRITICAL = 3,
        SAFETY_EMERGENCY = 4
    };

    // Safety modes
    enum SafetyMode {
        SAFETY_MONITORING = 0,
        SAFETY_INTERVENTION = 1,
        SAFETY_OVERRIDE = 2,
        SAFETY_SHUTDOWN = 3
    };

    // Fault types
    enum FaultType {
        FAULT_NONE = 0,
        FAULT_SPEED_LIMIT = 1,
        FAULT_STEERING_LIMIT = 2,
        FAULT_COMMUNICATION = 4,
        FAULT_HARDWARE = 8,
        FAULT_SENSOR = 16,
        FAULT_STABILITY = 32,
        FAULT_LOAD = 64,
        FAULT_TERRAIN = 128
    };

    void run() override;

    /**
     * Safety monitoring functions
     */
    void monitor_speed_limits();
    void monitor_steering_limits();
    void monitor_stability();
    void monitor_load_conditions();
    void monitor_communication();
    void monitor_hardware_health();
    void monitor_sensor_validity();
    void monitor_terrain_conditions();

    /**
     * Safety assessment
     */
    void assess_overall_safety();
    void calculate_risk_factors();
    void update_safety_level();
    void check_safety_interlocks();

    /**
     * Fail-safe actions
     */
    void execute_fail_safe_actions();
    void apply_speed_limiting();
    void apply_steering_limiting();
    void initiate_controlled_stop();
    void activate_emergency_stop();
    void engage_stability_control();

    /**
     * Emergency procedures
     */
    void handle_emergency_conditions();
    void execute_emergency_shutdown();
    void activate_backup_systems();
    void send_emergency_alerts();

    /**
     * Safety permits and interlocks
     */
    bool check_motion_permit();
    bool check_steering_permit();
    bool check_autonomous_permit();
    bool check_load_operation_permit();

    /**
     * Recovery procedures
     */
    void attempt_fault_recovery();
    void reset_safety_violations();
    void validate_system_recovery();

    // uORB subscriptions - System status
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _module_status_sub{ORB_ID(module_status)};
    uORB::Subscription _wheel_speeds_sub{ORB_ID(wheel_speeds_setpoint)};
    uORB::Subscription _steering_setpoint_sub{ORB_ID(steering_setpoint)};
    uORB::Subscription _steering_status_sub{ORB_ID(steering_status)};

    // uORB subscriptions - Sensor data
    uORB::Subscription _imu_sub{ORB_ID(sensor_accel)};
    uORB::Subscription _gyro_sub{ORB_ID(sensor_gyro)};
    uORB::Subscription _slip_estimation_sub{ORB_ID(slip_estimation)};
    uORB::Subscription _load_aware_torque_sub{ORB_ID(load_aware_torque)};
    uORB::Subscription _terrain_adaptation_sub{ORB_ID(terrain_adaptation)};

    // uORB subscriptions - Control commands
    uORB::Subscription _manual_control_sub{ORB_ID(manual_control_input)};
    uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

    // uORB publications - Safety outputs
    uORB::Publication<vehicle_command_s> _safety_command_pub{ORB_ID(vehicle_command)};
    uORB::Publication<module_status_s> _safety_status_pub{ORB_ID(module_status)};

    // Safety state
    struct SafetyState {
        SafetyLevel current_level{SAFETY_NORMAL};
        SafetyMode current_mode{SAFETY_MONITORING};
        uint32_t active_faults{FAULT_NONE};
        uint32_t fault_history{FAULT_NONE};
        bool emergency_active{false};
        bool safety_override_active{false};
        uint64_t last_fault_time{0};
        uint64_t safety_violation_count{0};
        float overall_risk_factor{0.0f};
    } _safety_state;

    // Speed monitoring
    struct SpeedMonitoring {
        float current_speed_ms{0.0f};
        float max_safe_speed_ms{0.0f};
        float speed_limit_ms{0.0f};
        bool speed_limit_exceeded{false};
        float acceleration_ms2{0.0f};
        float max_safe_acceleration{0.0f};
        bool hard_braking_detected{false};
        uint32_t speed_violations{0};
    } _speed_monitor;

    // Steering monitoring
    struct SteeringMonitoring {
        float current_angle_rad{0.0f};
        float commanded_angle_rad{0.0f};
        float max_safe_angle_rad{0.0f};
        float steering_rate_rads{0.0f};
        float max_safe_rate_rads{0.0f};
        bool angle_limit_exceeded{false};
        bool rate_limit_exceeded{false};
        bool steering_error_detected{false};
        uint32_t steering_violations{0};
    } _steering_monitor;

    // Stability monitoring
    struct StabilityMonitoring {
        float roll_angle_rad{0.0f};
        float pitch_angle_rad{0.0f};
        float roll_rate_rads{0.0f};
        float pitch_rate_rads{0.0f};
        float lateral_acceleration_ms2{0.0f};
        float longitudinal_acceleration_ms2{0.0f};
        bool stability_warning{false};
        bool rollover_risk{false};
        bool tip_over_detected{false};
        float stability_margin{1.0f};
        uint32_t stability_violations{0};
    } _stability_monitor;

    // Load monitoring
    struct LoadMonitoring {
        float payload_mass_kg{0.0f};
        float max_safe_payload_kg{0.0f};
        float center_of_gravity_offset_m{0.0f};
        float max_cg_offset_m{0.0f};
        bool overload_detected{false};
        bool load_shift_detected{false};
        bool unsafe_load_distribution{false};
        uint32_t load_violations{0};
    } _load_monitor;

    // Communication monitoring
    struct CommunicationMonitoring {
        uint64_t last_manual_command_time{0};
        uint64_t last_vehicle_command_time{0};
        uint64_t last_module_status_time{0};
        bool manual_control_timeout{false};
        bool vehicle_command_timeout{false};
        bool module_status_timeout{false};
        uint32_t communication_failures{0};
        float communication_quality{1.0f};
    } _comm_monitor;

    // Hardware monitoring
    struct HardwareMonitoring {
        bool front_wheel_fault{false};
        bool rear_wheel_fault{false};
        bool steering_fault{false};
        bool sensor_fault{false};
        bool power_fault{false};
        bool actuator_fault{false};
        uint32_t hardware_failures{0};
        uint32_t critical_failures{0};
        float hardware_health_score{1.0f};
    } _hardware_monitor;

    // Sensor monitoring
    struct SensorMonitoring {
        bool imu_valid{false};
        bool gyro_valid{false};
        bool encoder_valid{false};
        bool load_sensor_valid{false};
        bool steering_sensor_valid{false};
        uint64_t last_imu_time{0};
        uint64_t last_gyro_time{0};
        uint64_t last_encoder_time{0};
        uint32_t sensor_failures{0};
        float sensor_confidence{1.0f};
    } _sensor_monitor;

    // Terrain monitoring
    struct TerrainMonitoring {
        float estimated_slope_rad{0.0f};
        float max_safe_slope_rad{0.0f};
        float surface_roughness{0.0f};
        float traction_coefficient{1.0f};
        bool unsafe_terrain{false};
        bool slope_limit_exceeded{false};
        bool poor_traction{false};
        uint32_t terrain_violations{0};
    } _terrain_monitor;

    // Safety permits
    struct SafetyPermits {
        bool motion_permitted{false};
        bool steering_permitted{false};
        bool autonomous_permitted{false};
        bool load_operation_permitted{false};
        bool high_speed_permitted{false};
        bool manual_override_active{false};
        uint64_t permit_update_time{0};
    } _safety_permits;

    // Emergency response
    struct EmergencyResponse {
        bool emergency_stop_commanded{false};
        bool controlled_stop_active{false};
        bool emergency_shutdown_active{false};
        bool backup_systems_active{false};
        uint64_t emergency_start_time{0};
        uint32_t emergency_activations{0};
        float emergency_deceleration_rate{5.0f}; // m/s²
    } _emergency_response;

    // Performance monitoring
    struct SafetyPerformance {
        float safety_check_time_ms{0.0f};
        uint32_t total_safety_checks{0};
        uint32_t safety_interventions{0};
        float intervention_response_time_ms{0.0f};
        float safety_system_availability{1.0f};
        uint64_t last_performance_update{0};
    } _safety_performance;

    // Parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::SM_MAX_SPEED>) _max_speed_ms,
        (ParamFloat<px4::params::SM_MAX_ACCEL>) _max_acceleration_ms2,
        (ParamFloat<px4::params::SM_MAX_STEER_ANGLE>) _max_steering_angle_rad,
        (ParamFloat<px4::params::SM_MAX_STEER_RATE>) _max_steering_rate_rads,
        (ParamFloat<px4::params::SM_MAX_ROLL_ANGLE>) _max_roll_angle_rad,
        (ParamFloat<px4::params::SM_MAX_PITCH_ANGLE>) _max_pitch_angle_rad,
        (ParamFloat<px4::params::SM_MAX_PAYLOAD>) _max_payload_kg,
        (ParamFloat<px4::params::SM_MAX_CG_OFFSET>) _max_cg_offset_m,
        (ParamFloat<px4::params::SM_MAX_SLOPE>) _max_slope_rad,
        (ParamFloat<px4::params::SM_COMM_TIMEOUT>) _communication_timeout_s,
        (ParamFloat<px4::params::SM_SENSOR_TIMEOUT>) _sensor_timeout_s,
        (ParamFloat<px4::params::SM_EMERGENCY_DECEL>) _emergency_decel_rate,
        (ParamFloat<px4::params::SM_STABILITY_MARGIN>) _stability_margin_factor,
        (ParamFloat<px4::params::SM_RISK_THRESHOLD>) _risk_threshold,
        (ParamBool<px4::params::SM_ENABLE_OVERRIDE>) _enable_safety_override,
        (ParamBool<px4::params::SM_AUTO_RECOVERY>) _enable_auto_recovery
    )
};
