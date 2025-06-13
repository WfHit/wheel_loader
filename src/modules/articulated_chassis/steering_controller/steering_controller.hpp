#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>

// uORB message includes
#include <uORB/topics/steering_setpoint.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/robotic_servo_command.h>
#include <uORB/topics/robotic_servo_feedback.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/predictive_traction.h>
#include <uORB/topics/limit_sensor.h>

using namespace time_literals;

/**
 * @brief Steering Controller for articulated wheel loader
 *
 * Controls the steering servo via ST3125 servo controller with:
 * - Direct position commands (ST3125 handles internal PID)
 * - Slip compensation based on PredictiveTraction data
 * - Rate limiting and safety monitoring
 * - Feedforward control for improved dynamic response
 * - Limit sensor integration for safety
 * - Comprehensive safety management
 */
class SteeringController : public ModuleBase<SteeringController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    SteeringController();
    ~SteeringController() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static SteeringController *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    bool init();

    int print_status() override;

private:
    static constexpr float CONTROL_RATE_HZ = 50.0f;
    static constexpr uint64_t CONTROL_INTERVAL_US = 1_s / CONTROL_RATE_HZ;

    void Run() override;  // ScheduledWorkItem interface
    void run() override;  // ModuleBase interface

    /**
     * Core control functions
     */
    void process_steering_command();
    float apply_slip_compensation(float target_angle);
    float apply_feedforward(float target_angle, float current_angle);
    float rate_limit(float target, float current);

    /**
     * ST3125 servo interface
     */
    void send_servo_command(float position_rad, float velocity_rad_s = 0.0f);
    void process_servo_feedback();

    /**
     * Limit sensor and safety management
     */
    void process_limit_sensors();
    bool check_position_limits(float target_angle);
    void handle_safety_violation();
    void update_safety_state();

    /**
     * Safety and monitoring
     */
    void check_command_timeout();
    void update_status();

    // uORB subscriptions
    uORB::Subscription _steering_setpoint_sub{ORB_ID(steering_setpoint)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _predictive_traction_sub{ORB_ID(predictive_traction)};
    uORB::Subscription _servo_feedback_sub{ORB_ID(robotic_servo_feedback)};
    uORB::Subscription _limit_sensor_sub{ORB_ID(limit_sensor)};

    // uORB publications
    uORB::Publication<steering_status_s> _steering_status_pub{ORB_ID(steering_status)};
    uORB::Publication<robotic_servo_command_s> _servo_command_pub{ORB_ID(robotic_servo_command)};

    // State variables
    float _current_angle_rad{0.0f};
    float _commanded_angle_rad{0.0f};
    float _target_angle_rad{0.0f};
    float _slip_compensation_rad{0.0f};

    // Timing
    uint64_t _last_command_time{0};
    uint64_t _last_update_time{0};
    uint64_t _last_feedback_time{0};
    uint64_t _last_sensor_update{0};
    float _dt{0.02f};  // 50Hz nominal

    // Sensor state
    bool _sensor_valid{false};

    // Vehicle state
    float _vehicle_speed_mps{0.0f};
    bool _emergency_stop{false};

    // Limit sensor state
    struct {
        bool left_limit_active{false};
        bool right_limit_active{false};
        bool left_limit_healthy{false};
        bool right_limit_healthy{false};
        uint64_t left_last_update{0};
        uint64_t right_last_update{0};
        uint8_t left_instance{255};
        uint8_t right_instance{255};
    } _limit_sensors;

    // Safety manager state
    struct {
        bool safety_violation{false};
        bool position_limit_violation{false};
        bool emergency_stop_active{false};
        bool servo_fault{false};
        bool sensor_fault{false};
        float safe_position_rad{0.0f};
        uint32_t violation_count{0};
        uint64_t last_violation_time{0};
    } _safety_state;

    // Servo feedback state
    struct {
        float position_rad{0.0f};
        float velocity_rad_s{0.0f};
        float current_a{0.0f};
        float temperature_c{0.0f};
        uint16_t error_flags{0};
        bool torque_enabled{false};
        bool feedback_valid{false};
    } _servo_state;

    // Slip compensation state
    struct {
        float slip_front{0.0f};
        float slip_rear{0.0f};
        float slip_error{0.0f};
        bool traction_active{false};
        uint64_t last_update{0};
    } _slip_state;

    // Simple moving average for smooth control
    static constexpr int FILTER_SIZE = 3;
    float _angle_history[FILTER_SIZE]{};
    int _history_index{0};

    // Performance metrics
    struct {
        float max_error_rad{0.0f};
        float avg_error_rad{0.0f};
        uint32_t command_count{0};
        uint32_t timeout_count{0};
        uint32_t feedback_timeout_count{0};
        uint32_t sensor_update_count{0};
        uint32_t sensor_error_count{0};
        uint32_t sensor_timeout_count{0};
    } _metrics;

    // Parameters
    DEFINE_PARAMETERS(
        // Basic control
        (ParamFloat<px4::params::STEER_MAX_ANG>) _max_steering_angle,
        (ParamFloat<px4::params::STEER_MAX_RATE>) _max_steering_rate,
        (ParamFloat<px4::params::STEER_DEADBAND>) _steering_deadband,
        (ParamFloat<px4::params::STEER_TRIM>) _steering_trim,
        (ParamInt<px4::params::STEER_REVERSE>) _steering_reverse,

        // Slip compensation
        (ParamInt<px4::params::STEER_SLP_CP_EN>) _slip_comp_enabled,
        (ParamFloat<px4::params::STEER_SLP_CP_GN>) _slip_comp_gain,
        (ParamFloat<px4::params::STEER_SLP_CP_MA>) _slip_comp_max,

        // Feedforward
        (ParamFloat<px4::params::STEER_FF_GAIN>) _feedforward_gain,
        (ParamFloat<px4::params::STEER_FF_SPD_SC>) _feedforward_speed_scale,

        // ST3125 servo specifications
        (ParamInt<px4::params::STEER_ST3125_ID>) _st3125_servo_id,
        (ParamFloat<px4::params::STEER_ST3125_AN>) _st3125_max_angle,
        (ParamFloat<px4::params::STEER_ST3125_VL>) _st3125_max_velocity,
        (ParamFloat<px4::params::STEER_ST3125_CR>) _st3125_current_limit,
        (ParamFloat<px4::params::STEER_ST3125_DB>) _st3125_deadband,

        // Safety limits
        (ParamFloat<px4::params::STEER_MAX_POS>) _max_position_error,
        (ParamFloat<px4::params::STEER_CMD_TO>) _command_timeout_ms,
        (ParamFloat<px4::params::STEER_FB_TO>) _feedback_timeout_ms,
        (ParamFloat<px4::params::STEER_SENS_TO>) _sensor_timeout_ms,

        // Legacy servo calibration (deprecated)
        (ParamFloat<px4::params::STEER_PWM_MIN>) _servo_pwm_min,
        (ParamFloat<px4::params::STEER_PWM_MAX>) _servo_pwm_max,
        (ParamFloat<px4::params::STEER_CURR_LT>) _servo_current_limit,

        // Limit sensor configuration
        (ParamInt<px4::params::STEER_LIMIT_EN>) _limit_sensors_enabled,
        (ParamInt<px4::params::STEER_LT_LF_ID>) _limit_left_instance,
        (ParamInt<px4::params::STEER_LT_RT_ID>) _limit_right_instance,
        (ParamFloat<px4::params::STEER_LIMIT_MAR>) _limit_margin_rad,

        // Safety manager configuration
        (ParamInt<px4::params::STEER_SAFETY_EN>) _safety_manager_enabled,
        (ParamFloat<px4::params::STEER_SAFE_POS>) _safety_position_rad,
        (ParamFloat<px4::params::STEER_FT_TO>) _fault_timeout_ms,
        (ParamInt<px4::params::STEER_MAX_VIOL>) _max_violations
    )
};
