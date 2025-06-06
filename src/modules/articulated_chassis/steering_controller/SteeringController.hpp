#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/pid/pid.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix.hpp>

// uORB message includes
#include <uORB/topics/WheelLoaderSetpoint.h>
#include <uORB/topics/WheelLoaderStatus.h>
#include <uORB/topics/SteeringStatus.h>
#include <uORB/topics/ModuleStatus.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

/**
 * @brief Steering Controller for articulated wheel loader
 *
 * Controls the steering servo via ST3125 servo controller with:
 * - PID position control with rate limiting
 * - Articulation angle feedback
 * - Safety monitoring and limits
 * - Controller health reporting
 */
class SteeringController : public ModuleBase<SteeringController>, public ModuleParams
{
public:
    SteeringController();
    ~SteeringController() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    bool init();

    int print_status() override;

private:
    static constexpr float CONTROL_RATE_HZ = 50.0f;
    static constexpr uint64_t CONTROL_INTERVAL_US = 1_s / CONTROL_RATE_HZ;

    // Steering limits and constants
    static constexpr float MAX_STEERING_ANGLE_DEG = 45.0f;
    static constexpr float MIN_STEERING_ANGLE_DEG = -45.0f;
    static constexpr float MAX_STEERING_RATE_DEG_S = 30.0f;
    static constexpr float SERVO_PWM_CENTER = 1500.0f;
    static constexpr float SERVO_PWM_RANGE = 500.0f;
    static constexpr float ANGLE_DEADBAND_DEG = 0.5f;

    // Safety thresholds
    static constexpr float MAX_POSITION_ERROR_DEG = 10.0f;
    static constexpr uint64_t WATCHDOG_TIMEOUT_US = 1_s;
    static constexpr float MAX_SERVO_CURRENT_MA = 2000.0f;

    void run() override;

    /**
     * Control functions
     */
    void update_steering_control();
    void apply_rate_limiting();
    void update_controller_status();
    void check_safety_limits();
    void communicate_with_servo();

    /**
     * Position feedback processing
     */
    void process_position_feedback();
    float get_articulation_angle_deg();

    /**
     * Utility functions
     */
    float angle_to_pwm(float angle_deg);
    float pwm_to_angle(float pwm_us);
    float normalize_angle(float angle_deg);
    bool is_emergency_stop_active();

    // uORB subscriptions
    uORB::Subscription _wheel_loader_setpoint_sub{ORB_ID(wheel_loader_setpoint)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

    // uORB publications
    uORB::Publication<wheel_loader_status_s> _wheel_loader_status_pub{ORB_ID(wheel_loader_status)};
    uORB::Publication<steering_status_s> _steering_status_pub{ORB_ID(steering_status)};
    uORB::Publication<module_status_s> _module_status_pub{ORB_ID(module_status)};

    // Control system
    PID_t _position_pid;

    // State variables
    float _current_angle_deg{0.0f};
    float _target_angle_deg{0.0f};
    float _rate_limited_target_deg{0.0f};
    float _servo_pwm_us{SERVO_PWM_CENTER};
    uint64_t _last_position_time{0};

    // Rate limiting
    float _max_rate_deg_per_cycle{0.0f};
    float _previous_target_deg{0.0f};

    // Safety state
    bool _emergency_stop{false};
    bool _module_healthy{true};
    float _servo_current_ma{0.0f};
    uint64_t _last_command_time{0};
    bool _position_feedback_valid{false};

    // Controller status
    module_status_s _status{};
    steering_status_s _steering_status{};
    wheel_loader_status_s _wheel_loader_status{};

    // Performance monitoring
    struct {
        float position_error_rms{0.0f};
        float tracking_error_max{0.0f};
        uint32_t safety_violations{0};
        uint32_t command_timeouts{0};
        float response_time_avg{0.0f};
    } _performance;

    // Parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::STEER_P_GAIN>) _position_p_gain,
        (ParamFloat<px4::params::STEER_I_GAIN>) _position_i_gain,
        (ParamFloat<px4::params::STEER_D_GAIN>) _position_d_gain,
        (ParamFloat<px4::params::STEER_I_MAX>) _position_i_max,
        (ParamFloat<px4::params::STEER_MAX_ANGLE>) _max_steering_angle,
        (ParamFloat<px4::params::STEER_MAX_RATE>) _max_steering_rate,
        (ParamFloat<px4::params::STEER_DEADBAND>) _steering_deadband,
        (ParamFloat<px4::params::STEER_PWM_MIN>) _servo_pwm_min,
        (ParamFloat<px4::params::STEER_PWM_MAX>) _servo_pwm_max,
        (ParamFloat<px4::params::STEER_TRIM>) _steering_trim,
        (ParamBool<px4::params::STEER_REVERSE>) _steering_reverse,
        (ParamFloat<px4::params::STEER_CURR_LIMIT>) _servo_current_limit
    )
};
