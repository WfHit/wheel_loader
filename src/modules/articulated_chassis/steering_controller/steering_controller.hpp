#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix.hpp>

// uORB message includes
#include <uORB/topics/WheelLoaderSetpoint.h>
#include <uORB/topics/WheelLoaderStatus.h>
#include <uORB/topics/SteeringStatus.h>
#include <uORB/topics/RoboticServoCommand.h>
#include <uORB/topics/ServoFeedback.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/PredictiveTraction.h>

using namespace time_literals;

/**
 * @brief Simplified Steering Controller for articulated wheel loader
 *
 * Controls the steering servo via ST3125 servo controller with:
 * - Direct position commands (ST3125 handles internal PID)
 * - Slip compensation based on PredictiveTraction data
 * - Rate limiting and safety monitoring
 * - Feedforward control for improved dynamic response
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

    // ST3125 servo specifications
    static constexpr uint8_t ST3125_SERVO_ID = 1;
    static constexpr float ST3125_MAX_ANGLE_RAD = math::radians(45.0f);
    static constexpr float ST3125_MAX_VELOCITY_RAD_S = math::radians(60.0f);
    static constexpr float ST3125_CURRENT_LIMIT_A = 2.0f;
    static constexpr float ST3125_DEADBAND_RAD = math::radians(0.1f);

    // Safety limits
    static constexpr float MAX_POSITION_ERROR_RAD = math::radians(10.0f);
    static constexpr uint64_t COMMAND_TIMEOUT_US = 500_ms;
    static constexpr uint64_t SERVO_FEEDBACK_TIMEOUT_US = 100_ms;
    static constexpr uint64_t SENSOR_TIMEOUT_US = 200_ms;

    void run() override;

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
     * Safety and monitoring
     */
    void check_command_timeout();
    void update_status();

    // uORB subscriptions
    uORB::Subscription _wheel_loader_setpoint_sub{ORB_ID(wheel_loader_setpoint)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _predictive_traction_sub{ORB_ID(predictive_traction)};
    uORB::Subscription _servo_feedback_sub{ORB_ID(servo_feedback)};

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
        (ParamFloat<px4::params::STEER_MAX_ANGLE>) _max_steering_angle,
        (ParamFloat<px4::params::STEER_MAX_RATE>) _max_steering_rate,
        (ParamFloat<px4::params::STEER_DEADBAND>) _steering_deadband,
        (ParamFloat<px4::params::STEER_TRIM>) _steering_trim,
        (ParamBool<px4::params::STEER_REVERSE>) _steering_reverse,

        // Slip compensation
        (ParamBool<px4::params::STEER_SLIP_COMP_EN>) _slip_comp_enabled,
        (ParamFloat<px4::params::STEER_SLIP_COMP_GAIN>) _slip_comp_gain,
        (ParamFloat<px4::params::STEER_SLIP_COMP_MAX>) _slip_comp_max,

        // Feedforward
        (ParamFloat<px4::params::STEER_FF_GAIN>) _feedforward_gain,
        (ParamFloat<px4::params::STEER_FF_SPEED_SCALE>) _feedforward_speed_scale,

        // ST3125 calibration
        (ParamFloat<px4::params::STEER_PWM_MIN>) _servo_pwm_min,
        (ParamFloat<px4::params::STEER_PWM_MAX>) _servo_pwm_max,
        (ParamFloat<px4::params::STEER_CURR_LIMIT>) _servo_current_limit,

        // AS5600 sensor calibration
        (ParamInt32<px4::params::AS5600_INSTANCE_ID>) _param_as5600_instance_id,
        (ParamFloat<px4::params::AS5600_SCALE>) _param_as5600_scale,
        (ParamFloat<px4::params::AS5600_OFFSET>) _param_as5600_offset
    )
};
