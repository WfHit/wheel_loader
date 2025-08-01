#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

// uORB message includes - simplified
#include <uORB/topics/steering_setpoint.h>
#include <uORB/topics/robotic_servo_command.h>
#include <uORB/topics/robotic_servo_feedback.h>
#include <uORB/topics/limit_sensor.h>
#include <uORB/topics/parameter_update.h>

/**
 * @brief Simple Steering Controller for articulated wheel loader
 *
 * Essential functions only:
 * - Subscribe to steering target
 * - Convert to ST3125 servo command
 * - Check limit sensors
 * - Monitor servo state for abnormal events
 */
class SteeringController : public ModuleBase<SteeringController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    SteeringController();
    ~SteeringController() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    int print_status() override;

private:
    // Constants
    static constexpr float DEFAULT_CONTROL_RATE_HZ = 50.0f;
    static constexpr uint64_t CONTROL_INTERVAL_US = 20000; // 20ms = 50Hz

    void Run() override;

    /**
     * Core functions - simplified
     */
    void process_steering_command();
    void send_servo_command(float position_rad);
    void process_servo_feedback();
    void process_limit_sensors();
    void handle_abnormal_events();

    // Instance identification
    const uint8_t _instance{0};

    // uORB subscriptions - essential only
    uORB::Subscription _steering_setpoint_sub{ORB_ID(steering_setpoint)};
    uORB::Subscription _servo_feedback_sub{ORB_ID(robotic_servo_feedback)};
    uORB::Subscription _limit_sensor_sub{ORB_ID(limit_sensor)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

    // uORB publications - essential only
    uORB::Publication<robotic_servo_command_s> _servo_command_pub{ORB_ID(robotic_servo_command)};

    // State variables - simplified
    float _target_angle_rad{0.0f};      // Target steering angle from setpoint
    float _current_angle_rad{0.0f};     // Current servo position from feedback
    uint64_t _last_command_time{0};     // Last steering command timestamp
    uint64_t _last_feedback_time{0};    // Last servo feedback timestamp

    // Limit sensor state - simplified
    bool _left_limit_active{false};
    bool _right_limit_active{false};
    bool _limit_sensors_healthy{true};

    // Servo state - simplified
    bool _servo_healthy{true};
    uint16_t _servo_error_flags{0};
    float _servo_current_a{0.0f};
    float _servo_temperature_c{0.0f};

    // Emergency state
    bool _emergency_stop{false};

    // YAML-based parameters - simplified for essential control only
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::STEER_MAX_ANG>) _max_steering_angle,
        (ParamInt<px4::params::STEER_ST3125_ID>) _st3125_servo_id,
        (ParamFloat<px4::params::STEER_ST3125_CR>) _st3125_current_limit,
        (ParamInt<px4::params::STEER_LT_LF_ID>) _limit_left_instance,
        (ParamInt<px4::params::STEER_LT_RT_ID>) _limit_right_instance,
        (ParamFloat<px4::params::STEER_CMD_TO>) _command_timeout_ms,
        (ParamFloat<px4::params::STEER_FB_TO>) _feedback_timeout_ms
    )

    // Helper methods - simplified
    void update_parameters();
    float saturate_angle(float angle_rad);
    bool is_command_timeout();
    bool is_feedback_timeout();
};
