#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/pid/PID.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>

// uORB message includes - matching existing system
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/hbridge_command.h>
#include <uORB/topics/wheel_speeds_setpoint.h>
#include <uORB/topics/traction_control.h>
#include <uORB/topics/module_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_outputs.h>

using namespace time_literals;

/**
 * @brief Unified Wheel Controller for articulated wheel loader
 *
 * Controls front or rear wheel motor via DRV8701 H-bridge with:
 * - PID speed control using quad encoder feedback
 * - Integration with existing traction control system
 * - Safety monitoring and emergency stop
 * - Controller health reporting
 */
class WheelController : public ModuleBase<WheelController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    WheelController(uint8_t instance, bool is_front);
    ~WheelController() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    int print_status() override;

private:
    static constexpr float CONTROL_RATE_HZ = 100.0f;
    static constexpr uint64_t CONTROL_INTERVAL_US = 1_s / CONTROL_RATE_HZ;

    // Hardware limits (matching existing RearWheelController)
    static constexpr float MAX_WHEEL_SPEED_RPM = 200.0f;
    static constexpr float MIN_WHEEL_SPEED_RPM = -200.0f;
    static constexpr float MAX_PWM_OUTPUT = 1000.0f;
    static constexpr float ENCODER_RESOLUTION = 2048.0f; // quad encoder
    static constexpr float GEAR_RATIO = 50.0f;

    // Safety thresholds
    static constexpr float MAX_SPEED_ERROR_RPM = 50.0f;
    static constexpr float MAX_CURRENT_AMPS = 15.0f;
    static constexpr uint64_t WATCHDOG_TIMEOUT_US = 500_ms;

    void Run() override;

    /**
     * Control functions - matching existing interface
     */
    void update_speed_control();
    void apply_traction_control();
    void update_controller_status();
    void check_safety_limits();
    void communicate_with_hbridge();

    /**
     * Encoder processing
     */
    void process_encoder_data();
    float calculate_wheel_speed_rpm();

    /**
     * Utility functions
     */
    float saturate_pwm(float pwm_value);
    bool is_emergency_stop_active();
    void update_performance_metrics();
    void calculate_health_score();

    // Instance identification
    const uint8_t _instance;
    const bool _is_front_wheel;

    // uORB subscriptions - matching existing system
    uORB::Subscription _wheel_speeds_setpoint_sub{ORB_ID(wheel_speeds_setpoint)};
    uORB::Subscription _sensor_quad_encoder_sub{ORB_ID(sensor_quad_encoder)};
    uORB::Subscription _traction_control_sub{ORB_ID(traction_control)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

    // uORB publications
    uORB::Publication<module_status_s> _module_status_pub{ORB_ID(module_status)};
    uORB::PublicationMulti<actuator_outputs_s> _actuator_outputs_pub{ORB_ID(actuator_outputs)};
    uORB::Publication<hbridge_command_s> _hbridge_command_pub{ORB_ID(hbridge_command)};

    // Control system
    PID _speed_pid;

    // State variables - matching existing RearWheelController
    float _current_speed_rpm{0.0f};
    float _target_speed_rpm{0.0f};
    float _pwm_output{0.0f};
    int32_t _encoder_count{0};
    int32_t _encoder_count_prev{0};
    uint64_t _last_encoder_time{0};

    // Traction control state
    float _traction_limit_factor{1.0f};
    bool _slip_detected{false};
    float _slip_ratio{0.0f};
    uint64_t _slip_start_time{0};

    // Safety state
    bool _emergency_stop{false};
    bool _controller_healthy{true};
    float _motor_current{0.0f};
    uint64_t _last_command_time{0};
    bool _armed{false};

    // Module status for health reporting
    module_status_s _status{};
    float _health_score{100.0f}; // Local health score calculation

    // Performance monitoring - matching existing structure
    struct {
        float speed_error_rms{0.0f};
        float control_effort_avg{0.0f};
        uint32_t slip_events{0};
        uint32_t safety_violations{0};

        // Additional health metrics
        uint32_t missed_updates{0};
        uint32_t encoder_errors{0};
        float max_speed_error{0.0f};
        hrt_abstime last_health_check{0};
    } _performance;

    // Filters for signal processing
    math::LowPassFilter2p<float> _speed_filter{CONTROL_RATE_HZ, 10.0f};
    math::LowPassFilter2p<float> _current_filter{CONTROL_RATE_HZ, 5.0f};

    // Performance counters
    perf_counter_t _loop_perf;
    perf_counter_t _control_latency_perf;
    perf_counter_t _encoder_timeout_perf;

    // Parameters - instance specific, matching existing naming convention
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::WC_P_GAIN>) _speed_p_gain,
        (ParamFloat<px4::params::WC_I_GAIN>) _speed_i_gain,
        (ParamFloat<px4::params::WC_D_GAIN>) _speed_d_gain,
        (ParamFloat<px4::params::WC_I_MAX>) _speed_i_max,
        (ParamFloat<px4::params::WC_MAX_SPEED>) _max_wheel_speed,
        (ParamFloat<px4::params::WC_RAMP_RATE>) _speed_ramp_rate,
        (ParamInt<px4::params::WC_TRACT_EN>) _traction_control_enable,
        (ParamFloat<px4::params::WC_SLIP_THRS>) _slip_threshold,
        (ParamFloat<px4::params::WC_CURR_LIM>) _current_limit,
        (ParamFloat<px4::params::WC_GEAR_RAT>) _gear_ratio,
        (ParamInt<px4::params::WC_ENC_CPR>) _encoder_cpr
    )

    // Helper methods
    void parameters_update();
    void publish_actuator_outputs();
    void reset_emergency_stop();
    float calculate_slip_ratio(float wheel_speed, float reference_speed);
};
