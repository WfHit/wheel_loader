#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/limit_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <board_config.h>

using namespace time_literals;

class LimitSensor : public ModuleBase<LimitSensor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    LimitSensor(uint8_t instance);
    ~LimitSensor() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    int print_status() override;

    static LimitSensor *instantiate(int argc, char *argv[]);

private:
    void Run() override;

    // Instance details
    const uint8_t _instance;

    // GPIO state for each switch
    struct SwitchState {
        uint32_t pin{0};
        bool configured{false};
        bool inverted{false};
        bool current_state{false};
        bool last_state{false};
        uint64_t last_change_time{0};
        uint8_t debounce_count{0};
    };

    SwitchState _switch_1;
    SwitchState _switch_2;

    // Combined sensor state
    struct {
        bool combined_state{false};
        bool last_combined_state{false};
        uint32_t activation_count{0};
        uint64_t last_activation_time{0};
        bool redundancy_fault{false};
    } _sensor_state;

    // Publications
    uORB::PublicationMulti<limit_sensor_s> _limit_sensor_pub{ORB_ID(limit_sensor)};
    orb_advert_t _pub_handle{nullptr};

    // Subscriptions
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

    // Methods
    bool configure_switch(SwitchState &switch_state, uint32_t pin, bool inverted);
    bool read_switch_state(SwitchState &switch_state);
    bool debounce_switch(SwitchState &switch_state);
    void update_combined_state();
    void check_redundancy_fault();
    void publish_state();

    // Debouncing parameters (configurable via parameters)
    uint64_t _debounce_time_us{10000}; // Default 10ms, loaded from LS_DEBOUNCE_US
    static constexpr uint8_t DEBOUNCE_COUNTS = 3; // Need 3 consistent reads

    // Performance counters
    perf_counter_t _cycle_perf;
    perf_counter_t _sample_perf;
    perf_counter_t _fault_perf;

    // Polling rate (configurable via parameters)
    uint32_t _run_interval_us{5000}; // Default 200Hz, loaded from LS_POLL_RATE

    // Global configuration parameters
    struct {
        param_t poll_rate_handle;
        param_t debounce_us_handle;
        param_t redundancy_en_handle;
        param_t diag_enable_handle;

        int32_t poll_rate;
        int32_t debounce_us;
        int32_t redundancy_en;
        int32_t diag_enable;
    } _global_params;

    // Parameters per instance (using C API for dynamic parameter names)
    struct {
        param_t gpio_pin_1_handle;
        param_t gpio_pin_2_handle;
        param_t type_handle;
        param_t invert_handle;
        param_t redundancy_handle;
        param_t enable_handle;

        int32_t gpio_pin_1;
        int32_t gpio_pin_2;
        int32_t type;
        int32_t invert;
        int32_t redundancy;
        int32_t enable;
    } _params;

    void load_parameters();
    void load_global_parameters();
};
