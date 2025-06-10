#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/limit_sensor.h>
#include <uORB/topics/parameter_update.h>

#include <board_config.h>
#include "limit_sensor_config.h"

using namespace time_literals;

class LimitSensor : public ModuleBase<LimitSensor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    // Maximum number of instances (should match board configs)
    static constexpr int MAX_INSTANCES = 8;

    enum LimitFunction : uint8_t {
        BUCKET_LOAD = 0,
        BUCKET_DUMP = 1,
        BOOM_UP = 2,
        BOOM_DOWN = 3,
        STEERING_LEFT = 4,
        STEERING_RIGHT = 5,
        FUNCTION_DISABLED = 255
    };

    LimitSensor(uint8_t instance);
    ~LimitSensor() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    int print_status() override;

    // Get function from board config
    LimitFunction get_function() const {
        if (_board_config != nullptr) {
            return static_cast<LimitFunction>(_board_config->function);
        }
        return FUNCTION_DISABLED;
    }

    static LimitSensor *instantiate(int argc, char *argv[]);

private:
    void Run() override;

    // Static storage for multiple instances
    static LimitSensor *_instances[MAX_INSTANCES];
    static px4::atomic<uint8_t> _num_instances;

    // Instance details
    const uint8_t _instance;
    const limit_sensor_config_t* _board_config{nullptr};

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

    // Get board configuration for this instance
    const limit_sensor_config_t* get_board_config(uint8_t instance);
    const char* get_function_name(LimitFunction func);

    // Debouncing parameters (configurable via parameters)
    uint64_t _debounce_time_us{10000}; // Default 10ms, loaded from LS_DEBOUNCE_US
    static constexpr uint8_t DEBOUNCE_COUNTS = 3; // Need 3 consistent reads

    // Performance counters
    perf_counter_t _cycle_perf;
    perf_counter_t _sample_perf;
    perf_counter_t _fault_perf;

    // Polling rate (configurable via parameters)
    uint32_t _run_interval_us{5000}; // Default 200Hz, loaded from LS_POLL_RATE

    // Global configuration parameters (runtime adjustable)
    struct {
        param_t poll_rate_handle;
        param_t debounce_us_handle;
        param_t diag_enable_handle;

        int32_t poll_rate;
        int32_t debounce_us;
        int32_t diag_enable;
    } _global_params;

    void load_parameters();
    void load_global_parameters();
};
