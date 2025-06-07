#include "LimitSensor.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_arch/io_timer.h>
#include <parameters/param.h>

LimitSensor::LimitSensor(uint8_t instance) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
    _instance(instance),
    _cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
    _sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
    _fault_perf(perf_alloc(PC_COUNT, MODULE_NAME": fault"))
{
    // Initialize parameter handles to invalid
    _params.gpio_pin_1_handle = PARAM_INVALID;
    _params.gpio_pin_2_handle = PARAM_INVALID;
    _params.type_handle = PARAM_INVALID;
    _params.invert_handle = PARAM_INVALID;
    _params.redundancy_handle = PARAM_INVALID;
    _params.enable_handle = PARAM_INVALID;

    // Initialize global parameter handles
    _global_params.poll_rate_handle = PARAM_INVALID;
    _global_params.debounce_us_handle = PARAM_INVALID;
    _global_params.redundancy_en_handle = PARAM_INVALID;
    _global_params.diag_enable_handle = PARAM_INVALID;

    load_global_parameters();
    load_parameters();
}

LimitSensor::~LimitSensor()
{
    perf_free(_cycle_perf);
    perf_free(_sample_perf);
    perf_free(_fault_perf);
}

void LimitSensor::load_global_parameters()
{
    // Load global parameter handles
    _global_params.poll_rate_handle = param_find("LS_POLL_RATE");
    _global_params.debounce_us_handle = param_find("LS_DEBOUNCE_US");
    _global_params.redundancy_en_handle = param_find("LS_REDUNDANCY_EN");
    _global_params.diag_enable_handle = param_find("LS_DIAG_ENABLE");

    // Load current global parameter values
    param_get(_global_params.poll_rate_handle, &_global_params.poll_rate);
    param_get(_global_params.debounce_us_handle, &_global_params.debounce_us);
    param_get(_global_params.redundancy_en_handle, &_global_params.redundancy_en);
    param_get(_global_params.diag_enable_handle, &_global_params.diag_enable);

    // Apply global parameters to instance settings
    if (_global_params.poll_rate > 0 && _global_params.poll_rate <= 1000) {
        _run_interval_us = 1000000 / _global_params.poll_rate; // Convert Hz to microseconds
    }

    if (_global_params.debounce_us >= 1000 && _global_params.debounce_us <= 100000) {
        _debounce_time_us = _global_params.debounce_us;
    }

    PX4_INFO("Instance %d: poll_rate=%d Hz, debounce=%d us",
             _instance, _global_params.poll_rate, _global_params.debounce_us);
}

void LimitSensor::load_parameters()
{
    char param_name[32];

    // Build parameter names based on instance
    snprintf(param_name, sizeof(param_name), "LS%d_GPIO_1", _instance);
    _params.gpio_pin_1_handle = param_find(param_name);

    snprintf(param_name, sizeof(param_name), "LS%d_GPIO_2", _instance);
    _params.gpio_pin_2_handle = param_find(param_name);

    snprintf(param_name, sizeof(param_name), "LS%d_TYPE", _instance);
    _params.type_handle = param_find(param_name);

    snprintf(param_name, sizeof(param_name), "LS%d_INVERT", _instance);
    _params.invert_handle = param_find(param_name);

    snprintf(param_name, sizeof(param_name), "LS%d_REDUNDANCY", _instance);
    _params.redundancy_handle = param_find(param_name);

    snprintf(param_name, sizeof(param_name), "LS%d_ENABLE", _instance);
    _params.enable_handle = param_find(param_name);

    // Load current parameter values
    param_get(_params.gpio_pin_1_handle, &_params.gpio_pin_1);
    param_get(_params.gpio_pin_2_handle, &_params.gpio_pin_2);
    param_get(_params.type_handle, &_params.type);
    param_get(_params.invert_handle, &_params.invert);
    param_get(_params.redundancy_handle, &_params.redundancy);
    param_get(_params.enable_handle, &_params.enable);
}

bool LimitSensor::init()
{
    // Load both global and instance parameters
    load_global_parameters();
    load_parameters();

    if (!_params.enable) {
        PX4_INFO("Instance %d disabled", _instance);
        return false;
    }

    uint32_t pin1 = _params.gpio_pin_1;
    uint32_t pin2 = _params.gpio_pin_2;
    bool inverted = _params.invert;

    if (!configure_switch(_switch_1, pin1, inverted)) {
        PX4_ERR("Failed to configure switch 1 for instance %d", _instance);
        return false;
    }

    if (_params.redundancy) {
        if (!configure_switch(_switch_2, pin2, inverted)) {
            PX4_ERR("Failed to configure switch 2 for instance %d", _instance);
            return false;
        }
        PX4_INFO("Instance %d configured with redundancy", _instance);
    }

    debounce_switch(_switch_1);
    if (_params.redundancy) {
        debounce_switch(_switch_2);
    }
    update_combined_state();

    ScheduleOnInterval(_run_interval_us);

    PX4_INFO("Limit sensor %d initialized (type: %d, poll: %d Hz)",
             _instance, _params.type, _global_params.poll_rate);

    return true;
}

bool LimitSensor::configure_switch(SwitchState &switch_state, uint32_t pin, bool inverted)
{
    if (pin == 0) {
        return false;
    }

    px4_arch_configgpio(pin | GPIO_INPUT | GPIO_PULLUP);

    switch_state.pin = pin;
    switch_state.configured = true;
    switch_state.inverted = inverted;
    switch_state.current_state = false;
    switch_state.last_state = false;
    switch_state.last_change_time = hrt_absolute_time();
    switch_state.debounce_count = 0;

    return true;
}

bool LimitSensor::read_switch_state(SwitchState &switch_state)
{
    if (!switch_state.configured) {
        return false;
    }

    bool raw_state = px4_arch_gpioread(switch_state.pin);
    return switch_state.inverted ? !raw_state : raw_state;
}

bool LimitSensor::debounce_switch(SwitchState &switch_state)
{
    if (!switch_state.configured) {
        return false;
    }

    bool raw_state = read_switch_state(switch_state);
    uint64_t now = hrt_absolute_time();

    if (raw_state != switch_state.last_state) {
        switch_state.last_state = raw_state;
        switch_state.last_change_time = now;
        switch_state.debounce_count = 1;
    } else if (raw_state != switch_state.current_state) {
        if ((now - switch_state.last_change_time) >= _debounce_time_us) {
            switch_state.debounce_count++;
            if (switch_state.debounce_count >= DEBOUNCE_COUNTS) {
                switch_state.current_state = raw_state;
                switch_state.debounce_count = 0;
            }
        }
    } else {
        switch_state.debounce_count = 0;
    }

    return switch_state.current_state;
}

void LimitSensor::update_combined_state()
{
    bool new_state = false;

    if (_params.redundancy && _switch_1.configured && _switch_2.configured) {
        new_state = _switch_1.current_state && _switch_2.current_state;
        check_redundancy_fault();
    } else {
        new_state = _switch_1.current_state;
        _sensor_state.redundancy_fault = false;
    }

    if (new_state && !_sensor_state.combined_state) {
        _sensor_state.activation_count++;
        _sensor_state.last_activation_time = hrt_absolute_time();
    }

    _sensor_state.last_combined_state = _sensor_state.combined_state;
    _sensor_state.combined_state = new_state;
}

void LimitSensor::check_redundancy_fault()
{
    bool fault = (_switch_1.current_state != _switch_2.current_state);

    if (fault && !_sensor_state.redundancy_fault) {
        perf_count(_fault_perf);
        PX4_WARN("Redundancy fault on limit sensor %d", _instance);
    }

    _sensor_state.redundancy_fault = fault;
}

void LimitSensor::publish_state()
{
    limit_sensor_s msg{};

    msg.timestamp = hrt_absolute_time();
    msg.instance = _instance;
    msg.type = _params.type;
    msg.state = _sensor_state.combined_state;
    msg.state_switch_1 = _switch_1.current_state;
    msg.state_switch_2 = _switch_2.current_state;
    msg.redundancy_enabled = _params.redundancy;
    msg.redundancy_fault = _sensor_state.redundancy_fault;
    msg.healthy = _switch_1.configured && (!_params.redundancy || _switch_2.configured) && !_sensor_state.redundancy_fault;
    msg.activation_count = _sensor_state.activation_count;
    msg.last_activation_time = _sensor_state.last_activation_time;

    if (_pub_handle == nullptr) {
        _pub_handle = _limit_sensor_pub.advertise();
    }

    _limit_sensor_pub.publish(msg);
}

void LimitSensor::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_cycle_perf);

    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        // Reload both global and instance parameters
        load_global_parameters();
        load_parameters();

        // Update scheduling if polling rate changed
        ScheduleClear();
        ScheduleOnInterval(_run_interval_us);

        PX4_INFO("Instance %d: Parameters reloaded", _instance);
    }

    perf_begin(_sample_perf);

    bool switch1_state = debounce_switch(_switch_1);
    bool switch2_state = false;

    if (_params.redundancy) {
        switch2_state = debounce_switch(_switch_2);
    }

    update_combined_state();
    publish_state();

    perf_end(_sample_perf);
    perf_end(_cycle_perf);
}

int LimitSensor::print_status()
{
    PX4_INFO("Limit Sensor %d:", _instance);
    PX4_INFO("  Type: %d", _params.type);
    PX4_INFO("  Enabled: %s", _params.enable ? "yes" : "no");
    PX4_INFO("  Redundancy: %s", _params.redundancy ? "enabled" : "disabled");
    PX4_INFO("  Global config: poll=%dHz, debounce=%dus",
             _global_params.poll_rate, _global_params.debounce_us);
    PX4_INFO("  Switch 1: %s (pin: 0x%08x)",
             _switch_1.configured ? (_switch_1.current_state ? "ACTIVE" : "inactive") : "not configured",
             _switch_1.pin);

    if (_params.redundancy) {
        PX4_INFO("  Switch 2: %s (pin: 0x%08x)",
                 _switch_2.configured ? (_switch_2.current_state ? "ACTIVE" : "inactive") : "not configured",
                 _switch_2.pin);
        PX4_INFO("  Redundancy fault: %s", _sensor_state.redundancy_fault ? "YES" : "no");
    }

    PX4_INFO("  Combined state: %s", _sensor_state.combined_state ? "LIMIT REACHED" : "normal");
    PX4_INFO("  Activations: %u", _sensor_state.activation_count);

    perf_print_counter(_cycle_perf);
    perf_print_counter(_sample_perf);
    perf_print_counter(_fault_perf);

    return 0;
}

LimitSensor *LimitSensor::instantiate(int argc, char *argv[])
{
    int instance = -1;
    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'i':
            instance = atoi(myoptarg);
            break;
        default:
            return nullptr;
        }
    }

    if (instance < 0 || instance >= 8) {
        PX4_ERR("Instance must be between 0 and 7");
        return nullptr;
    }

    LimitSensor *sensor = new LimitSensor(instance);

    if (sensor == nullptr) {
        PX4_ERR("Failed to allocate limit sensor %d", instance);
    }

    return sensor;
}

int LimitSensor::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int LimitSensor::task_spawn(int argc, char *argv[])
{
    LimitSensor *instance = instantiate(argc, argv);

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

int LimitSensor::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Monitor limit switches with optional redundancy. Each instance can monitor one or two switches.
When redundancy is enabled, both switches must be active to trigger the limit.

### Implementation
Uses high-rate polling (200Hz) with software debouncing.
Supports single switch or redundant dual-switch configuration per instance.

### Examples
Start instance 0 (bucket dump):
$ limit_sensor start -i 0

Start instance 1 (bucket load):
$ limit_sensor start -i 1

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("limit_sensor", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_INT('i', -1, 0, 7, "Instance number", false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int limit_sensor_main(int argc, char *argv[]);

int limit_sensor_main(int argc, char *argv[])
{
    return LimitSensor::main(argc, argv);
}
