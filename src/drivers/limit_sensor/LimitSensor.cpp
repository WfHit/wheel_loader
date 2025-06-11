/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "LimitSensor.hpp"

#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <inttypes.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <parameters/param.h>
#include <uORB/topics/limit_sensor.h>
#include <uORB/topics/parameter_update.h>

// External declaration of board configuration
#ifdef BOARD_HAS_LIMIT_SENSOR_CONFIG
extern const limit_sensor_config_t g_limit_sensor_config[];
#endif

// Static storage for multiple instances
LimitSensor *LimitSensor::_instances[MAX_INSTANCES] = {};
px4::atomic<uint8_t> LimitSensor::_num_instances{0};

LimitSensor::LimitSensor(uint8_t instance) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
    _instance(instance),
    _parameter_update_sub(ORB_ID(parameter_update)),
    _cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
    _sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
    _fault_perf(perf_alloc(PC_COUNT, MODULE_NAME": fault"))
{
    // Initialize switch states
    _switch_1 = {};
    _switch_2 = {};
    _sensor_state = {};
    _global_params = {};
    _run_interval_us = 10000; // Default 10ms, will be updated from parameters

    // Register this instance
    if (_instance < MAX_INSTANCES && _instances[_instance] == nullptr) {
        _instances[_instance] = this;
        _num_instances.fetch_add(1);
    }
}

LimitSensor::~LimitSensor()
{
    // Stop the work queue
    ScheduleClear();

    // Unadvertise publication
    if (_pub_handle != nullptr) {
        orb_unadvertise(_pub_handle);
    }

    // Unregister this instance
    if (_instance < MAX_INSTANCES && _instances[_instance] == this) {
        _instances[_instance] = nullptr;
        _num_instances.fetch_sub(1);
    }

    // Free performance counters
    perf_free(_cycle_perf);
    perf_free(_sample_perf);
    perf_free(_fault_perf);
}

const limit_sensor_config_t* LimitSensor::get_board_config(uint8_t instance)
{
#ifdef BOARD_HAS_LIMIT_SENSOR_CONFIG
    if (instance < BOARD_NUM_LIMIT_SENSORS) {
        return &g_limit_sensor_config[instance];
    }
#endif
    return nullptr;
}

bool LimitSensor::init()
{
    // Get board configuration for this instance
    _board_config = get_board_config(_instance);

    if (_board_config == nullptr) {
        PX4_ERR("No board configuration for instance %d", _instance);
        return false;
    }

    // Load runtime parameters
    load_parameters();
    load_global_parameters();

    // Configure primary switch
    if (_board_config->gpio_pin_1 != 0) {
        if (!configure_switch(_switch_1, _board_config->gpio_pin_1, _board_config->inverted)) {
            PX4_ERR("Failed to configure switch 1 for %s", _board_config->name);
            return false;
        }
    }

    // Configure secondary switch if defined and redundancy is enabled
    if (_board_config->gpio_pin_2 != 0 && _board_config->redundancy_enabled) {
        if (!configure_switch(_switch_2, _board_config->gpio_pin_2, _board_config->inverted)) {
            PX4_ERR("Failed to configure switch 2 for %s", _board_config->name);
            return false;
        }
    }

    // Initialize uORB publications
    limit_sensor_s msg{};
    msg.timestamp = hrt_absolute_time();
    msg.instance = _instance;
    msg.function = _board_config->function;

    int instance_copy = _instance;
    _pub_handle = orb_advertise_multi(ORB_ID(limit_sensor), &msg, &instance_copy);

    if (_pub_handle == nullptr) {
        PX4_ERR("Failed to advertise limit_sensor");
        return false;
    }

    // Start work queue
    ScheduleOnInterval(_run_interval_us);

    PX4_INFO("LimitSensor %d initialized: %s", _instance, _board_config->name);

    return true;
}

bool LimitSensor::configure_switch(SwitchState &switch_state, uint32_t pin, bool inverted)
{
    switch_state.pin = pin;
    switch_state.inverted = inverted;

    // Configure GPIO pin
#if defined(__PX4_NUTTX)
    if (px4_arch_configgpio(pin) < 0) {
        PX4_ERR("Failed to configure GPIO pin 0x%08" PRIx32, pin);
        return false;
    }
#endif

    switch_state.configured = true;
    switch_state.current_state = false;
    switch_state.last_state = false;
    switch_state.last_change_time = 0;
    switch_state.debounce_count = 0;

    return true;
}

bool LimitSensor::read_switch_state(SwitchState &switch_state)
{
    if (!switch_state.configured) {
        return false;
    }

    // Read GPIO pin
    bool pin_state = false;
#if defined(__PX4_NUTTX)
    pin_state = px4_arch_gpioread(switch_state.pin);
#elif defined(__PX4_POSIX)
    // POSIX simulation - always return false for now
    pin_state = false;
#endif

    // Apply inversion if configured
    if (switch_state.inverted) {
        pin_state = !pin_state;
    }

    switch_state.current_state = pin_state;
    return true;
}

bool LimitSensor::debounce_switch(SwitchState &switch_state)
{
    if (!switch_state.configured) {
        return false;
    }

    uint64_t now = hrt_absolute_time();

    // Check if state changed
    if (switch_state.current_state != switch_state.last_state) {
        switch_state.last_change_time = now;
        switch_state.debounce_count = 1;
        switch_state.last_state = switch_state.current_state;
        return false; // State is not stable yet
    }

    // Check if enough time has passed for debouncing
    if ((now - switch_state.last_change_time) >= _debounce_time_us) {
        if (switch_state.debounce_count < DEBOUNCE_COUNTS) {
            switch_state.debounce_count++;
            return false; // Need more consistent reads
        }
        return true; // State is stable
    }

    return false; // Still debouncing
}

void LimitSensor::update_combined_state()
{
    bool new_state = false;

    if (_board_config->redundancy_enabled && _switch_2.configured) {
        // For redundant sensors (bucket load/dump), require both switches to agree
        new_state = _switch_1.current_state && _switch_2.current_state;

        // Check for redundancy fault (switches disagree)
        if (_switch_1.current_state != _switch_2.current_state) {
            if (!_sensor_state.redundancy_fault) {
                _sensor_state.redundancy_fault = true;
                perf_count(_fault_perf);
                PX4_WARN("%s: Redundancy fault detected", _board_config->name);
            }
        } else {
            _sensor_state.redundancy_fault = false;
        }
    } else {
        // For non-redundant sensors (boom, steering), use only switch 1
        new_state = _switch_1.current_state;
        _sensor_state.redundancy_fault = false;
    }

    // Update combined state
    _sensor_state.combined_state = new_state;

    // Track state changes
    if (new_state && !_sensor_state.last_combined_state) {
        _sensor_state.activation_count++;
        _sensor_state.last_activation_time = hrt_absolute_time();
    }

    _sensor_state.last_combined_state = new_state;
}

void LimitSensor::check_redundancy_fault()
{
    // This is handled in update_combined_state()
}

void LimitSensor::publish_state()
{
    if (_pub_handle == nullptr) {
        return;
    }

    limit_sensor_s msg{};
    msg.timestamp = hrt_absolute_time();
    msg.instance = _instance;
    msg.function = _board_config->function;
    msg.state = _sensor_state.combined_state;
    msg.switch_1_state = _switch_1.current_state;
    msg.switch_2_state = _switch_2.configured ? _switch_2.current_state : false;
    msg.redundancy_enabled = _board_config->redundancy_enabled;
    msg.redundancy_fault = _sensor_state.redundancy_fault;
    msg.activation_count = _sensor_state.activation_count;
    msg.last_activation_time = _sensor_state.last_activation_time;

    orb_publish(ORB_ID(limit_sensor), _pub_handle, &msg);
}

void LimitSensor::load_parameters()
{
    updateParams();
}

void LimitSensor::load_global_parameters()
{
    // Load global parameters
    _global_params.poll_rate_handle = param_find("LS_POLL_RATE");
    _global_params.debounce_us_handle = param_find("LS_DEBOUNCE_US");
    _global_params.diag_enable_handle = param_find("LS_DIAG_ENABLE");

    if (_global_params.poll_rate_handle != PARAM_INVALID) {
        param_get(_global_params.poll_rate_handle, &_global_params.poll_rate);
        _run_interval_us = 1000000 / _global_params.poll_rate; // Convert Hz to microseconds
    }

    if (_global_params.debounce_us_handle != PARAM_INVALID) {
        param_get(_global_params.debounce_us_handle, &_global_params.debounce_us);
        _debounce_time_us = _global_params.debounce_us;
    }

    if (_global_params.diag_enable_handle != PARAM_INVALID) {
        param_get(_global_params.diag_enable_handle, &_global_params.diag_enable);
    }
}

void LimitSensor::Run()
{
    if (should_exit()) {
        ScheduleClear();
        return;
    }

    perf_begin(_cycle_perf);

    // Check for parameter updates
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        load_parameters();
        load_global_parameters();
    }

    perf_begin(_sample_perf);

    // Read switch states
    read_switch_state(_switch_1);
    if (_switch_2.configured) {
        read_switch_state(_switch_2);
    }

    // Debounce switches
    bool switch_1_stable = debounce_switch(_switch_1);
    bool switch_2_stable = _switch_2.configured ? debounce_switch(_switch_2) : true;

    // Update combined state only when switches are stable
    if (switch_1_stable && switch_2_stable) {
        update_combined_state();
    }

    // Check for faults
    check_redundancy_fault();

    // Publish state
    publish_state();

    perf_end(_sample_perf);
    perf_end(_cycle_perf);
}

int LimitSensor::print_status()
{
    if (_board_config == nullptr) {
        PX4_INFO("Instance %d: Not configured", _instance);
        return 0;
    }

    PX4_INFO("Instance %d: %s", _instance, _board_config->name);
    PX4_INFO("  Poll rate: %" PRIu32 " Hz", _global_params.poll_rate);
    PX4_INFO("  Debounce: %" PRIu32 " us", _global_params.debounce_us);
    PX4_INFO("  Switch 1: %s", _switch_1.current_state ? "ACTIVE" : "INACTIVE");
    if (_switch_2.configured) {
        PX4_INFO("  Switch 2: %s", _switch_2.current_state ? "ACTIVE" : "INACTIVE");
    }
    PX4_INFO("  Combined: %s", _sensor_state.combined_state ? "ACTIVE" : "INACTIVE");
    PX4_INFO("  Activations: %" PRIu32, _sensor_state.activation_count);
    PX4_INFO("  Run interval: %" PRIu32 " us", _run_interval_us);
    return 0;
}

int LimitSensor::task_spawn(int argc, char *argv[])
{
#ifdef BOARD_HAS_LIMIT_SENSOR_CONFIG
    // Start all configured instances
    bool any_started = false;
    for (int i = 0; i < BOARD_NUM_LIMIT_SENSORS; i++) {
        LimitSensor *obj = new LimitSensor(i);

        if (obj == nullptr) {
            PX4_ERR("alloc failed for instance %d", i);
            continue;
        }

        if (obj->init()) {
            // For the first instance, store it as the primary object
            if (!any_started) {
                _object.store(obj);
                _task_id = task_id_is_work_queue;
            }
            any_started = true;
            PX4_INFO("Started limit sensor instance %d", i);
        } else {
            delete obj;
            PX4_ERR("Failed to start limit sensor instance %d", i);
        }
    }

    if (any_started) {
        return PX4_OK;
    } else {
        PX4_ERR("No limit sensor instances could be started");
        return PX4_ERROR;
    }
#else
    PX4_ERR("No limit sensors configured for this board");
    return PX4_ERROR;
#endif
}

LimitSensor *LimitSensor::instantiate(int argc, char *argv[])
{
    LimitSensor *obj = new LimitSensor(0);
    return obj;
}

int LimitSensor::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int LimitSensor::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Limit sensor driver for wheel loader operations.

The limit sensor driver monitors GPIO pins for limit switch states and publishes
them via uORB. It supports both single and redundant switch configurations.

### Implementation
The driver can run multiple instances, each monitoring different limit switches
based on board configuration. Switch states are debounced and published at a
configurable rate.

### Examples
Start the limit sensor driver:
$ limit_sensor start

Stop the limit sensor driver:
$ limit_sensor stop

Show status:
$ limit_sensor status
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("limit_sensor", "driver");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int limit_sensor_main(int argc, char *argv[])
{
    return LimitSensor::main(argc, argv);
}
