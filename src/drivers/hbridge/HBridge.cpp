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

#include "HBridge.hpp"

#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_arch/io_timer.h>
#include <board_config.h>

// Static storage for multiple instances
HBridge *HBridge::_objects[MAX_INSTANCES] = {};
bool HBridge::_pwm_initialized = false;
px4::atomic<int> HBridge::_running_instances{0};

HBridge::HBridge(int instance_id) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
    _instance_id(instance_id),
    _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
    _command_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": command"))
{
    // Initialize channel data
    for (int i = 0; i < MAX_CHANNELS; i++) {
        _channels[i] = {};
    }
}

HBridge::~HBridge()
{
    // Disable all channels
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (_channels[i].initialized && _channels[i].pwm_mask != 0) {
            up_pwm_servo_set(_channels[i].pwm_channel, 0);
            up_pwm_servo_arm(false, _channels[i].pwm_mask);
        }
    }

    // Unadvertise status
    if (_status_pub != nullptr) {
        orb_unadvertise(_status_pub);
    }

    // Decrement running instances
    _running_instances.fetch_sub(1);

    // Deinitialize PWM if this was the last instance
    if (_running_instances.load() == 0 && _pwm_initialized) {
        up_pwm_servo_deinit(0);
        _pwm_initialized = false;
    }

    perf_free(_loop_perf);
    perf_free(_command_perf);
}

bool HBridge::init()
{
    // Load initial parameters
    parameters_update();

    // Configure GPIOs based on board configuration and instance
#if defined(DRV8701_DIR1_GPIO) && defined(DRV8701_DIR2_GPIO)
    if (_instance_id == 0) {
        _channels[0].dir_gpio = DRV8701_DIR1_GPIO;
        _channels[1].dir_gpio = DRV8701_DIR2_GPIO;
    }
    // Add more instances if needed for additional boards
#else
    PX4_ERR("DRV8701 direction GPIOs not defined in board config");
    return false;
#endif

    // Configure direction GPIOs
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (_channels[i].dir_gpio != 0) {
            px4_arch_configgpio(_channels[i].dir_gpio);
            px4_arch_gpiowrite(_channels[i].dir_gpio, 0); // Default forward
        }
    }

    // Initialize PWM system if needed
    if (!_pwm_initialized) {
        int ret = up_pwm_servo_init(0xFFFF); // Initialize all channels
        if (ret < 0) {
            PX4_ERR("PWM init failed: %d", ret);
            return false;
        }

        // Set PWM frequency
        ret = up_pwm_servo_set_rate((unsigned)_param_pwm_freq.get());
        if (ret != OK) {
            PX4_WARN("Failed to set PWM rate to %.0f Hz", (double)_param_pwm_freq.get());
        }

        _pwm_initialized = true;
    }

    // Configure each channel
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (!configure_channel(i)) {
            PX4_WARN("Failed to configure channel %d", i);
        }
    }

    // Advertise status topic with multi-instance support
    hbridge_status_s status{};
    status.timestamp = hrt_absolute_time();
    status.instance = _instance_id;

    _status_pub = orb_advertise_multi(ORB_ID(hbridge_status), &status,
                                     &_status_instance);

    if (_status_pub == nullptr) {
        PX4_ERR("Failed to advertise hbridge_status");
        return false;
    }

    PX4_INFO("Advertising hbridge_status instance %d", _status_instance);

    // Increment running instances
    _running_instances.fetch_add(1);

    // Start periodic updates
    ScheduleOnInterval(SCHEDULE_INTERVAL);

    _is_running = true;

    PX4_INFO("HBridge initialized (instance %d, %d channels)", _instance_id, MAX_CHANNELS);
    return true;
}

bool HBridge::configure_channel(int channel)
{
    if (channel >= MAX_CHANNELS) {
        return false;
    }

    _channels[channel].pwm_channel = get_pwm_channel(channel);
    _channels[channel].pwm_mask = 1 << _channels[channel].pwm_channel;

    // Arm the channel
    up_pwm_servo_arm(true, _channels[channel].pwm_mask);

    // Set to neutral position
    uint16_t neutral = (uint16_t)((get_pwm_min(channel) + get_pwm_max(channel)) / 2);
    up_pwm_servo_set(_channels[channel].pwm_channel, neutral);

    _channels[channel].initialized = true;

    PX4_INFO("Channel %d: PWM ch=%d, mask=0x%04x, range=%.0f-%.0f us",
             channel, _channels[channel].pwm_channel, _channels[channel].pwm_mask,
             (double)get_pwm_min(channel), (double)get_pwm_max(channel));

    return true;
}

void HBridge::Run()
{
    if (should_exit()) {
        ScheduleClear();
        _is_running = false;
        return;
    }

    perf_begin(_loop_perf);

    // Check for parameter updates
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
        parameters_update();
    }

    // Process commands
    process_commands();

    // Publish status
    publish_status();

    perf_end(_loop_perf);
}

void HBridge::process_commands()
{
    perf_begin(_command_perf);

    hbridge_command_s cmd;

    // Check all command publishers using MultiSub
    for (unsigned i = 0; i < _command_sub.size(); i++) {
        while (_command_sub[i].update(&cmd)) {
            // Validate channel
            if (cmd.channel < MAX_CHANNELS) {
                if (cmd.enable) {
                    set_channel_speed(cmd.channel, cmd.duty_cycle);
                    _channels[cmd.channel].enabled = true;
                } else {
                    set_channel_speed(cmd.channel, 0.0f);
                    _channels[cmd.channel].enabled = false;
                }

                _command_count++;
                _last_command_time = cmd.timestamp;
            } else {
                PX4_WARN("Invalid channel %d in command", cmd.channel);
                _error_count++;
            }
        }
    }

    perf_end(_command_perf);
}

void HBridge::set_channel_speed(int channel, float duty_cycle)
{
    if (channel >= MAX_CHANNELS || !_channels[channel].initialized) {
        return;
    }

    // Clamp duty cycle
    duty_cycle = math::constrain(duty_cycle, -1.0f, 1.0f);

    // Update direction
    bool forward = duty_cycle >= 0.0f;
    update_channel_direction(channel, forward);

    // Convert to PWM value
    float abs_duty = fabsf(duty_cycle);
    uint16_t pwm_value;

    if (abs_duty < 0.001f) {
        // Dead zone - output neutral PWM
        pwm_value = (uint16_t)((get_pwm_min(channel) + get_pwm_max(channel)) / 2);
    } else {
        // Map duty cycle to PWM range
        float pwm_range = get_pwm_max(channel) - get_pwm_min(channel);
        pwm_value = (uint16_t)(get_pwm_min(channel) + (abs_duty * pwm_range));
    }

    // Set PWM value
    up_pwm_servo_set(_channels[channel].pwm_channel, pwm_value);

    _channels[channel].current_duty_cycle = duty_cycle;
}

void HBridge::update_channel_direction(int channel, bool forward)
{
    if (channel < MAX_CHANNELS && _channels[channel].dir_gpio != 0) {
        px4_arch_gpiowrite(_channels[channel].dir_gpio, forward ? 0 : 1);
    }
}

void HBridge::publish_status()
{
    if (_status_pub == nullptr) {
        return;
    }

    hbridge_status_s status{};
    status.timestamp = hrt_absolute_time();
    status.instance = _instance_id;

    for (int i = 0; i < MAX_CHANNELS; i++) {
        status.channel_enabled[i] = _channels[i].enabled;
        status.channel_duty_cycle[i] = _channels[i].current_duty_cycle;
        status.channel_current[i] = 0.0f; // TODO: Add current sensing
    }

    status.temperature = 0.0f; // TODO: Add temperature sensing
    status.fault_detected = false; // TODO: Add fault detection

    orb_publish(ORB_ID(hbridge_status), _status_pub, &status);
}

void HBridge::parameters_update()
{
    updateParams();

    // Update PWM frequency if changed
    if (_pwm_initialized) {
        up_pwm_servo_set_rate((unsigned)_param_pwm_freq.get());
    }

    // Reconfigure channels if PWM assignments changed
    for (int i = 0; i < MAX_CHANNELS; i++) {
        int new_pwm = get_pwm_channel(i);
        if (new_pwm != (int)_channels[i].pwm_channel && _channels[i].initialized) {
            // Disable old channel
            up_pwm_servo_set(_channels[i].pwm_channel, 0);
            up_pwm_servo_arm(false, _channels[i].pwm_mask);

            // Configure new channel
            configure_channel(i);
        }
    }
}

// Parameter getters
int HBridge::get_pwm_channel(int ch) const
{
    return (ch == 0) ? _param_ch0_pwm.get() : _param_ch1_pwm.get();
}

float HBridge::get_pwm_min(int ch) const
{
    return (ch == 0) ? _param_ch0_min.get() : _param_ch1_min.get();
}

float HBridge::get_pwm_max(int ch) const
{
    return (ch == 0) ? _param_ch0_max.get() : _param_ch1_max.get();
}

int HBridge::task_spawn(int argc, char *argv[])
{
    int instance_id = 0;
    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'i':
            instance_id = atoi(myoptarg);
            break;
        default:
            print_usage("unrecognized flag");
            return -1;
        }
    }

    if (instance_id < 0 || instance_id >= MAX_INSTANCES) {
        PX4_ERR("Instance must be between 0 and %d", MAX_INSTANCES - 1);
        return -1;
    }

    if (_objects[instance_id] != nullptr) {
        PX4_ERR("Instance %d already running", instance_id);
        return -1;
    }

    HBridge *instance = instantiate(instance_id);

    if (instance == nullptr) {
        PX4_ERR("Allocation failed");
        return -1;
    }

    // Store instance
    _objects[instance_id] = instance;

    // Store in ModuleBase for the main commands (use instance 0 as primary)
    if (instance_id == 0) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
    }

    if (!instance->init()) {
        delete instance;
        _objects[instance_id] = nullptr;
        if (instance_id == 0) {
            _object.store(nullptr);
            _task_id = -1;
        }
        return -1;
    }

    return 0;
}

HBridge *HBridge::instantiate(int instance)
{
    HBridge *obj = new HBridge(instance);
    return obj;
}

int HBridge::custom_command(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    const char *command = argv[1];

    if (!strcmp(command, "stop")) {
        // Parse instance ID
        int instance_id = 0;
        int myoptind = 2;
        int ch;
        const char *myoptarg = nullptr;

        while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
            switch (ch) {
            case 'i':
                instance_id = atoi(myoptarg);
                break;
            }
        }

        if (instance_id < 0 || instance_id >= MAX_INSTANCES) {
            PX4_ERR("Invalid instance ID");
            return 1;
        }

        if (_objects[instance_id] != nullptr) {
            _objects[instance_id]->request_stop();
            delete _objects[instance_id];
            _objects[instance_id] = nullptr;

            if (instance_id == 0) {
                _object.store(nullptr);
                _task_id = -1;
            }
            return 0;
        }

        PX4_WARN("Instance %d not running", instance_id);
        return 1;
    }

    if (!strcmp(command, "status")) {
        bool any_running = false;

        for (int i = 0; i < MAX_INSTANCES; i++) {
            if (_objects[i] != nullptr) {
                PX4_INFO("=== Instance %d ===", i);
                _objects[i]->print_status();
                any_running = true;
            }
        }

        if (!any_running) {
            PX4_INFO("No instances running");
        }

        return 0;
    }

    if (!strcmp(command, "test")) {
        // Test command for debugging - send test commands
        int instance_id = 0;
        int channel = 0;
        float duty_cycle = 0.0f;
        int myoptind = 2;
        int ch;
        const char *myoptarg = nullptr;

        while ((ch = px4_getopt(argc, argv, "i:c:d:", &myoptind, &myoptarg)) != EOF) {
            switch (ch) {
            case 'i':
                instance_id = atoi(myoptarg);
                break;
            case 'c':
                channel = atoi(myoptarg);
                break;
            case 'd':
                duty_cycle = atof(myoptarg);
                break;
            }
        }

        if (instance_id < 0 || instance_id >= MAX_INSTANCES) {
            PX4_ERR("Invalid instance ID");
            return 1;
        }

        if (channel < 0 || channel >= MAX_CHANNELS) {
            PX4_ERR("Invalid channel");
            return 1;
        }

        if (_objects[instance_id] != nullptr) {
            // Directly set the channel for testing
            _objects[instance_id]->set_channel_speed(channel, duty_cycle);
            PX4_INFO("Test: Set instance %d channel %d to %.2f duty cycle",
                     instance_id, channel, (double)duty_cycle);
            return 0;
        }

        PX4_WARN("Instance %d not running", instance_id);
        return 1;
    }

    return print_usage("unknown command");
}

int HBridge::print_status()
{
    PX4_INFO("HBridge Instance: %d", _instance_id);
    PX4_INFO("Running: %s", _is_running ? "yes" : "no");
    PX4_INFO("PWM Frequency: %.1f Hz", (double)_param_pwm_freq.get());
    PX4_INFO("Status topic instance: %d", _status_instance);
    PX4_INFO("Command count: %u", _command_count);
    PX4_INFO("Error count: %u", _error_count);

    if (_last_command_time > 0) {
        PX4_INFO("Last command: %llu us ago", hrt_absolute_time() - _last_command_time);
    }

    for (int i = 0; i < MAX_CHANNELS; i++) {
        PX4_INFO("Channel %d:", i);
        PX4_INFO("  PWM Channel: %d", _channels[i].pwm_channel);
        PX4_INFO("  PWM Range: %.0f - %.0f us",
                (double)get_pwm_min(i), (double)get_pwm_max(i));
        PX4_INFO("  Duty Cycle: %.2f", (double)_channels[i].current_duty_cycle);
        PX4_INFO("  Enabled: %s", _channels[i].enabled ? "Yes" : "No");
        PX4_INFO("  Direction GPIO: 0x%08lx", (unsigned long)_channels[i].dir_gpio);
        PX4_INFO("  Initialized: %s", _channels[i].initialized ? "Yes" : "No");
    }

    perf_print_counter(_loop_perf);
    perf_print_counter(_command_perf);

    return 0;
}

int HBridge::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
H-Bridge motor driver supporting multiple instances with dual channels.

Each instance controls 2 H-bridge channels with PWM speed control and GPIO direction control.
The PWM channels are configured via parameters.

### Implementation
The module subscribes to hbridge_command messages and filters by channel ID using MultiSub.
Each instance publishes status information with multi-instance support.

### Configuration
Configure each channel using the following parameters:
- HBRIDGE_CH0_PWM: PWM channel for channel 0
- HBRIDGE_CH1_PWM: PWM channel for channel 1
- HBRIDGE_PWM_FREQ: PWM frequency in Hz
- HBRIDGE_CH0_MIN/MAX: PWM range for channel 0
- HBRIDGE_CH1_MIN/MAX: PWM range for channel 1

### Examples
Start the driver (instance 0):
$ hbridge start

Start with specific instance:
$ hbridge start -i 1

Test channel control:
$ hbridge test -i 0 -c 0 -d 0.5

Check status of all instances:
$ hbridge status

Stop specific instance:
$ hbridge stop -i 1
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("hbridge", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
    PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, MAX_INSTANCES-1, "Instance ID", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test channel control");
    PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, MAX_INSTANCES-1, "Instance ID", true);
    PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, MAX_CHANNELS-1, "Channel", true);
    PRINT_MODULE_USAGE_PARAM_FLOAT('d', 0.0, -1.0, 1.0, "Duty cycle", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop specific instance");
    PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, MAX_INSTANCES-1, "Instance ID", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

void HBridge::usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
H-Bridge motor driver supporting multiple instances with dual channels.

Each instance controls 2 H-bridge channels with PWM speed control and GPIO direction control.
The PWM channels are configured via parameters.

### Implementation
The module subscribes to hbridge_command messages and filters by channel ID using MultiSub.
Each instance publishes status information with multi-instance support.

### Examples
Start the driver:
$ hbridge start

Start with specific instance:
$ hbridge start -i 1

Test channel control:
$ hbridge test -i 0 -c 0 -d 0.5

Check status:
$ hbridge status

Stop the driver:
$ hbridge stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("hbridge", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
    PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, MAX_INSTANCES-1, "Instance ID", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test channel control");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int hbridge_main(int argc, char *argv[])
{
    return HBridge::main(argc, argv);
}
