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

#include "hbridge.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_arch/io_timer.h>
#include <board_config.h>

// External declaration of board configuration
#ifdef BOARD_HAS_HBRIDGE_CONFIG
extern const hbridge_config_t g_hbridge_config[];
#endif

// Static storage for multiple instances
HBridge *HBridge::_instances[MAX_INSTANCES] = {};
px4::atomic<uint8_t> HBridge::_num_instances{0};
HBridge *HBridge::_manager_instance = nullptr;
bool HBridge::_pwm_initialized = false;

HBridge::HBridge(uint8_t instance) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_instance(instance),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_command_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": command"))
{
	// Initialize channel data
	for (int i = 0; i < MAX_CHANNELS; i++) {
		_channels[i] = {};
	}

	// Set manager instance if this is the manager
	if (_instance == MANAGER_INSTANCE) {
		_manager_instance = this;
	}
}

HBridge::~HBridge()
{
	// Disable all channels
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (_channels[i].initialized) {
			set_channel_speed(i, 0.0f);
		}
	}

	// Disable H-bridge if configured
	if (_board_config && _board_config->enable_gpio != 0) {
		px4_arch_gpiowrite(_board_config->enable_gpio, 0);
	}

	perf_free(_loop_perf);
	perf_free(_command_perf);
}

bool HBridge::init()
{
#ifdef BOARD_HAS_HBRIDGE_CONFIG
	// Get board configuration
	if (_instance < BOARD_NUM_HBRIDGES) {
		_board_config = &g_hbridge_config[_instance];
	} else {
		PX4_ERR("Instance %d exceeds board configuration", _instance);
		return false;
	}
#else
	PX4_ERR("No HBridge configuration for this board");
	return false;
#endif

	// Load parameters
	updateParams();

	// Configure GPIOs
	if (_board_config->left_dir_gpio != 0) {
		px4_arch_configgpio(_board_config->left_dir_gpio);
		px4_arch_gpiowrite(_board_config->left_dir_gpio, 0);
	}

	if (_board_config->right_dir_gpio != 0) {
		px4_arch_configgpio(_board_config->right_dir_gpio);
		px4_arch_gpiowrite(_board_config->right_dir_gpio, 0);
	}

	if (_board_config->enable_gpio != 0) {
		px4_arch_configgpio(_board_config->enable_gpio);
		px4_arch_gpiowrite(_board_config->enable_gpio, 0); // Disabled during init
	}

	// Initialize PWM system if needed
	if (!_pwm_initialized) {
		uint32_t required_channels = 0;
		required_channels |= (1 << _board_config->left_pwm_channel);
		required_channels |= (1 << _board_config->right_pwm_channel);

		int ret = up_motor_pwm_init(required_channels);
		if (ret < 0) {
			PX4_ERR("Motor PWM init failed: %d", ret);
			return false;
		}

		up_motor_pwm_set_rate(MOTOR_PWM_FREQ_25KHZ);
		up_motor_pwm_arm(true, required_channels);
		_pwm_initialized = true;
		PX4_INFO("Motor PWM initialized");
	}

	// Configure channels
	if (!configure_channels()) {
		PX4_ERR("Failed to configure channels");
		return false;
	}

	// Initialize status publication
	hbridge_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.instance = _instance;
	_status_pub.advertise(status);

	// Start periodic updates
	ScheduleOnInterval(SCHEDULE_INTERVAL);
	_is_running = true;

	// Enable H-bridge
	if (_board_config->enable_gpio != 0) {
		px4_arch_gpiowrite(_board_config->enable_gpio, 1);
	}

	PX4_INFO("HBridge instance %d (%s) initialized", _instance, _board_config->name);
	return true;
}

bool HBridge::configure_channels()
{
	// Set initial duty cycle to zero
	up_motor_pwm_set_duty_cycle(_board_config->left_pwm_channel, 0.0f);
	up_motor_pwm_set_duty_cycle(_board_config->right_pwm_channel, 0.0f);

	_channels[LEFT_CHANNEL].initialized = true;
	_channels[RIGHT_CHANNEL].initialized = true;

	PX4_INFO("Channels configured: Left PWM%d, Right PWM%d",
		_board_config->left_pwm_channel, _board_config->right_pwm_channel);

	return true;
}

void HBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_is_running = false;
		return;
	}

	// Manager instance doesn't need to run
	if (_instance == MANAGER_INSTANCE) {
		return;
	}

	perf_begin(_loop_perf);

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Process commands
	process_commands();

	// Process limit sensors
	process_limit_sensors();

	// Publish status
	publish_status();

	perf_end(_loop_perf);
}

void HBridge::process_commands()
{
	perf_begin(_command_perf);

	hbridge_command_s cmd;
	while (_command_sub.update(&cmd)) {
		// Check if command is for this instance
		if (cmd.instance != _instance) {
			continue;
		}

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
			PX4_WARN("Instance %d: Invalid channel %d", _instance, cmd.channel);
			_error_count++;
		}
	}

	perf_end(_command_perf);
}

void HBridge::process_limit_sensors()
{
	limit_sensor_s limit_msg;
	for (uint8_t instance = 0; instance < _limit_sensor_sub.size(); instance++) {
		while (_limit_sensor_sub[instance].update(&limit_msg)) {
			// Check if this limit sensor affects any channel
			for (int ch = 0; ch < MAX_CHANNELS; ch++) {
				// Check forward limit
				int fwd_limit_function = get_limit_sensor_function(ch, true);
				if (fwd_limit_function != 255 && limit_msg.function == fwd_limit_function) {
					_channels[ch].forward_limit_active = limit_msg.state;
					if (limit_msg.state && _channels[ch].current_duty_cycle > 0.0f) {
						set_channel_speed(ch, 0.0f);
					}
				}

				// Check reverse limit
				int rev_limit_function = get_limit_sensor_function(ch, false);
				if (rev_limit_function != 255 && limit_msg.function == rev_limit_function) {
					_channels[ch].reverse_limit_active = limit_msg.state;
					if (limit_msg.state && _channels[ch].current_duty_cycle < 0.0f) {
						set_channel_speed(ch, 0.0f);
					}
				}
			}
		}
	}
}

void HBridge::set_channel_speed(int channel, float duty_cycle)
{
	if (channel >= MAX_CHANNELS || !_channels[channel].initialized) {
		return;
	}

	// Clamp duty cycle
	duty_cycle = math::constrain(duty_cycle, -1.0f, 1.0f);

	// Check limit sensors
	bool forward = duty_cycle >= 0.0f;
	if (!check_limit_sensor_for_direction(channel, forward)) {
		duty_cycle = 0.0f;
	}

	// Update direction
	update_channel_direction(channel, forward);

	// Set PWM
	float abs_duty = fabsf(duty_cycle);
	if (abs_duty < 0.001f) {
		abs_duty = 0.0f;
	}

	int pwm_channel = (channel == LEFT_CHANNEL) ?
		_board_config->left_pwm_channel : _board_config->right_pwm_channel;

	up_motor_pwm_set_duty_cycle(pwm_channel, abs_duty);
	_channels[channel].current_duty_cycle = duty_cycle;
}

void HBridge::update_channel_direction(int channel, bool forward)
{
	uint32_t dir_gpio = (channel == LEFT_CHANNEL) ?
		_board_config->left_dir_gpio : _board_config->right_dir_gpio;

	if (dir_gpio != 0) {
		px4_arch_gpiowrite(dir_gpio, forward ? 1 : 0);
	}
}

bool HBridge::check_limit_sensor_for_direction(int channel, bool forward)
{
	if (channel >= MAX_CHANNELS) {
		return true;
	}

	if (forward && _channels[channel].forward_limit_active) {
		return false;
	}

	if (!forward && _channels[channel].reverse_limit_active) {
		return false;
	}

	return true;
}

int HBridge::get_limit_sensor_function(int ch, bool forward) const
{
	if (ch == LEFT_CHANNEL) {
		return forward ? _param_left_fwd_limit.get() : _param_left_rev_limit.get();
	} else if (ch == RIGHT_CHANNEL) {
		return forward ? _param_right_fwd_limit.get() : _param_right_rev_limit.get();
	}
	return 255;
}

void HBridge::publish_status()
{
	hbridge_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.instance = _instance;

	for (int i = 0; i < MAX_CHANNELS; i++) {
		status.channel_enabled[i] = _channels[i].enabled;
		status.channel_duty_cycle[i] = _channels[i].current_duty_cycle;
		status.channel_current[i] = 0.0f; // TODO: Add current sensing
	}

	status.temperature = 0.0f; // TODO: Add temperature sensing
	status.fault_detected = false;

	// Channel limit status
	status.channel_limit_state[0] = _channels[LEFT_CHANNEL].forward_limit_active;
	status.channel_limit_state[1] = _channels[LEFT_CHANNEL].reverse_limit_active;
	status.channel_limit_state[2] = _channels[RIGHT_CHANNEL].forward_limit_active;
	status.channel_limit_state[3] = _channels[RIGHT_CHANNEL].reverse_limit_active;

	status.channel_limits_available = (get_limit_sensor_function(LEFT_CHANNEL, true) != 255) ||
					  (get_limit_sensor_function(LEFT_CHANNEL, false) != 255) ||
					  (get_limit_sensor_function(RIGHT_CHANNEL, true) != 255) ||
					  (get_limit_sensor_function(RIGHT_CHANNEL, false) != 255);

	_status_pub.publish(status);
}

int HBridge::task_spawn(int argc, char *argv[])
{
	// Parse command line arguments
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int target_instance = -1;

	while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			target_instance = atoi(myoptarg);
			if (target_instance < 0 || target_instance >= MAX_INSTANCES) {
				PX4_ERR("Invalid instance %d, must be 0-%d", target_instance, MAX_INSTANCES - 1);
				return PX4_ERROR;
			}
			break;
		default:
			return print_usage("unknown option");
		}
	}

	// Create manager instance if needed
	if (_manager_instance == nullptr) {
		HBridge *manager = new HBridge(MANAGER_INSTANCE);
		if (manager == nullptr) {
			PX4_ERR("Failed to allocate HBridge manager");
			return PX4_ERROR;
		}

		_object.store(manager);
		_task_id = task_id_is_work_queue;
		PX4_INFO("HBridge manager started");
	}

#ifdef BOARD_HAS_HBRIDGE_CONFIG
	bool any_started = false;

	if (target_instance >= 0) {
		any_started = start_instance(target_instance);
	} else {
		for (int i = 0; i < BOARD_NUM_HBRIDGES; i++) {
			if (start_instance(i)) {
				any_started = true;
			}
		}
	}

	return any_started ? PX4_OK : PX4_ERROR;
#else
	PX4_ERR("No HBridge configured for this board");
	return PX4_ERROR;
#endif
}

bool HBridge::start_instance(int instance)
{
	if (_instances[instance] != nullptr) {
		PX4_INFO("HBridge instance %d already running", instance);
		return true;
	}

	HBridge *obj = new HBridge(instance);
	if (obj == nullptr) {
		PX4_ERR("Failed to allocate HBridge instance %d", instance);
		return false;
	}

	if (!obj->init()) {
		PX4_ERR("Failed to initialize HBridge instance %d", instance);
		delete obj;
		return false;
	}

	_instances[instance] = obj;
	_num_instances.fetch_add(1);
	obj->ScheduleNow();

	return true;
}

void HBridge::stop_all_instances()
{
	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (_instances[i] != nullptr) {
			delete _instances[i];
			_instances[i] = nullptr;
		}
	}
	_num_instances.store(0);
}

int HBridge::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "test")) {
		return test(argc, argv);
	}
	return print_usage("unknown command");
}

int HBridge::print_status()
{
	if (_instance == MANAGER_INSTANCE) {
		PX4_INFO("HBridge Manager Status");
		PX4_INFO("  Active instances: %d", _num_instances.load());
		for (int i = 0; i < MAX_INSTANCES; i++) {
			if (_instances[i] != nullptr) {
				_instances[i]->print_status();
			}
		}
		return 0;
	}

	PX4_INFO("HBridge Instance %d (%s):", _instance, _board_config ? _board_config->name : "unknown");
	PX4_INFO("  Running: %s", _is_running ? "yes" : "no");
	PX4_INFO("  Commands: %lu, Errors: %lu", _command_count, _error_count);

	for (int i = 0; i < MAX_CHANNELS; i++) {
		const char* channel_name = (i == LEFT_CHANNEL) ? "Left" : "Right";
		PX4_INFO("  %s: duty=%.2f%%, enabled=%s",
			channel_name,
			(double)(_channels[i].current_duty_cycle * 100.0f),
			_channels[i].enabled ? "yes" : "no");
	}

	return 0;
}

int HBridge::test(int argc, char *argv[])
{
	uint8_t instance = 0;
	int channel = 0;
	float duty_cycle = 0.0f;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:c:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			instance = (uint8_t)atoi(myoptarg);
			break;
		case 'c':
			channel = atoi(myoptarg);
			break;
		case 'd':
			duty_cycle = atof(myoptarg);
			break;
		default:
			return print_usage("unrecognized flag");
		}
	}

	if (instance >= MAX_INSTANCES || _instances[instance] == nullptr) {
		PX4_ERR("Instance %d not running", instance);
		return 1;
	}

	if (channel < 0 || channel >= MAX_CHANNELS) {
		PX4_ERR("Invalid channel %d", channel);
		return 1;
	}

	_instances[instance]->set_channel_speed(channel, duty_cycle);
	PX4_INFO("Instance %d: Set channel %d to %.2f", instance, channel, (double)duty_cycle);
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
Multi-instance H-Bridge motor driver.

Controls H-bridge channels with PWM speed and GPIO direction control.
Configuration is board-specific via hbridge_config.

### Examples
Start driver:
$ hbridge start

Test channel:
$ hbridge test -i 0 -c 0 -d 0.5

Check status:
$ hbridge status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hbridge", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_INT('i', -1, 0, MAX_INSTANCES-1, "Instance", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test channel");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, MAX_INSTANCES-1, "Instance", true);
	PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, MAX_CHANNELS-1, "Channel", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('d', 0.0, -1.0, 1.0, "Duty cycle", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int hbridge_main(int argc, char *argv[])
{
	return HBridge::main(argc, argv);
}
