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

#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_arch/io_timer.h>
#include <drivers/drv_motor_pwm.h>
#include <board_config.h>

bool HBridge::_pwm_initialized = false;

HBridge::HBridge() :
	ModuleBase<HBridge>(),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
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
#if defined(DRV8701_ENABLE_GPIO)
	// Disable H-bridge first for safety
	px4_arch_gpiowrite(DRV8701_ENABLE_GPIO, 0);
	PX4_INFO("H-Bridge disabled during shutdown");
#endif

	// Disable all channels
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (_channels[i].initialized && _channels[i].pwm_mask != 0) {
			up_motor_pwm_set_duty_cycle(_channels[i].pwm_channel, 0.0f);
		}
	}

	// Unadvertise status
	if (_status_pub != nullptr) {
		orb_unadvertise(_status_pub);
	}

	// Deinitialize PWM
	if (_pwm_initialized) {
		up_motor_pwm_deinit(0);
		_pwm_initialized = false;
	}

	perf_free(_loop_perf);
	perf_free(_command_perf);
}

bool HBridge::init()
{
	// Load initial parameters
	parameters_update();

	// Configure GPIOs based on board configuration
#if defined(DRV8701_RIGHT_DIR_GPIO) && defined(DRV8701_LEFT_DIR_GPIO)
	_channels[LEFT_CHANNEL].dir_gpio = DRV8701_LEFT_DIR_GPIO;   // PE14 for left wheel
	_channels[RIGHT_CHANNEL].dir_gpio = DRV8701_RIGHT_DIR_GPIO; // PE13 for right wheel
#else
	PX4_ERR("DRV8701 direction GPIOs not defined in board config");
	return false;
#endif

#if defined(DRV8701_ENABLE_GPIO)
	// Configure enable GPIO but keep it disabled during init
	px4_arch_configgpio(DRV8701_ENABLE_GPIO);
	px4_arch_gpiowrite(DRV8701_ENABLE_GPIO, 0); // Keep disabled during init
	PX4_INFO("H-Bridge enable GPIO configured (disabled during init)");

	// test
	// px4_arch_configgpio(GPIO_PA4);
	// px4_arch_gpiowrite(GPIO_PA4, 1);
	// px4_arch_configgpio(GPIO_PC0);
	// px4_arch_gpiowrite(GPIO_PC0, 0);
	// px4_arch_configgpio(GPIO_PC1);
	// px4_arch_gpiowrite(GPIO_PC1, 1);
	// px4_arch_configgpio(GPIO_PB2);
	// px4_arch_gpiowrite(GPIO_PB2, 1);
	// px4_arch_configgpio(GPIO_PB3);
	// px4_arch_gpiowrite(GPIO_PB3, 0);
	// px4_arch_configgpio(GPIO_PB4);
	// px4_arch_gpiowrite(GPIO_PB4, 1);

#endif

	// Configure direction GPIOs
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (_channels[i].dir_gpio != 0) {
			px4_arch_configgpio(_channels[i].dir_gpio);
			px4_arch_gpiowrite(_channels[i].dir_gpio, 0); // Default forward
			const char* channel_name = (i == LEFT_CHANNEL) ? "left" : "right";
			PX4_INFO("%s channel direction GPIO configured", channel_name);
		}
	}

	// Initialize PWM system if needed
	if (!_pwm_initialized) {
		// Calculate required PWM channel mask based on configured channels
		uint32_t required_channels = 0;
		for (int i = 0; i < MAX_CHANNELS; i++) {
			int pwm_ch = get_pwm_channel(i);
			if (pwm_ch >= 0 && pwm_ch < MOTOR_PWM_MAX_CHANNELS) {
				required_channels |= (1 << pwm_ch);
			}
		}

		if (required_channels == 0) {
			PX4_ERR("No valid PWM channels configured");
			return false;
		}

		PX4_INFO("Initializing motor PWM channels: 0x%04lx at 25 kHz", (unsigned long)required_channels);

		int ret = up_motor_pwm_init(required_channels);
		if (ret < 0) {
			PX4_ERR("Motor PWM init failed: %d (channels: 0x%04lx)", ret, (unsigned long)required_channels);
			return false;
		}

		// Set PWM frequency to 25 kHz
		up_motor_pwm_set_rate(MOTOR_PWM_FREQ_25KHZ);

		// Enable PWM outputs
		up_motor_pwm_arm(true, required_channels);

		_pwm_initialized = true;
		PX4_INFO("Motor PWM initialized at 25 kHz");
	}

	// Configure each channel
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (!configure_channel(i)) {
			PX4_WARN("Failed to configure channel %d", i);
		}
	}

	// Advertise status topic
	hbridge_status_s status{};
	status.timestamp = hrt_absolute_time();

	_status_pub = orb_advertise(ORB_ID(hbridge_status), &status);

	if (_status_pub == nullptr) {
		PX4_ERR("Failed to advertise hbridge_status");
		return false;
	}

	// Start periodic updates
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	_is_running = true;

#if defined(DRV8701_ENABLE_GPIO)
	// Now that everything is initialized, enable the H-bridge
	px4_arch_gpiowrite(DRV8701_ENABLE_GPIO, 1);
	PX4_INFO("H-Bridge enabled after successful initialization");
#endif

	PX4_INFO("HBridge initialized with %d channels", MAX_CHANNELS);
	return true;
}

bool HBridge::configure_channel(int channel)
{
	if (channel >= MAX_CHANNELS) {
		return false;
	}

	int pwm_ch = get_pwm_channel(channel);
	if (pwm_ch < 0 || pwm_ch >= 16) {
		PX4_ERR("Channel %d has invalid PWM channel: %d", channel, pwm_ch);
		return false;
	}

	_channels[channel].pwm_channel = pwm_ch;
	_channels[channel].pwm_mask = 1 << _channels[channel].pwm_channel;

	// Set to neutral position (0% duty cycle)
	up_motor_pwm_set_duty_cycle(_channels[channel].pwm_channel, 0.0f);

	_channels[channel].initialized = true;

	const char* channel_name = (channel == LEFT_CHANNEL) ? "left" : "right";
	PX4_INFO("%s channel: PWM ch=%d, mask=0x%04lx",
		 channel_name, _channels[channel].pwm_channel, (unsigned long)_channels[channel].pwm_mask);

	return true;
}

void HBridge::Run()
{
	if (should_exit()) {
#if defined(DRV8701_ENABLE_GPIO)
		// Disable H-bridge when exiting
		px4_arch_gpiowrite(DRV8701_ENABLE_GPIO, 0);
		PX4_INFO("H-Bridge disabled on exit");
#endif
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

	// Process all pending commands
	while (_command_sub.update(&cmd)) {
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

	perf_end(_command_perf);
}

void HBridge::process_limit_sensors()
{
	limit_sensor_s limit_msg;

	// Process all limit sensor instances
	for (uint8_t instance = 0; instance < _limit_sensor_sub.size(); instance++) {
		// Process all pending limit sensor messages for this instance
		while (_limit_sensor_sub[instance].update(&limit_msg)) {
			// Check if this limit sensor affects any channel
			for (int ch = 0; ch < MAX_CHANNELS; ch++) {
				// Check forward limit
				int fwd_limit_function = get_limit_sensor_function(ch, true);
				if (fwd_limit_function != 255 && limit_msg.function == fwd_limit_function) {
					bool was_active = _channels[ch].forward_limit_active;
					_channels[ch].forward_limit_active = limit_msg.state;

					// If limit just became active and currently moving forward, stop the channel
					if (limit_msg.state && !was_active && _channels[ch].current_duty_cycle > 0.0f) {
						set_channel_speed(ch, 0.0f);
						const char* channel_name = (ch == LEFT_CHANNEL) ? "left" : "right";
						PX4_INFO("%s channel stopped due to forward limit sensor", channel_name);
					}
				}

				// Check reverse limit
				int rev_limit_function = get_limit_sensor_function(ch, false);
				if (rev_limit_function != 255 && limit_msg.function == rev_limit_function) {
					bool was_active = _channels[ch].reverse_limit_active;
					_channels[ch].reverse_limit_active = limit_msg.state;

					// If limit just became active and currently moving reverse, stop the channel
					if (limit_msg.state && !was_active && _channels[ch].current_duty_cycle < 0.0f) {
						set_channel_speed(ch, 0.0f);
						const char* channel_name = (ch == LEFT_CHANNEL) ? "left" : "right";
						PX4_INFO("%s channel stopped due to reverse limit sensor", channel_name);
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

	// Check limit sensors before allowing movement
	bool forward = duty_cycle >= 0.0f;
	if (!check_limit_sensor_for_direction(channel, forward)) {
		// Limit sensor is active for this direction, set duty to zero
		duty_cycle = 0.0f;
		const char* channel_name = (channel == LEFT_CHANNEL) ? "left" : "right";
		const char* direction = forward ? "forward" : "reverse";
		PX4_DEBUG("%s channel %s movement blocked by limit sensor", channel_name, direction);
	}

	// Update direction based on sign
	update_channel_direction(channel, forward);

	// Set PWM duty cycle (absolute value)
	float abs_duty = fabsf(duty_cycle);

	// Apply dead zone
	if (abs_duty < 0.001f) {
		abs_duty = 0.0f;
	}

	// Set motor PWM duty cycle directly
	int ret = up_motor_pwm_set_duty_cycle(_channels[channel].pwm_channel, abs_duty);
	if (ret != 0) {
		PX4_WARN("Failed to set PWM duty cycle for channel %d: %d", channel, ret);
	}

	_channels[channel].current_duty_cycle = duty_cycle;

	const char* channel_name = (channel == LEFT_CHANNEL) ? "left" : "right";
	PX4_DEBUG("%s channel: duty=%.2f%%, dir=%s",
		  channel_name, (double)(abs_duty * 100.0f), forward ? "FWD" : "REV");
}

void HBridge::update_channel_direction(int channel, bool forward)
{
	if (channel < MAX_CHANNELS && _channels[channel].dir_gpio != 0) {
		// Calculate final direction signal using cached dir_reversed flag
		bool gpio_state = forward;
		if (_channels[channel].dir_reversed) {
			gpio_state = !gpio_state;
		}

		px4_arch_gpiowrite(_channels[channel].dir_gpio, gpio_state ? 1 : 0);
	}
}

bool HBridge::check_limit_sensor_for_direction(int channel, bool forward)
{
	if (channel >= MAX_CHANNELS) {
		return true; // Allow movement for invalid channels
	}

	// Check if the limit sensor for this direction is active
	if (forward && _channels[channel].forward_limit_active) {
		return false; // Block forward movement
	}

	if (!forward && _channels[channel].reverse_limit_active) {
		return false; // Block reverse movement
	}

	return true; // Allow movement
}

void HBridge::publish_status()
{
	if (_status_pub == nullptr) {
		return;
	}

	hbridge_status_s status{};
	status.timestamp = hrt_absolute_time();

	for (int i = 0; i < MAX_CHANNELS; i++) {
		status.channel_enabled[i] = _channels[i].enabled;
		status.channel_duty_cycle[i] = _channels[i].current_duty_cycle;
		status.channel_current[i] = 0.0f; // TODO: Add current sensing
	}

	status.temperature = 0.0f; // TODO: Add temperature sensing
	status.fault_detected = false; // TODO: Add fault detection

	// Add limit sensor information
	status.fault_detected = _channels[LEFT_CHANNEL].forward_limit_active ||
				_channels[LEFT_CHANNEL].reverse_limit_active ||
				_channels[RIGHT_CHANNEL].forward_limit_active ||
				_channels[RIGHT_CHANNEL].reverse_limit_active;

	orb_publish(ORB_ID(hbridge_status), _status_pub, &status);
}

void HBridge::parameters_update()
{
	updateParams();

	// Update direction reverse settings for each channel
	_channels[LEFT_CHANNEL].dir_reversed = (_param_left_dir_rev.get() != 0);
	_channels[RIGHT_CHANNEL].dir_reversed = (_param_right_dir_rev.get() != 0);

	// Update PWM frequency if changed
	if (_pwm_initialized) {
		up_motor_pwm_set_rate((unsigned)_param_pwm_freq.get());
	}

	// Reconfigure channels if PWM assignments changed
	for (int i = 0; i < MAX_CHANNELS; i++) {
		int new_pwm = get_pwm_channel(i);
		if (new_pwm != (int)_channels[i].pwm_channel && _channels[i].initialized) {
			// Disable old channel
			up_motor_pwm_set_duty_cycle(_channels[i].pwm_channel, 0.0f);

			// Configure new channel
			configure_channel(i);
		}
	}
}

// Parameter getters
int HBridge::get_pwm_channel(int ch) const
{
	// ch 0 = left channel, ch 1 = right channel
	return (ch == LEFT_CHANNEL) ? _param_left_pwm.get() : _param_right_pwm.get();
}

int HBridge::get_limit_sensor_function(int ch, bool forward) const
{
	// Return the limit sensor function ID for the given channel and direction
	if (ch == LEFT_CHANNEL) {
		return forward ? _param_left_fwd_limit.get() : _param_left_rev_limit.get();
	} else if (ch == RIGHT_CHANNEL) {
		return forward ? _param_right_fwd_limit.get() : _param_right_rev_limit.get();
	}
	return 255; // Disabled
}

int HBridge::task_spawn(int argc, char *argv[])
{
	HBridge *instance = new HBridge();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		instance->ScheduleNow();
		return 0;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return -1;
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
	PX4_INFO("HBridge Status:");
	PX4_INFO("Running: %s", _is_running ? "yes" : "no");
	PX4_INFO("PWM Frequency: %.1f Hz", (double)_param_pwm_freq.get());
	PX4_INFO("Command count: %lu", _command_count);
	PX4_INFO("Error count: %lu", _error_count);

	if (_last_command_time > 0) {
		PX4_INFO("Last command: %llu us ago", hrt_absolute_time() - _last_command_time);
	}

	for (int i = 0; i < MAX_CHANNELS; i++) {
		const char* channel_name = (i == LEFT_CHANNEL) ? "Left" : "Right";
		PX4_INFO("%s Channel:", channel_name);
		PX4_INFO("  PWM Channel: %d", _channels[i].pwm_channel);
		PX4_INFO("  Duty Cycle: %.2f", (double)_channels[i].current_duty_cycle);
		PX4_INFO("  Enabled: %s", _channels[i].enabled ? "Yes" : "No");
		PX4_INFO("  Direction GPIO: 0x%08lx", (unsigned long)_channels[i].dir_gpio);
		PX4_INFO("  Direction Reversed: %s", _channels[i].dir_reversed ? "Yes" : "No");
		PX4_INFO("  Initialized: %s", _channels[i].initialized ? "Yes" : "No");
		PX4_INFO("  Forward Limit: %s (Function: %d)",
			 _channels[i].forward_limit_active ? "ACTIVE" : "OK",
			 get_limit_sensor_function(i, true));
		PX4_INFO("  Reverse Limit: %s (Function: %d)",
			 _channels[i].reverse_limit_active ? "ACTIVE" : "OK",
			 get_limit_sensor_function(i, false));
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
H-Bridge motor driver with dual channels (left and right).

Controls 2 H-bridge channels with PWM speed control and GPIO direction control.
The PWM channels are configured via parameters.

### Implementation
The module subscribes to hbridge_command messages and controls the specified channel.
Status information is published to hbridge_status topic.

### Configuration
Configure each channel using the following parameters:
- HBRIDGE_L_PWM: PWM channel for left channel (default: 0)
- HBRIDGE_R_PWM: PWM channel for right channel (default: 1)
- HBRIDGE_PWM_FREQ: PWM frequency in Hz (default: 25000 Hz)
- HBRIDGE_L_DREV: Left channel direction reverse (default: 0=normal)
- HBRIDGE_R_DREV: Right channel direction reverse (default: 0=normal)
- HBRIDGE_L_FLIM: Left channel forward limit sensor function (default: 255=disabled)
- HBRIDGE_L_RLIM: Left channel reverse limit sensor function (default: 255=disabled)
- HBRIDGE_R_FLIM: Right channel forward limit sensor function (default: 255=disabled)
- HBRIDGE_R_RLIM: Right channel reverse limit sensor function (default: 255=disabled)

Motor control uses duty cycle (0.0 to 1.0) for speed control.
Direction is controlled via separate GPIO pins.
Direction reverse parameters allow inverting the direction signal if needed.
Limit sensors will automatically stop movement when activated for the configured direction.

### Examples
Start the driver:
$ hbridge start

Test channel control:
$ hbridge test -c 0 -d 0.5   # Left channel forward at 50%
$ hbridge test -c 1 -d -0.3  # Right channel reverse at 30%

Check status:
$ hbridge status

Stop the driver:
$ hbridge stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hbridge", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test channel control");
	PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, MAX_CHANNELS-1, "Channel (0=left, 1=right)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('d', 0.0, -1.0, 1.0, "Duty cycle", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// Test command implementation
int HBridge::test(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_ERR("Driver not running");
		return 1;
	}

	int channel = 0;
	float duty_cycle = 0.0f;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "c:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
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

	if (channel < 0 || channel >= MAX_CHANNELS) {
		PX4_ERR("Invalid channel %d (valid: 0=left, 1=right)", channel);
		return 1;
	}

	// Get the running instance
	HBridge *inst = _object.load();
	if (inst != nullptr) {
		const char* channel_name = (channel == LEFT_CHANNEL) ? "left" : "right";
		inst->set_channel_speed(channel, duty_cycle);
		PX4_INFO("Set %s channel to %.2f duty cycle", channel_name, (double)duty_cycle);
		return 0;
	}

	return 1;
}

extern "C" __EXPORT int hbridge_main(int argc, char *argv[])
{
	return HBridge::main(argc, argv);
}
