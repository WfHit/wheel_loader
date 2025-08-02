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

#include "wheel_controller.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

WheelController::WheelController() :
	ModuleBase(),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_speed_controller(),
	_speed_filter(CONTROL_DT, DEFAULT_FILTER_FREQ)
{
}

WheelController::~WheelController()
{
	perf_free(_loop_perf);
	perf_free(_control_perf);
}

bool WheelController::init()
{
	// Update parameters first to get instance information
	updateParams();

	// Initialize instance-specific subscriptions based on parameters
	uint8_t encoder_instance = static_cast<uint8_t>(_param_encoder_id.get());
	uint8_t motor_channel = static_cast<uint8_t>(_param_motor_channel.get());

	// Initialize encoder subscription with specific instance
	_encoder_sub = uORB::Subscription{ORB_ID(sensor_quad_encoder), encoder_instance};

	if (!_encoder_sub.subscribe()) {
		PX4_ERR("Failed to subscribe to sensor_quad_encoder instance %d", encoder_instance);
		return false;
	}

	// Initialize H-bridge status subscription with specific instance
	_hbridge_status_sub = uORB::Subscription{ORB_ID(hbridge_status), motor_channel};

	if (!_hbridge_status_sub.subscribe()) {
		PX4_ERR("Failed to subscribe to hbridge_status instance %d", motor_channel);
		return false;
	}

	// Initialize H-bridge command publication with specific instance
	_motor_cmd_pub = uORB::PublicationMulti<hbridge_command_s>{ORB_ID(hbridge_command), motor_channel};

	// Initialize other subscriptions
	if (!_setpoint_sub.subscribe()) {
		PX4_ERR("Failed to subscribe to wheel_loader_setpoint");
		return false;
	}

	if (!_param_update_sub.subscribe()) {
		PX4_ERR("Failed to subscribe to parameter_update");
		return false;
	}

	// Initialize performance counters
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	_control_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": control");

	// Configure PID controller with initial parameters
	_speed_controller.setGains(_param_speed_p.get(),
				   _param_speed_i.get(),
				   _param_speed_d.get());
	_speed_controller.setIntegralLimit(_param_integrator_max.get());
	_speed_controller.setOutputLimit(MAX_PWM_VALUE);

	// Initialize speed filter
	_speed_filter.set_cutoff_frequency(CONTROL_DT, _param_filter_freq.get());

	// Mark as initialized
	_state.initialized = true;

	// Start the work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	PX4_INFO("Wheel controller initialized - Encoder: %d, Motor: %d, Front: %s",
		 encoder_instance, motor_channel,
		 (_param_is_front_wheel.get() == 1) ? "YES" : "NO");

	return true;
}

void WheelController::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	perf_begin(_loop_perf);

	// Check for parameter updates
	if (_param_update_sub.updated()) {
		parameter_update_s param_update;
		_param_update_sub.copy(&param_update);
		parameters_update();
	}

	// Update sensor data
	update_encoder_feedback();
	update_hbridge_status();

	// Safety checks first
	check_safety_conditions();

	if (!_state.emergency_stop) {
		// Update setpoint and run control
		if (update_speed_setpoint()) {
			perf_begin(_control_perf);
			run_speed_controller();
			perf_end(_control_perf);
		} else {
			// No valid setpoint - stop motor
			_state.setpoint_rad_s = 0.0f;
			_state.pwm_output = 0.0f;
		}

	} else {
		// Emergency stop active
		_state.setpoint_rad_s = 0.0f;
		_state.pwm_output = 0.0f;
	}

	// Publish motor command
	publish_motor_command();

	perf_end(_loop_perf);
}

bool WheelController::update_speed_setpoint()
{
	wheel_loader_setpoint_s setpoint;

	if (_setpoint_sub.update(&setpoint)) {
		// Check if setpoint is for this wheel instance
		bool is_for_this_wheel = (_param_is_front_wheel.get() == 1) ?
					 setpoint.front_wheel_active :
					 setpoint.rear_wheel_active;

		if (is_for_this_wheel) {
			float target_speed = (_param_is_front_wheel.get() == 1) ?
					     setpoint.front_wheel_speed_rad_s :
					     setpoint.rear_wheel_speed_rad_s;

			// Limit speed to maximum
			_state.setpoint_rad_s = math::constrain(target_speed,
								-_param_max_speed.get(),
								_param_max_speed.get());
			_state.last_setpoint_us = hrt_absolute_time();
			return true;
		}
	}

	return is_setpoint_valid();
}

void WheelController::update_encoder_feedback()
{
	sensor_quad_encoder_s encoder;

	if (_encoder_sub.update(&encoder)) {
		// Check if encoder data is from correct instance
		if (encoder.device_id == (uint32_t)_param_encoder_id.get()) {
			// Convert encoder data to rad/s (implementation depends on encoder specifics)
			float raw_speed = encoder.speed; // Assuming speed is already in rad/s

			// Apply low-pass filtering
			_state.speed_rad_s = _speed_filter.apply(raw_speed);
			_state.last_encoder_us = encoder.timestamp;
		}
	}
}

void WheelController::run_speed_controller()
{
	// PID control
	float speed_error = _state.setpoint_rad_s - _state.speed_rad_s;
	float pid_output = _speed_controller.update(speed_error, CONTROL_DT);

	// Apply output constraints
	_state.pwm_output = constrain_pwm(pid_output);
}

void WheelController::publish_motor_command()
{
	hbridge_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.channel = static_cast<uint8_t>(_param_motor_channel.get());
	cmd.duty_cycle = _state.pwm_output;
	cmd.enabled = _state.motor_enabled && !_state.emergency_stop;

	_motor_cmd_pub.publish(cmd);
}

void WheelController::update_hbridge_status()
{
	hbridge_status_s status;

	if (_hbridge_status_sub.update(&status)) {
		if (status.channel == static_cast<uint8_t>(_param_motor_channel.get())) {
			_state.motor_enabled = status.enabled;

			// Check for faults
			if (status.fault) {
				PX4_WARN("Motor channel %d fault detected", _param_motor_channel.get());
				_state.emergency_stop = true;
			}
		}
	}
}

void WheelController::check_safety_conditions()
{
	const uint64_t now = hrt_absolute_time();

	// Check setpoint timeout
	if (!isSetpointValid()) {
		if (!_state.emergency_stop) {
			PX4_WARN("Setpoint timeout");
		}

		_state.emergency_stop = true;

	} else {
		_state.emergency_stop = false;
	}

	// Check encoder timeout
	if (now - _state.last_encoder_us > ENCODER_TIMEOUT_US) {
		PX4_WARN("Encoder timeout");
		_state.emergency_stop = true;
	}
}

void WheelController::parameters_update()
{
	updateParams();

	// Update PID gains
	_speed_controller.setGains(_param_speed_p.get(),
				   _param_speed_i.get(),
				   _param_speed_d.get());

	// Update filter frequency
	_speed_filter.set_cutoff_frequency(CONTROL_DT, _param_filter_freq.get());
}

float WheelController::constrain_pwm(float value) const
{
	return math::constrain(value, MIN_PWM_VALUE, MAX_PWM_VALUE);
}

bool WheelController::is_setpoint_valid() const
{
	const uint64_t timeout_us = static_cast<uint64_t>(_param_setpoint_timeout.get() * 1e6f);
	return (hrt_absolute_time() - _state.last_setpoint_us) < timeout_us;
}

int WheelController::print_status()
{
	PX4_INFO("=== Wheel Controller Status ===");
	PX4_INFO("Speed: %.2f rad/s (target: %.2f)",
		 (double)_state.speed_rad_s, (double)_state.setpoint_rad_s);
	PX4_INFO("PWM: %.3f", (double)_state.pwm_output);
	PX4_INFO("Motor enabled: %s", _state.motor_enabled ? "YES" : "NO");
	PX4_INFO("Emergency stop: %s", _state.emergency_stop ? "YES" : "NO");
	PX4_INFO("Initialized: %s", _state.initialized ? "YES" : "NO");

	perf_print_counter(_loop_perf);
	perf_print_counter(_control_perf);

	return 0;
}

int WheelController::task_spawn(int argc, char *argv[])
{
	WheelController *instance = new WheelController();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	if (!instance->init()) {
		delete instance;
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	return PX4_OK;
}

int WheelController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Wheel controller for articulated wheel loader.

Implements closed-loop speed control using quadrature encoder feedback
and DRV8701 H-bridge motor driver interface.

### Features
- PID speed control with configurable gains
- Low-pass filtering for noise reduction
- Safety monitoring and emergency stop
- Instance-based multi-wheel support
- Performance monitoring

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wheel_controller", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int WheelController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}
