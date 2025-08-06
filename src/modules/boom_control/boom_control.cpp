/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice	_trajectory_generator.setMaxJerkZ(_param_boom_max_jerk.get());

	// Use PositionSmoothing for trajectory generation (1D motion using Z-axis)
	matrix::Vector3f current_pos{0.0f, 0.0f, _current_actuator_length / 1000.0f}; // Convert to meters
	matrix::Vector3f target_pos{0.0f, 0.0f, _target_actuator_length / 1000.0f};
	matrix::Vector3f feedforward_velocity{0.0f, 0.0f, 0.0f};

	float dt = 1.0f / _param_update_rate.get();
	PositionSmoothing::PositionSmoothingSetpoints setpoints;
	_trajectory_generator.generateSetpoints(current_pos, target_pos, feedforward_velocity,
	                                       dt, false, setpoints);

	// Store the generated setpoints for use in position control
	_desired_position_m = setpoints.position(2);
	_desired_velocity_m_s = setpoints.velocity(2);
}

void BoomControl::run_position_control()
{
	if (_state == BoomState::ERROR || !_sensor_valid) {
		_motor_output = 0.0f;
		return;
	}

	// Use the setpoints from trajectory generator (already in meters)
	float desired_position_m = _desired_position_m;
	float desired_velocity_m_s = _desired_velocity_m_s;

	// Convert back to mm for control
	float desired_position_mm = desired_position_m * 1000.0f;

	// Position error in mm
	float position_error = desired_position_mm - _current_actuator_length;

	// Check if we've reached the target
	if (fabsf(position_error) < 5.0f && fabsf(desired_velocity_m_s) < 0.001f) { // 5mm tolerance
		_state = BoomState::HOLDING;
	}

	// PID control
	float dt = 1.0f / _param_update_rate.get();
	_motor_output = _position_pid.update(position_error, dt);

	// Add velocity feedforward
	float velocity_ff = desired_velocity_m_s / (_param_boom_max_vel.get() / 1000.0f);
	_motor_output += velocity_ff * 0.3f; // Feedforward gain

	// Apply deadzone compensation
	if (fabsf(_motor_output) > _param_motor_deadzone.get()) {
		if (_motor_output > 0.0f) {
			_motor_output += _param_motor_deadzone.get();
		} else {
			_motor_output -= _param_motor_deadzone.get();
		}
	}
}f conditions and the following disclaimer.
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

#include "boom_control.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

BoomControl::BoomControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

BoomControl::~BoomControl()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

bool BoomControl::init()
{
	// Update parameters and initialize kinematics
	update_parameters();
	update_kinematics_from_params();

	// Initialize trajectory generator
	_trajectory_generator.setMaxVelocityZ(_param_boom_max_vel.get() / 1000.0f); // Convert mm/s to m/s
	_trajectory_generator.setMaxAccelerationZ(_param_boom_max_acc.get() / 1000.0f);
	_trajectory_generator.setMaxJerkZ(_param_boom_max_jerk.get() / 1000.0f);

	// Set initial target to current position
	_target_boom_angle = _param_boom_pos_carry.get();
	_target_actuator_length = boom_angle_to_actuator_length(_target_boom_angle);

	// Start scheduled execution
	int update_rate_hz = _param_update_rate.get();
	ScheduleOnInterval(1_s / update_rate_hz);

	return true;
}

bool BoomControl::check_hbridge_status()
{
	// Check hbridge status for our motor instance using EKF2-style SubscriptionMultiArray
	uint8_t hbridge_channel = static_cast<uint8_t>(_param_hbridge_channel.get());
	hbridge_status_s hbridge_status;

	if (hbridge_channel < _hbridge_status_sub.size()) {
		if (_hbridge_status_sub[hbridge_channel].update(&hbridge_status)) {
			// Verify this is the correct instance
			if (hbridge_status.instance == hbridge_channel) {
				// Update our status based on hbridge feedback
				// Could use this for fault detection, current monitoring, etc.
				return hbridge_status.enabled;
			}
		}
	}
	return false;  // No status received or not enabled
}

void BoomControl::updateHBridgeStatus()
{
	hbridge_status_s hbridge_status;

	// If no specific instance selected, find our motor's instance
	if (_hbridge_status_selected < 0) {
		const hrt_abstime timestamp_stale = math::max(hrt_absolute_time(), 100_ms) - 100_ms;
		uint8_t target_hbridge_channel = static_cast<uint8_t>(_param_hbridge_channel.get());

		if (_hbridge_status_sub.advertised()) {
			for (unsigned i = 0; i < _hbridge_status_sub.size(); i++) {
				if (_hbridge_status_sub[i].update(&hbridge_status)) {
					// Check if this is our motor's status
					if ((hbridge_status.timestamp != 0) &&
					    (hbridge_status.timestamp > timestamp_stale) &&
					    (hbridge_status.instance == target_hbridge_channel)) {

						int nstatus = orb_group_count(ORB_ID(hbridge_status));
						if (nstatus > 1) {
							PX4_INFO("Boom control selected hbridge_status:%d (channel %d, %d advertised)",
							         i, target_hbridge_channel, nstatus);
						}

						_hbridge_status_selected = i;
						_last_hbridge_status_update = hbridge_status.timestamp;
						break;
					}
				}
			}
		}
	}

	// Use the selected instance
	if (_hbridge_status_selected >= 0 &&
	    _hbridge_status_sub[_hbridge_status_selected].update(&hbridge_status)) {

		if (hbridge_status.instance == static_cast<uint8_t>(_param_hbridge_channel.get())) {
			// Process our motor's status
			_last_hbridge_status_update = hbridge_status.timestamp;

			// Example: check if motor is disabled (potential fault condition)
			if (!hbridge_status.enabled) {
				PX4_ERR("HBridge channel %" PRId32 " is disabled", _param_hbridge_channel.get());
				// Handle disabled state...
			}

			// Update internal state if needed
			// _motor_current = hbridge_status.current;
			// _motor_voltage = hbridge_status.voltage;
		}
	}
}

void BoomControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// Check for parameter updates
	parameter_update_s param_update;
	if (_parameter_update_sub.update(&param_update)) {
		update_parameters();
		update_kinematics_from_params();
	}

	// Update sensor data from AS5600
	update_sensor_data();

	// Update HBridge status using EKF2-style instance discovery
	updateHBridgeStatus();

	// Handle auto-calibration if active
	if (_calibration_mode) {
		update_calibration();
	}

	// Process boom command inputs
	process_boom_command();

	// Update trajectory planning
	update_trajectory();

	// Run position control
	run_position_control();

	// Publish H-bridge channel command
	publish_hbridge_command();

	// Publish boom status
	publish_boom_status();

	perf_end(_cycle_perf);
}

void BoomControl::update_parameters()
{
	updateParams();

	// Update PID gains
	_position_pid.setGains(_param_boom_p.get(), _param_boom_i.get(), _param_boom_d.get());

	// Update trajectory generator limits
	_trajectory_generator.setMaxVelocityZ(_param_boom_max_vel.get() / 1000.0f);
	_trajectory_generator.setMaxAccelerationZ(_param_boom_max_acc.get() / 1000.0f);
	_trajectory_generator.setMaxJerkZ(_param_boom_max_jerk.get() / 1000.0f);
}

void BoomControl::update_kinematics_from_params()
{
	_kinematics.pivot_to_actuator_base = _param_kin_pivot_to_base.get();
	_kinematics.pivot_to_actuator_attach = _param_kin_pivot_to_attach.get();
	_kinematics.actuator_base_angle = math::radians(_param_kin_base_angle.get());
}

void BoomControl::update_sensor_data()
{
	sensor_mag_encoder_s mag_encoder_data;
	if (_mag_encoder_sub.update(&mag_encoder_data) && mag_encoder_data.device_id == static_cast<uint32_t>(_param_mag_encoder_instance_id.get())) {
		_last_sensor_update = hrt_absolute_time();
		_sensor_valid = (mag_encoder_data.magnet_detected == 1) &&
		                (mag_encoder_data.magnet_too_strong == 0) &&
		                (mag_encoder_data.magnet_too_weak == 0);

		// Magnetic encoder provides angle directly in radians
		float raw_angle_deg = math::degrees(mag_encoder_data.angle);

		// Apply calibration
		float calibrated_angle = raw_angle_deg * _param_mag_encoder_scale.get() + _param_mag_encoder_offset.get();

		// Apply angle reverse if enabled
		if (_param_angle_reverse.get()) {
			calibrated_angle = -calibrated_angle;
		}

		// Ensure angle is in 0-360 range
		while (calibrated_angle < 0.0f) {
			calibrated_angle += 360.0f;
		}
		while (calibrated_angle >= 360.0f) {
			calibrated_angle -= 360.0f;
		}

		// Convert AS5600 angle to actuator length
		_current_actuator_length = as5600_angle_to_actuator_length(calibrated_angle);

		// Convert actuator length to boom angle
		_current_boom_angle = actuator_length_to_boom_angle(_current_actuator_length);
	} else if (is_sensor_timeout()) {
		_sensor_valid = false;
		emergency_stop();
	}
}

float BoomControl::as5600_angle_to_actuator_length(float as5600_angle)
{
	// AS5600 measures the angle at the actuator pivot point
	// Use law of cosines to find actuator length

	float theta = math::radians(as5600_angle);
	float a = _kinematics.pivot_to_actuator_base;
	float b = _kinematics.pivot_to_actuator_attach;
	float gamma = _kinematics.actuator_base_angle + theta;

	// Law of cosines: c² = a² + b² - 2ab*cos(gamma)
	float length_squared = a * a + b * b - 2.0f * a * b * cosf(gamma);

	// Ensure we don't take square root of negative number
	if (length_squared < 0.0f) {
		length_squared = 0.0f;
	}

	return sqrtf(length_squared);
}

float BoomControl::boom_angle_to_actuator_length(float boom_angle)
{
	// Inverse kinematics: Given boom angle, find required actuator length

	float theta = math::radians(boom_angle);
	float a = _kinematics.pivot_to_actuator_base;
	float b = _kinematics.pivot_to_actuator_attach;

	// Calculate the angle between pivot-to-base and pivot-to-attach
	float gamma = _kinematics.actuator_base_angle - theta;

	// Use law of cosines to find actuator length
	float length_squared = a * a + b * b - 2.0f * a * b * cosf(gamma);

	if (length_squared < 0.0f) {
		length_squared = 0.0f;
	}

	return sqrtf(length_squared);
}

float BoomControl::actuator_length_to_boom_angle(float actuator_length)
{
	// Forward kinematics: Given actuator length, find boom angle

	float a = _kinematics.pivot_to_actuator_base;
	float b = _kinematics.pivot_to_actuator_attach;
	float c = actuator_length;

	// Law of cosines to find angle
	float cos_gamma = (a * a + b * b - c * c) / (2.0f * a * b);
	cos_gamma = math::constrain(cos_gamma, -1.0f, 1.0f); // Ensure valid range

	float gamma = acosf(cos_gamma);
	float boom_angle = math::degrees(_kinematics.actuator_base_angle - gamma);

	return boom_angle;
}

void BoomControl::process_boom_command()
{
	boom_command_s cmd;
	if (_boom_command_sub.update(&cmd)) {
		_last_command_time = hrt_absolute_time();

		// Handle emergency stop first
		if (cmd.emergency_stop) {
			emergency_stop();
			return;
		}

		// Process command based on mode
		switch (cmd.control_mode) {
		case 0: // Position control
			_target_boom_angle = cmd.lift_angle_cmd;

			// Update trajectory parameters
			_trajectory_generator.setMaxVelocityZ(cmd.max_lift_velocity);
			_trajectory_generator.setMaxAccelerationZ(_param_boom_max_acc.get());
			_trajectory_generator.setMaxJerkZ(_param_boom_max_jerk.get());

			_state = BoomState::MOVING;
			break;

		case 1: // Velocity control
		{
			// Direct velocity command - integrate to get position
			float dt = 0.01f; // Assume 100Hz update rate
			_target_boom_angle += cmd.lift_velocity_cmd * dt; // lift_velocity_cmd used as velocity

			_state = BoomState::MOVING;
			break;
		}

		case 2: // Force control (not implemented yet)
			PX4_WARN("Force control mode not implemented");
			break;

		default:
			PX4_WARN("Unknown boom command mode: %d", cmd.control_mode);
			break;
		}
	}
}

void BoomControl::set_target_position(BoomPreset preset)
{
	switch (preset) {
	case BoomPreset::GROUND:
		_target_boom_angle = _param_boom_pos_ground.get();
		break;
	case BoomPreset::CARRY:
		_target_boom_angle = _param_boom_pos_carry.get();
		break;
	case BoomPreset::MAX_HEIGHT:
		_target_boom_angle = _param_boom_pos_max.get();
		break;
	default:
		return; // Don't change target for unknown preset
	}

	_state = BoomState::MOVING;
}

void BoomControl::update_trajectory()
{
	if (_state == BoomState::ERROR) {
		return;
	}

	// Convert boom angle to actuator length for trajectory planning
	_target_actuator_length = boom_angle_to_actuator_length(_target_boom_angle);

	// Use PositionSmoothing for trajectory generation (1D motion using Z-axis)
	matrix::Vector3f current_pos{0.0f, 0.0f, _current_actuator_length / 1000.0f}; // Convert to meters
	matrix::Vector3f target_pos{0.0f, 0.0f, _target_actuator_length / 1000.0f};
	matrix::Vector3f feedforward_velocity{0.0f, 0.0f, 0.0f};

	float dt = 1.0f / _param_update_rate.get();
	PositionSmoothing::PositionSmoothingSetpoints setpoints;
	_trajectory_generator.generateSetpoints(current_pos, target_pos, feedforward_velocity,
	                                       dt, false, setpoints);

	// Store the generated setpoints for use in position control
	_desired_position_m = setpoints.position(2);
	_desired_velocity_m_s = setpoints.velocity(2);
}

void BoomControl::run_position_control()
{
	if (_state == BoomState::ERROR || !_sensor_valid) {
		_motor_output = 0.0f;
		return;
	}

	// Use the setpoints from trajectory generator (already in meters)
	float desired_position_m = _desired_position_m;
	float desired_velocity_m_s = _desired_velocity_m_s;

	// Convert back to mm for control
	float desired_position_mm = desired_position_m * 1000.0f;

	// Position error in mm
	float position_error = desired_position_mm - _current_actuator_length;

	// Check if we've reached the target
	if (fabsf(position_error) < 5.0f && fabsf(desired_velocity_m_s) < 0.001f) { // 5mm tolerance
		_state = BoomState::HOLDING;
	}

	// PID control
	float dt = 1.0f / _param_update_rate.get();
	_motor_output = _position_pid.update(position_error, dt);

	// Add velocity feedforward
	float velocity_ff = desired_velocity_m_s / (_param_boom_max_vel.get() / 1000.0f);
	_motor_output += velocity_ff * 0.3f; // Feedforward gain

	// Apply deadzone compensation
	if (fabsf(_motor_output) > _param_motor_deadzone.get()) {
		if (_motor_output > 0.0f) {
			_motor_output += _param_motor_deadzone.get();
		} else {
			_motor_output -= _param_motor_deadzone.get();
		}
	}
}

void BoomControl::publish_hbridge_command()
{
	hbridge_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();

	// Set instance from parameter (use instance instead of channel)
	uint8_t hbridge_instance = static_cast<uint8_t>(_param_hbridge_channel.get());
	cmd.instance = hbridge_instance;

	// Apply deadzone to prevent motor hunting around zero
	float output = _motor_output;
	if (fabsf(output) < _param_motor_deadzone.get()) {
		output = 0.0f;
	}

	// Set duty cycle (-1.0 to 1.0 range)
	// Positive = extend actuator (boom up), Negative = retract actuator (boom down)
	cmd.duty_cycle = output;

	// Enable channel if not in error state
	cmd.enable = (_state != BoomState::ERROR);

	// Publish to specific message instance for this motor
	if (_hbridge_command_pub == nullptr) {
		int instance = hbridge_instance;
		_hbridge_command_pub = orb_advertise_multi(ORB_ID(hbridge_command), nullptr, &instance);
	}

	if (_hbridge_command_pub != nullptr) {
		orb_publish(ORB_ID(hbridge_command), _hbridge_command_pub, &cmd);
	}
}

void BoomControl::publish_boom_status()
{
	boom_status_s status{};
	status.timestamp = hrt_absolute_time();

	// Position and velocity in radians and rad/s
	status.angle = math::radians(_current_boom_angle);
	status.velocity = _desired_velocity_m_s; // Using desired velocity as current velocity estimate

	// Load estimation (using motor output as proxy)
	status.load = fabsf(_motor_output);

	// Motor information
	status.motor_current = 0.0f; // Not available from H-bridge feedback yet
	status.motor_voltage = 0.0f; // Not available from H-bridge feedback yet
	status.motor_temperature_c = 0.0f; // Not available from H-bridge feedback yet
	status.motor_fault = (_state == BoomState::ERROR);
	status.encoder_fault = !_sensor_valid;

	// Control state mapping
	switch (_state) {
		case BoomState::IDLE:
			status.state = 2; // ready
			break;
		case BoomState::MOVING:
			status.state = 3; // moving
			break;
		case BoomState::HOLDING:
			status.state = 2; // ready
			break;
		case BoomState::ERROR:
			status.state = 4; // error
			break;
	}

	_boom_status_pub.publish(status);
}

void BoomControl::emergency_stop()
{
	_motor_output = 0.0f;
	_state = BoomState::ERROR;
	PX4_ERR("Emergency stop activated");
}

bool BoomControl::is_sensor_timeout()
{
	return (hrt_absolute_time() - _last_sensor_update) > 500_ms; // 500ms timeout
}

void BoomControl::reset_trajectory()
{
	// Reset trajectory to current position
	// Note: PositionSmoothing doesn't have simple setters, so we reset our internal state
	_desired_position_m = _current_actuator_length / 1000.0f;
	_desired_velocity_m_s = 0.0f;
}
