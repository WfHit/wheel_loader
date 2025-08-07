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

#include "bucket_motion_controller.hpp"
#include <px4_platform_common/log.h>

BucketMotionController::BucketMotionController(ModuleParams *parent) :
	ModuleParams(parent)
{
	// Initialize PID controllers with default gains
	_position_controller.setGains(1.0f, 0.1f, 0.05f);
	_velocity_controller.setGains(2.0f, 0.2f, 0.01f);

	// Initialize motion planning with conservative defaults
	_velocity_smoother.setMaxAccel(100.0f);  // mm/s²
	_velocity_smoother.setMaxVel(50.0f);     // mm/s
	_velocity_smoother.setMaxJerk(500.0f);   // mm/s³

	_position_smoother.setMaxAccelerationZ(100.0f);
	_position_smoother.setMaxVelocityZ(50.0f);
	_position_smoother.setMaxJerkZ(500.0f);
}

void BucketMotionController::initialize(const ControllerConfig& config)
{
	_config = config;

	// Configure PID controllers
	_position_controller.setGains(config.position_p, config.position_i, config.position_d);
	_velocity_controller.setGains(config.velocity_p, config.velocity_i, config.velocity_d);

	// Configure motion planning
	_velocity_smoother.setMaxAccel(config.max_acceleration);
	_velocity_smoother.setMaxVel(config.max_velocity);
	_velocity_smoother.setMaxJerk(config.max_jerk);

	_position_smoother.setMaxAccelerationZ(config.max_acceleration);
	_position_smoother.setMaxVelocityZ(config.max_velocity);
	_position_smoother.setMaxJerkZ(config.max_jerk);

	// Initialize safety limits
	set_safety_limits(config.position_min, config.position_max);

	_initialized = true;

	PX4_INFO("Motion controller initialized with max_vel=%.1f, max_acc=%.1f",
		 (double)config.max_velocity, (double)config.max_acceleration);
}

void BucketMotionController::update_config(const ControllerConfig& config)
{
	if (!_initialized) {
		initialize(config);
		return;
	}

	_config = config;

	// Update PID gains
	_position_controller.setGains(config.position_p, config.position_i, config.position_d);
	_velocity_controller.setGains(config.velocity_p, config.velocity_i, config.velocity_d);

	// Update motion constraints
	_velocity_smoother.setMaxAccel(config.max_acceleration);
	_velocity_smoother.setMaxVel(config.max_velocity);
	_velocity_smoother.setMaxJerk(config.max_jerk);

	_position_smoother.setMaxAccelerationZ(config.max_acceleration);
	_position_smoother.setMaxVelocityZ(config.max_velocity);
	_position_smoother.setMaxJerkZ(config.max_jerk);

	// Update safety limits
	set_safety_limits(config.position_min, config.position_max);
}

BucketMotionController::MotionSetpoint BucketMotionController::plan_trajectory(
	float current_position, float target_position, float dt)
{
	MotionSetpoint setpoint{};

	if (!_initialized) {
		PX4_WARN("Motion controller not initialized");
		setpoint.position = current_position;
		return setpoint;
	}

	// Use position smoother for 1D trajectory generation (Z-axis)
	Vector3f current_pos{0.0f, 0.0f, current_position};
	Vector3f target_pos{0.0f, 0.0f, target_position};
	Vector3f feedforward_vel{0.0f, 0.0f, 0.0f};

	PositionSmoothing::PositionSmoothingSetpoints smooth_setpoints;
	_position_smoother.generateSetpoints(current_pos, target_pos, feedforward_vel,
					     dt, false, smooth_setpoints);

	// Extract trajectory from Z-component
	setpoint.position = smooth_setpoints.position(2);
	setpoint.velocity = smooth_setpoints.velocity(2);
	setpoint.acceleration = smooth_setpoints.acceleration(2);
	setpoint.jerk = smooth_setpoints.jerk(2);

	// Apply safety constraints
	setpoint.position = math::constrain(setpoint.position, _position_min_safe, _position_max_safe);
	setpoint.velocity = math::constrain(setpoint.velocity, -_config.max_velocity, _config.max_velocity);

	return setpoint;
}

BucketMotionController::ControlOutput BucketMotionController::compute_control(
	const MotionSetpoint& setpoint,
	float current_position,
	float current_velocity,
	float boom_angle,
	float dt)
{
	ControlOutput output{};

	if (!_initialized) {
		PX4_WARN("Motion controller not initialized");
		return output;
	}

	// Create working copy of setpoint for boom compensation
	MotionSetpoint compensated_setpoint = setpoint;

	// Apply boom compensation if enabled
	if (_boom_compensation.enabled) {
		apply_boom_compensation(compensated_setpoint, boom_angle, dt);
	}

	// Check safety constraints
	bool safe = check_safety_constraints(current_position, current_velocity, compensated_setpoint);
	if (!safe) {
		output.safety_stop = true;
		output.duty_cycle = 0.0f;
		return output;
	}

	// Cascade control: Position controller generates velocity command
	float position_error = compensated_setpoint.position - current_position;
	_position_controller.setSetpoint(compensated_setpoint.position);
	float velocity_command = _position_controller.update(current_position, dt);

	// Add velocity feedforward from trajectory
	velocity_command += compensated_setpoint.velocity;

	// Constrain velocity command
	velocity_command = math::constrain(velocity_command, -_config.max_velocity, _config.max_velocity);

	// Velocity controller generates motor duty cycle
	float velocity_error = velocity_command - current_velocity;
	_velocity_controller.setSetpoint(velocity_command);
	float duty_cycle = _velocity_controller.update(current_velocity, dt);

	// Add feedforward for better performance
	duty_cycle += compute_feedforward(compensated_setpoint);

	// Apply duty cycle limits
	duty_cycle = math::constrain(duty_cycle, -_config.duty_cycle_limit, _config.duty_cycle_limit);

	// Populate output
	output.duty_cycle = duty_cycle;
	output.position_error = position_error;
	output.velocity_error = velocity_error;
	output.safety_stop = false;

	// Update performance metrics
	update_performance_metrics(position_error, velocity_error, duty_cycle);

	return output;
}

void BucketMotionController::apply_boom_compensation(MotionSetpoint& setpoint, float boom_angle, float dt)
{
	if (!_boom_compensation.enabled) {
		return;
	}

	// Calculate boom angle change since last update
	float boom_delta = boom_angle - _boom_compensation.previous_angle;

	// Check if boom has moved significantly (deadband to avoid noise)
	if (fabsf(boom_delta) < _config.boom_deadband) {
		return;
	}

	// Update compensation factor based on current boom angle
	_boom_compensation.compensation_factor = update_compensation_factor(boom_angle);

	// Calculate required actuator adjustment
	float actuator_compensation = -boom_delta * _boom_compensation.compensation_factor;

	// Apply compensation to position setpoint
	setpoint.position += actuator_compensation;

	// Apply derivative compensation for smooth motion during boom movement
	if (_config.boom_derivative_gain > 0.0f && dt > 0.0f) {
		// Estimate boom angular velocity
		float boom_velocity = boom_delta / dt;

		// Add velocity feedforward to match boom motion
		float velocity_compensation = -boom_velocity * _boom_compensation.compensation_factor *
					      _config.boom_derivative_gain;
		setpoint.velocity += velocity_compensation;
	}

	// Update previous boom angle for next iteration
	_boom_compensation.previous_angle = boom_angle;
	_boom_compensation.last_update = hrt_absolute_time();

	// Log significant compensations for debugging
	if (fabsf(actuator_compensation) > 1.0f) {
		PX4_DEBUG("Boom compensation: Δboom=%.3f rad, Δactuator=%.1f mm",
			  (double)boom_delta, (double)actuator_compensation);
	}
}

float BucketMotionController::update_compensation_factor(float boom_angle)
{
	// Use parameter-based compensation factor with boom angle adjustment
	float base_factor = _param_boom_compensation_factor.get();

	// Simple linear adjustment based on boom angle
	// (More sophisticated models could use the kinematic Jacobian)
	const float ANGLE_SENSITIVITY = 0.2f;  // Factor change per radian
	float factor = base_factor * (1.0f + ANGLE_SENSITIVITY * boom_angle);

	// Limit factor to reasonable range
	return math::constrain(factor, 100.0f, 400.0f);
}

void BucketMotionController::set_boom_compensation(bool enable, float reference_boom_angle,
						   float compensation_factor)
{
	_boom_compensation.enabled = enable;
	_boom_compensation.reference_angle = reference_boom_angle;
	_boom_compensation.previous_angle = reference_boom_angle;
	_boom_compensation.compensation_factor = compensation_factor;
	_boom_compensation.last_update = hrt_absolute_time();

	if (enable) {
		PX4_INFO("Boom compensation enabled: ref_angle=%.2f rad, factor=%.1f mm/rad",
			 (double)reference_boom_angle, (double)compensation_factor);
	} else {
		PX4_INFO("Boom compensation disabled");
	}
}

bool BucketMotionController::check_safety_constraints(float position, float velocity,
						       const MotionSetpoint& setpoint)
{
	// Check position limits
	if (position < _position_min_safe || position > _position_max_safe) {
		if (!_safety_stop_active) {
			PX4_WARN("Position limit exceeded: %.1f mm (limits: %.1f - %.1f)",
				 (double)position, (double)_position_min_safe, (double)_position_max_safe);
			_safety_stop_active = true;
		}
		return false;
	}

	// Check velocity limits
	if (fabsf(velocity) > _config.max_velocity * 1.1f) { // 10% tolerance
		if (!_safety_stop_active) {
			PX4_WARN("Velocity limit exceeded: %.1f mm/s (limit: %.1f)",
				 (double)fabsf(velocity), (double)_config.max_velocity);
			_safety_stop_active = true;
		}
		return false;
	}

	// Check if setpoint is trying to drive us out of bounds
	if (setpoint.position < _position_min_safe || setpoint.position > _position_max_safe) {
		return false;
	}

	// Reset safety stop if constraints are satisfied
	_safety_stop_active = false;
	return true;
}

void BucketMotionController::set_safety_limits(float min_position, float max_position)
{
	_position_min_safe = min_position;
	_position_max_safe = max_position;

	PX4_DEBUG("Safety limits set: %.1f - %.1f mm", (double)min_position, (double)max_position);
}

float BucketMotionController::compute_feedforward(const MotionSetpoint& setpoint)
{
	// Simple velocity feedforward (can be enhanced with acceleration feedforward)
	const float VELOCITY_FEEDFORWARD_GAIN = 0.001f;  // Tunable parameter
	return setpoint.velocity * VELOCITY_FEEDFORWARD_GAIN;
}

void BucketMotionController::update_performance_metrics(float position_error, float velocity_error,
							float control_output)
{
	_performance_metrics.position_error_sum_squared += position_error * position_error;
	_performance_metrics.velocity_error_sum_squared += velocity_error * velocity_error;
	_performance_metrics.control_effort_sum += fabsf(control_output);
	_performance_metrics.sample_count++;

	// Reset metrics periodically to prevent overflow
	if (_performance_metrics.sample_count > 10000) {
		_performance_metrics.position_error_sum_squared *= 0.9f;
		_performance_metrics.velocity_error_sum_squared *= 0.9f;
		_performance_metrics.control_effort_sum *= 0.9f;
		_performance_metrics.sample_count = static_cast<uint32_t>(_performance_metrics.sample_count * 0.9f);
	}
}

void BucketMotionController::get_performance_metrics(float& position_rms_error, float& velocity_rms_error,
						      float& control_effort) const
{
	if (_performance_metrics.sample_count > 0) {
		position_rms_error = sqrtf(_performance_metrics.position_error_sum_squared /
					   _performance_metrics.sample_count);
		velocity_rms_error = sqrtf(_performance_metrics.velocity_error_sum_squared /
					   _performance_metrics.sample_count);
		control_effort = _performance_metrics.control_effort_sum / _performance_metrics.sample_count;
	} else {
		position_rms_error = 0.0f;
		velocity_rms_error = 0.0f;
		control_effort = 0.0f;
	}
}

void BucketMotionController::reset()
{
	// Reset PID controllers
	_position_controller.resetIntegral();
	_position_controller.resetDerivative();
	_velocity_controller.resetIntegral();
	_velocity_controller.resetDerivative();

	// Reset motion planning
	_velocity_smoother.reset(0.0f, 0.0f, 0.0f);  // Reset with zero acceleration, velocity, position
	_position_smoother.reset(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f));

	// Reset performance metrics
	_performance_metrics = PerformanceMetrics{};

	// Reset safety state
	_safety_stop_active = false;

	PX4_DEBUG("Motion controller reset");
}
