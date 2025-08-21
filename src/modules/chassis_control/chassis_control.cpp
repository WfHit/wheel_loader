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

#include "chassis_control.hpp"

ChassisControl::ChassisControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	_mpc_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": mpc");
	_slip_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": slip");
}

ChassisControl::~ChassisControl()
{
	perf_free(_loop_perf);
	perf_free(_mpc_perf);
	perf_free(_slip_perf);

	delete _slip_estimator;
	delete _torque_distributor;
}

bool ChassisControl::init()
{
	// Initialize existing articulated chassis components
	_slip_estimator = new SlipEstimator();
	_torque_distributor = new LoadAwareTorqueDistribution();

	if (!_slip_estimator || !_torque_distributor) {
		PX4_ERR("Failed to allocate slip estimator or torque distributor");
		return false;
	}

	// Initialize PID controllers
	pid_init(&_velocity_pid, PID_MODE_DERIVATIV_CALC, 0.01f);
	pid_init(&_steering_pid, PID_MODE_DERIVATIV_CALC, 0.01f);

	// Load parameters
	update_parameters();

	// Configure vehicle model
	_vehicle_model.mass_kg = _param_mass.get();
	_vehicle_model.inertia_kgm2 = _param_inertia.get();
	_vehicle_model.wheelbase_m = _param_wheelbase.get();
	_vehicle_model.track_width_m = _param_track_width.get();
	_vehicle_model.wheel_radius_m = _param_wheel_radius.get();
	_vehicle_model.max_torque_nm = _param_max_torque.get();

	// Initialize slip estimator with vehicle parameters
	if (_slip_estimator) {
		_slip_estimator->set_vehicle_parameters(_vehicle_model.mass_kg,
							_vehicle_model.wheel_radius_m,
							_vehicle_model.track_width_m);
	}

	// Initialize torque distributor
	if (_torque_distributor) {
		_torque_distributor->set_vehicle_parameters(_vehicle_model.mass_kg,
							    _vehicle_model.wheelbase_m,
							    _vehicle_model.track_width_m);
	}

	// Configure PID gains
	pid_set_parameters(&_velocity_pid,
			   _param_velocity_p.get(),
			   _param_velocity_i.get(),
			   _param_velocity_d.get(),
			   1.0f, 1.0f);

	pid_set_parameters(&_steering_pid,
			   _param_steering_p.get(),
			   _param_steering_i.get(),
			   _param_steering_d.get(),
			   1.0f, 1.0f);

	// Enable MPC if configured
	_mpc_enabled = _param_mpc_enable.get();

	_chassis_state = ChassisState::READY;

	ScheduleOnInterval(CONTROL_PERIOD_US);

	return true;
}

void ChassisControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	perf_begin(_loop_perf);

	// Update parameters if needed
	if (_parameter_update_sub.updated()) {
		update_parameters();
	}

	// Process chassis commands
	process_chassis_command();

	// Update sensor feedback
	update_wheel_encoders();
	update_vehicle_state();

	// Update chassis state
	update_chassis_state();

	// Main control execution
	if (_chassis_state == ChassisState::ACTIVE ||
	    _chassis_state == ChassisState::TRACTION_LIMITED ||
	    _chassis_state == ChassisState::POWER_LIMITED) {

		// Update slip estimation at 50Hz
		if (hrt_elapsed_time(&_last_slip_update) >= SLIP_PERIOD_US) {
			perf_begin(_slip_perf);
			update_traction_control();
			perf_end(_slip_perf);
			_last_slip_update = hrt_absolute_time();
		}

		// Update MPC at 20Hz if enabled
		if (_mpc_enabled && _control_mode == ControlMode::MPC &&
		    hrt_elapsed_time(&_last_mpc_update) >= MPC_PERIOD_US) {
			perf_begin(_mpc_perf);
			update_mpc_control();
			perf_end(_mpc_perf);
			_last_mpc_update = hrt_absolute_time();
		}

		// Compute motion control
		compute_motion_control();

		// Apply power limits if needed
		if (_chassis_state == ChassisState::POWER_LIMITED) {
			apply_power_limits(_wheel_torques);
		}

		// Publish commands
		publish_wheel_commands();
		publish_steering_command();
	}

	// Publish status
	publish_chassis_status();

	perf_end(_loop_perf);
}

void ChassisControl::process_chassis_command()
{
	chassis_command_s cmd;
	if (_chassis_command_sub.update(&cmd)) {
		_current_command = cmd;
		_last_command_time = hrt_absolute_time();

		// Determine control mode from command
		if (cmd.mode == chassis_command_s::MODE_VELOCITY) {
			_control_mode = _mpc_enabled ? ControlMode::MPC : ControlMode::VELOCITY;
			_chassis_state = ChassisState::ACTIVE;
		} else if (cmd.mode == chassis_command_s::MODE_POSITION) {
			_control_mode = ControlMode::POSITION;
			_chassis_state = ChassisState::ACTIVE;
		} else {
			_control_mode = ControlMode::IDLE;
		}

		// Handle emergency stop
		if (cmd.emergency_stop) {
			_control_mode = ControlMode::IDLE;
			_chassis_state = ChassisState::FAULT;
		}
	}

	// Check for command timeout
	if (hrt_elapsed_time(&_last_command_time) > 500_ms) {
		_control_mode = ControlMode::IDLE;
		if (_chassis_state == ChassisState::ACTIVE) {
			_chassis_state = ChassisState::READY;
		}
	}
}

void ChassisControl::update_chassis_state()
{
	// State transitions based on conditions
	ChassisState new_state = _chassis_state;

	switch (_chassis_state) {
	case ChassisState::INIT:
		new_state = ChassisState::READY;
		break;

	case ChassisState::READY:
		if (_control_mode != ControlMode::IDLE) {
			new_state = ChassisState::ACTIVE;
		}
		break;

	case ChassisState::ACTIVE:
		if (_control_mode == ControlMode::IDLE) {
			new_state = ChassisState::READY;
		} else if (_slip_estimator && _slip_estimator->is_slip_detected()) {
			new_state = ChassisState::TRACTION_LIMITED;
		} else if (_power_limit_factor < 0.8f) {
			new_state = ChassisState::POWER_LIMITED;
		}
		break;

	case ChassisState::TRACTION_LIMITED:
		if (_control_mode == ControlMode::IDLE) {
			new_state = ChassisState::READY;
		} else if (_slip_estimator && !_slip_estimator->is_slip_detected()) {
			new_state = ChassisState::ACTIVE;
		}
		break;

	case ChassisState::POWER_LIMITED:
		if (_control_mode == ControlMode::IDLE) {
			new_state = ChassisState::READY;
		} else if (_power_limit_factor >= 0.9f) {
			new_state = ChassisState::ACTIVE;
		}
		break;

	case ChassisState::FAULT:
		if (!_current_command.emergency_stop) {
			new_state = ChassisState::READY;
		}
		break;
	}

	if (new_state != _chassis_state) {
		PX4_INFO("Chassis state: %d -> %d", static_cast<int>(_chassis_state), static_cast<int>(new_state));
		_chassis_state = new_state;
	}
}

void ChassisControl::compute_motion_control()
{
	switch (_control_mode) {
	case ControlMode::VELOCITY:
		compute_velocity_control(_current_command);
		break;

	case ControlMode::POSITION:
		compute_position_control(_current_command);
		break;

	case ControlMode::MPC:
		// MPC control is handled in update_mpc_control()
		break;

	default:
		// Stop all motion
		for (int i = 0; i < NUM_WHEELS; i++) {
			_wheel_speeds[i] = 0.0f;
			_wheel_torques[i] = 0.0f;
		}
		_current_steering_angle = 0.0f;
		break;
	}
}

void ChassisControl::compute_velocity_control(const chassis_command_s &cmd)
{
	float desired_velocity = cmd.linear_velocity;
	float desired_angular_velocity = cmd.angular_velocity;

	// Apply motion limits
	apply_motion_limits(desired_velocity, desired_angular_velocity);

	// Coordinate wheel speeds for articulated steering
	coordinate_wheel_speeds(desired_velocity, desired_angular_velocity);

	// Compute steering angle for articulated chassis
	float desired_curvature = 0.0f;
	if (fabsf(desired_velocity) > 0.1f) {
		desired_curvature = desired_angular_velocity / desired_velocity;
	}

	// Simple Ackermann model for articulated steering
	_current_steering_angle = atanf(desired_curvature * _vehicle_model.wheelbase_m);
	_current_steering_angle = math::constrain(_current_steering_angle,
						  -_param_max_steering.get(),
						  _param_max_steering.get());

	// Update current state
	_current_velocity = desired_velocity;
	_current_angular_velocity = desired_angular_velocity;
}

void ChassisControl::coordinate_wheel_speeds(float linear_vel, float angular_vel)
{
	// Differential drive with articulated steering consideration
	float wheel_radius = _vehicle_model.wheel_radius_m;
	float track_width = _vehicle_model.track_width_m;

	// Base wheel speed from linear velocity
	float base_speed = linear_vel / wheel_radius;

	// Differential component from angular velocity
	float diff_speed = (angular_vel * track_width * 0.5f) / wheel_radius;

	// Apply to wheels with consideration for articulated steering
	_wheel_speeds[FRONT_LEFT] = base_speed - diff_speed;
	_wheel_speeds[FRONT_RIGHT] = base_speed + diff_speed;
	_wheel_speeds[REAR_LEFT] = base_speed - diff_speed;
	_wheel_speeds[REAR_RIGHT] = base_speed + diff_speed;

	// Compute torques using torque distributor if available
	if (_torque_distributor) {
		float base_torque = base_speed * 100.0f;  // Simple torque estimation
		_torque_distributor->compute_optimal_distribution(base_torque, _wheel_torques);
	} else {
		// Fallback: uniform torque distribution
		for (int i = 0; i < NUM_WHEELS; i++) {
			_wheel_torques[i] = _wheel_speeds[i] * 100.0f;  // Simple speed-to-torque
		}
	}

	// Enforce torque limits
	enforce_wheel_torque_limits(_wheel_torques);
}

void ChassisControl::update_traction_control()
{
	if (_slip_estimator) {
		// Update slip estimator with current wheel speeds and vehicle state
		_slip_estimator->update(_wheel_speeds, _current_velocity, _current_angular_velocity);

		// Apply anti-slip regulation if enabled
		if (_param_asr_enable.get()) {
			apply_anti_slip_regulation();
		}

		// Apply stability control if enabled
		if (_param_esp_enable.get()) {
			apply_stability_control();
		}
	}
}

void ChassisControl::apply_anti_slip_regulation()
{
	if (!_slip_estimator) {
		return;
	}

	// Get slip ratios for each wheel
	float slip_ratios[NUM_WHEELS];
	_slip_estimator->get_slip_ratios(slip_ratios);

	// Reduce torque on slipping wheels
	for (int i = 0; i < NUM_WHEELS; i++) {
		if (fabsf(slip_ratios[i]) > _param_slip_threshold.get()) {
			// Proportional torque reduction based on slip amount
			float slip_error = fabsf(slip_ratios[i]) - _param_slip_threshold.get();
			float reduction_factor = 1.0f - math::min(slip_error * 3.0f, 0.7f);
			_wheel_torques[i] *= reduction_factor;

			// Also reduce wheel speed to regain traction
			_wheel_speeds[i] *= 0.95f;
		}
	}
}

void ChassisControl::apply_stability_control()
{
	// Simple stability control based on lateral acceleration estimation
	float lateral_accel = _current_angular_velocity * _current_velocity;
	float max_lateral_accel = 4.0f;  // m/s^2

	if (fabsf(lateral_accel) > max_lateral_accel) {
		// Reduce angular velocity command to maintain stability
		float reduction_factor = max_lateral_accel / fabsf(lateral_accel);
		_current_angular_velocity *= reduction_factor;

		// Recompute wheel speeds with reduced angular velocity
		coordinate_wheel_speeds(_current_velocity, _current_angular_velocity);
	}
}

void ChassisControl::update_mpc_control()
{
	// Update current state for MPC
	vehicle_odometry_s odom;
	if (_vehicle_odometry_sub.copy(&odom)) {
		_mpc_state.x_current(0) = odom.x;
		_mpc_state.x_current(1) = odom.y;
		_mpc_state.x_current(2) = odom.yaw;
	}
	_mpc_state.x_current(3) = _current_velocity;
	_mpc_state.x_current(4) = _current_angular_velocity;

	// Set reference state from command
	_mpc_state.x_ref(3) = _current_command.linear_velocity;
	_mpc_state.x_ref(4) = _current_command.angular_velocity;

	// Solve MPC optimization
	solve_mpc_problem();

	// Apply first control from optimal sequence
	if (_mpc_state.solution_valid) {
		apply_mpc_control();
	}
}

void ChassisControl::solve_mpc_problem()
{
	// Simplified MPC using gradient descent
	float dt = MPC_PERIOD_US * 1e-6f;

	// Initialize control sequence with current command
	for (size_t i = 0; i < MPC_HORIZON; i++) {
		_mpc_state.u_opt(0, i) = _current_command.linear_velocity;
		_mpc_state.u_opt(1, i) = _current_command.angular_velocity;
	}

	// Simple gradient descent optimization
	const int max_iterations = 5;  // Limited for real-time execution
	float learning_rate = 0.05f;

	for (int iter = 0; iter < max_iterations; iter++) {
		// Compute cost
		float cost = compute_mpc_cost(_mpc_state.u_opt);

		// Numerical gradient (simplified)
		const float delta = 0.01f;
		Matrix<float, MPC_CONTROL_DIM, MPC_HORIZON> gradient{};

		for (size_t i = 0; i < MPC_CONTROL_DIM; i++) {
			for (size_t j = 0; j < MPC_HORIZON; j++) {
				_mpc_state.u_opt(i, j) += delta;
				float cost_plus = compute_mpc_cost(_mpc_state.u_opt);
				_mpc_state.u_opt(i, j) -= 2 * delta;
				float cost_minus = compute_mpc_cost(_mpc_state.u_opt);
				_mpc_state.u_opt(i, j) += delta;

				gradient(i, j) = (cost_plus - cost_minus) / (2 * delta);
			}
		}

		// Update control sequence
		_mpc_state.u_opt -= gradient * learning_rate;

		// Apply constraints
		for (size_t j = 0; j < MPC_HORIZON; j++) {
			_mpc_state.u_opt(0, j) = math::constrain(_mpc_state.u_opt(0, j),
								 -_param_max_speed.get(),
								 _param_max_speed.get());
			_mpc_state.u_opt(1, j) = math::constrain(_mpc_state.u_opt(1, j),
								 -M_PI_F, M_PI_F);
		}
	}

	_mpc_state.solution_valid = true;
}

float ChassisControl::compute_mpc_cost(const Matrix<float, MPC_CONTROL_DIM, MPC_HORIZON> &u)
{
	float cost = 0.0f;
	float dt = MPC_PERIOD_US * 1e-6f;

	Vector<float, MPC_STATE_DIM> x = _mpc_state.x_current;

	// Predict states and compute cost
	for (size_t k = 0; k < MPC_HORIZON; k++) {
		// Simple kinematic model prediction
		x(0) += x(3) * cosf(x(2)) * dt;  // x position
		x(1) += x(3) * sinf(x(2)) * dt;  // y position
		x(2) += x(4) * dt;               // heading
		x(3) = u(0, k);                  // velocity
		x(4) = u(1, k);                  // angular velocity

		// State cost (tracking error)
		Vector<float, MPC_STATE_DIM> x_error = x - _mpc_state.x_ref;
		cost += _param_mpc_q_x.get() * x_error(0) * x_error(0);
		cost += _param_mpc_q_y.get() * x_error(1) * x_error(1);
		cost += _param_mpc_q_theta.get() * x_error(2) * x_error(2);

		// Control cost
		cost += _param_mpc_r_v.get() * u(0, k) * u(0, k);
		cost += _param_mpc_r_omega.get() * u(1, k) * u(1, k);
	}

	return cost;
}

void ChassisControl::apply_mpc_control()
{
	// Apply first control from MPC solution
	float v_cmd = _mpc_state.u_opt(0, 0);
	float omega_cmd = _mpc_state.u_opt(1, 0);

	// Update current command with MPC output
	_current_command.linear_velocity = v_cmd;
	_current_command.angular_velocity = omega_cmd;

	// Compute wheel speeds for MPC commands
	coordinate_wheel_speeds(v_cmd, omega_cmd);
}

void ChassisControl::apply_power_limits(float wheel_torques[NUM_WHEELS])
{
	// Calculate total power requirement
	float total_power = 0.0f;
	for (int i = 0; i < NUM_WHEELS; i++) {
		total_power += fabsf(wheel_torques[i] * _wheel_speeds[i]);
	}

	// Apply power limit
	if (total_power > _available_power_w && total_power > 0.0f) {
		float scale_factor = _available_power_w / total_power;
		for (int i = 0; i < NUM_WHEELS; i++) {
			wheel_torques[i] *= scale_factor;
		}
	}
}

void ChassisControl::publish_wheel_commands()
{
	wheel_speeds_setpoint_s wheel_cmd{};
	wheel_cmd.timestamp = hrt_absolute_time();

	// Front left wheel
	wheel_cmd.wheel_id = FRONT_LEFT;
	wheel_cmd.speed_setpoint = _wheel_speeds[FRONT_LEFT];
	wheel_cmd.torque_setpoint = _wheel_torques[FRONT_LEFT];
	_fl_wheel_pub.publish(wheel_cmd);

	// Front right wheel
	wheel_cmd.wheel_id = FRONT_RIGHT;
	wheel_cmd.speed_setpoint = _wheel_speeds[FRONT_RIGHT];
	wheel_cmd.torque_setpoint = _wheel_torques[FRONT_RIGHT];
	_fr_wheel_pub.publish(wheel_cmd);

	// Rear left wheel
	wheel_cmd.wheel_id = REAR_LEFT;
	wheel_cmd.speed_setpoint = _wheel_speeds[REAR_LEFT];
	wheel_cmd.torque_setpoint = _wheel_torques[REAR_LEFT];
	_rl_wheel_pub.publish(wheel_cmd);

	// Rear right wheel
	wheel_cmd.wheel_id = REAR_RIGHT;
	wheel_cmd.speed_setpoint = _wheel_speeds[REAR_RIGHT];
	wheel_cmd.torque_setpoint = _wheel_torques[REAR_RIGHT];
	_rr_wheel_pub.publish(wheel_cmd);
}

void ChassisControl::publish_steering_command()
{
	steering_command_s steering_cmd{};
	steering_cmd.timestamp = hrt_absolute_time();
	steering_cmd.steering_angle = _current_steering_angle;
	steering_cmd.steering_rate = 0.0f; // Let steering module handle rate

	_steering_command_pub.publish(steering_cmd);
}

void ChassisControl::publish_chassis_status()
{
	chassis_status_s status{};
	status.timestamp = hrt_absolute_time();

	// Motion state
	status.linear_velocity = _current_velocity;
	status.angular_velocity = _current_angular_velocity;
	status.steering_angle = _current_steering_angle;

	// Wheel state
	for (int i = 0; i < NUM_WHEELS; i++) {
		status.wheel_speeds[i] = _wheel_speeds[i];
		status.wheel_torques[i] = _wheel_torques[i];

		// Get slip ratios from slip estimator if available
		if (_slip_estimator) {
			float slip_ratios[NUM_WHEELS];
			_slip_estimator->get_slip_ratios(slip_ratios);
			status.wheel_slip[i] = slip_ratios[i];
		} else {
			status.wheel_slip[i] = 0.0f;
		}
	}

	// Traction info
	if (_slip_estimator) {
		status.traction_mu = _slip_estimator->get_average_friction();
		status.asr_active = _param_asr_enable.get() && _slip_estimator->is_slip_detected();
	} else {
		status.traction_mu = 0.8f;  // Default assumption
		status.asr_active = false;
	}

	status.esp_active = _param_esp_enable.get();

	// System state
	status.mode = static_cast<uint8_t>(_control_mode);
	status.state = static_cast<uint8_t>(_chassis_state);
	status.health = check_system_stability() ? chassis_status_s::HEALTH_OK :
		       chassis_status_s::HEALTH_WARNING;

	_chassis_status_pub.publish(status);
}

void ChassisControl::apply_motion_limits(float &linear_vel, float &angular_vel)
{
	// Apply velocity limits
	linear_vel = math::constrain(linear_vel, -_param_max_speed.get(), _param_max_speed.get());
	angular_vel = math::constrain(angular_vel, -M_PI_F, M_PI_F);

	// Apply acceleration limits (simplified)
	static float prev_linear_vel = 0.0f;
	float dt = CONTROL_PERIOD_US * 1e-6f;
	float max_accel = _param_max_accel.get();

	float vel_diff = linear_vel - prev_linear_vel;
	float max_vel_change = max_accel * dt;

	if (fabsf(vel_diff) > max_vel_change) {
		linear_vel = prev_linear_vel + copysignf(max_vel_change, vel_diff);
	}

	prev_linear_vel = linear_vel;
}

void ChassisControl::enforce_wheel_torque_limits(float torques[NUM_WHEELS])
{
	float max_torque = _vehicle_model.max_torque_nm;

	for (int i = 0; i < NUM_WHEELS; i++) {
		torques[i] = math::constrain(torques[i], -max_torque, max_torque);
	}
}

bool ChassisControl::check_system_stability()
{
	// Simple stability check based on wheel speed differences
	float max_speed_diff = 0.0f;
	for (int i = 0; i < NUM_WHEELS - 1; i++) {
		for (int j = i + 1; j < NUM_WHEELS; j++) {
			float diff = fabsf(_wheel_speeds[i] - _wheel_speeds[j]);
			max_speed_diff = math::max(max_speed_diff, diff);
		}
	}

	// Consider system unstable if wheel speeds differ too much
	return max_speed_diff < 5.0f;  // rad/s
}

void ChassisControl::update_vehicle_state()
{
	// Update vehicle state from odometry
	vehicle_odometry_s odom;
	if (_vehicle_odometry_sub.copy(&odom)) {
		_current_velocity = sqrtf(odom.vx * odom.vx + odom.vy * odom.vy);
		_current_angular_velocity = odom.yawspeed;
	}

	// Update steering angle from steering status
	steering_status_s steering_status;
	if (_steering_status_sub.copy(&steering_status)) {
		_current_steering_angle = steering_status.steering_angle;
	}
}

void ChassisControl::update_wheel_encoders()
{
	// Update wheel speeds from encoders
	for (int i = 0; i < NUM_WHEELS; i++) {
		wheel_encoder_s encoder;
		if (_wheel_encoder_subs[i].copy(&encoder)) {
			_wheel_speeds[i] = encoder.speed;
		}
	}
}

void ChassisControl::update_parameters()
{
	updateParams();

	// Update PID gains if they changed
	pid_set_parameters(&_velocity_pid,
			   _param_velocity_p.get(),
			   _param_velocity_i.get(),
			   _param_velocity_d.get(),
			   1.0f, 1.0f);

	pid_set_parameters(&_steering_pid,
			   _param_steering_p.get(),
			   _param_steering_i.get(),
			   _param_steering_d.get(),
			   1.0f, 1.0f);

	// Update MPC enable flag
	_mpc_enabled = _param_mpc_enable.get();
}

void ChassisControl::compute_position_control(const chassis_command_s &cmd)
{
	// Simple position control using PID
	vehicle_odometry_s odom;
	if (!_vehicle_odometry_sub.copy(&odom)) {
		return;
	}

	// Position errors
	float x_error = cmd.x_position - odom.x;
	float y_error = cmd.y_position - odom.y;
	float heading_error = cmd.heading - odom.yaw;

	// Wrap heading error
	heading_error = matrix::wrap_pi(heading_error);

	// Simple position to velocity conversion
	float desired_velocity = sqrtf(x_error * x_error + y_error * y_error) * 0.5f;
	desired_velocity = math::constrain(desired_velocity, 0.0f, _param_max_speed.get());

	float desired_angular_velocity = heading_error * 1.0f;
	desired_angular_velocity = math::constrain(desired_angular_velocity, -M_PI_F, M_PI_F);

	// Use velocity control for the computed velocities
	chassis_command_s vel_cmd = cmd;
	vel_cmd.linear_velocity = desired_velocity;
	vel_cmd.angular_velocity = desired_angular_velocity;
	compute_velocity_control(vel_cmd);
}

int ChassisControl::task_spawn(int argc, char *argv[])
{
	ChassisControl *instance = new ChassisControl();

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

int ChassisControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ChassisControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Advanced chassis control with MPC and slip-aware traction control.

Features:
- Model Predictive Control for trajectory tracking
- Slip estimation and optimal traction control using existing slip_estimator
- Load-aware torque distribution using existing load_aware_torque
- Anti-slip regulation (ASR) and stability control (ESP)
- Power-aware torque distribution
- Articulated steering coordination

This module leverages existing articulated_chassis components for
enhanced traction and stability control.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("chassis_control", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
