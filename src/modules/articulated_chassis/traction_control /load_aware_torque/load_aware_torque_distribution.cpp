#include "load_aware_torque_distribution.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/chassis_trajectory_setpoint.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

LoadAwareTorqueDistribution::LoadAwareTorqueDistribution() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// Initialize module status
	_status = {};
	_status.timestamp = hrt_absolute_time();

	// Initialize torque output
	_torque_output = {};
}

int LoadAwareTorqueDistribution::task_spawn(int argc, char *argv[])
{
	LoadAwareTorqueDistribution *instance = new LoadAwareTorqueDistribution();

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

LoadAwareTorqueDistribution *LoadAwareTorqueDistribution::instantiate(int argc, char *argv[])
{
	return new LoadAwareTorqueDistribution();
}

int LoadAwareTorqueDistribution::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LoadAwareTorqueDistribution::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Load-aware torque distribution for articulated wheel loader.
Optimizes torque allocation between front and rear wheels based on load and conditions.

### Implementation
The module runs at 50Hz and provides:
- Dynamic torque distribution based on load conditions
- Stability-aware torque allocation
- Efficiency optimization for fuel/energy savings
- Integration with traction control systems
- Real-time center of gravity estimation
- Comprehensive safety monitoring

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("load_aware_torque", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

bool LoadAwareTorqueDistribution::init()
{
	PX4_INFO("Load-Aware Torque Distribution initialized");
	PX4_INFO("Control rate: %.1f Hz", (double)CONTROL_RATE_HZ);
	PX4_INFO("Torque ratio limits: %.1f - %.1f",
		 (double)_min_front_ratio.get(), (double)_max_front_ratio.get());

	ScheduleOnInterval(CONTROL_INTERVAL_US);
	return true;
}

int LoadAwareTorqueDistribution::print_status()
{
	PX4_INFO("Load-Aware Torque Distribution Status:");
	PX4_INFO("  Active: %s", _torque_output.load_aware_active ? "YES" : "NO");
	PX4_INFO("  Total mass: %.1f kg", (double)_load_state.total_mass_kg);
	PX4_INFO("  Payload: %.1f kg", (double)_load_state.payload_mass_kg);
	PX4_INFO("  COG position: %.2f, %.2f m", (double)_load_state.cog_x_m, (double)_load_state.cog_z_m);
	PX4_INFO("  Torque split: %.2f", (double)_torque_distribution.final_distribution);
	PX4_INFO("  Front/Rear torque: %.1f/%.1f Nm",
		 (double)_torque_distribution.front_torque_nm, (double)_torque_distribution.rear_torque_nm);
	PX4_INFO("  Stability margin: %.2f", (double)_performance.stability_margin);
	PX4_INFO("  Efficiency factor: %.2f", (double)_performance.efficiency_factor);

	return 0;
}

void LoadAwareTorqueDistribution::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Update parameters if needed
	updateParams();

	// Update load state from sensors
	update_load_state();

	// Process chassis trajectory for predictive adjustments
	process_chassis_trajectory();

	// Calculate base torque distribution
	calculate_base_torque_distribution();

	// Apply dynamic adjustments
	apply_dynamic_adjustments();

	// Apply trajectory-based adjustments
	apply_trajectory_based_distribution();

	// Apply stability corrections
	apply_stability_corrections();

	// Apply terrain adaptations
	apply_terrain_adaptations();

	// Validate and apply safety limits
	validate_torque_distribution();

	// Optimize for efficiency/traction
	if (_efficiency_optimize.get()) {
		optimize_for_efficiency();
	}

	optimize_for_traction();

	// Update performance metrics
	update_performance_metrics();

	// Publish torque commands
	publish_torque_commands();
}

void LoadAwareTorqueDistribution::update_load_state()
{
	// Update boom and bucket status for load estimation
	if (_boom_status_sub.updated()) {
		boom_status_s boom_status;
		_boom_status_sub.copy(&boom_status);
		// Use boom position for load state estimation
	}

	if (_bucket_status_sub.updated()) {
		bucket_status_s bucket_status;
		_bucket_status_sub.copy(&bucket_status);
		// Use bucket status for load estimation
		_load_state.payload_mass_kg = bucket_status.estimated_load_kg;
		_load_state.total_mass_kg = EMPTY_MASS_KG + _load_state.payload_mass_kg;
	}

	// Update vehicle dynamics
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s att;
		_vehicle_attitude_sub.copy(&att);

		// Convert quaternion to Euler angles
		matrix::Eulerf euler(matrix::Quatf(att.q));
		_dynamics_state.pitch_angle_rad = euler.theta(); // pitch
		_dynamics_state.roll_angle_rad = euler.phi();    // roll
		_dynamics_state.dynamics_valid = true;
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s pos;
		_vehicle_local_position_sub.copy(&pos);

		_dynamics_state.vehicle_speed_ms = sqrtf(pos.vx * pos.vx + pos.vy * pos.vy);
		_dynamics_state.longitudinal_accel_ms2 = pos.ax;
		_dynamics_state.lateral_accel_ms2 = pos.ay;
	}

	_load_state.last_update_time = hrt_absolute_time();
}

void LoadAwareTorqueDistribution::calculate_static_weight_distribution()
{
	// Calculate static weight distribution based on COG
	estimate_center_of_gravity();

	float cog_x = _load_state.cog_x_m + _cog_x_offset.get();
	float wheelbase = WHEELBASE_M;

	// Calculate static front/rear weight distribution
	float front_weight_ratio = (wheelbase/2.0f - cog_x) / wheelbase;
	front_weight_ratio = math::constrain(front_weight_ratio, 0.1f, 0.9f);

	_load_state.front_axle_load_kg = _load_state.total_mass_kg * front_weight_ratio;
	_load_state.rear_axle_load_kg = _load_state.total_mass_kg * (1.0f - front_weight_ratio);
}

void LoadAwareTorqueDistribution::estimate_center_of_gravity()
{
	// Estimate COG based on load and vehicle geometry
	float load_factor = _load_state.payload_mass_kg / MAX_PAYLOAD_KG;

	// COG moves forward and up with load
	_load_state.cog_x_m = EMPTY_COG_X_M + (load_factor * 0.3f);  // Forward shift with load
	_load_state.cog_z_m = EMPTY_COG_Z_M + (load_factor * 0.2f);  // Height increase with load
}

void LoadAwareTorqueDistribution::calculate_dynamic_weight_transfer()
{
	// Calculate dynamic weight transfer due to acceleration
	float accel = _dynamics_state.longitudinal_accel_ms2;
	float cog_height = _load_state.cog_z_m + _cog_z_offset.get();
	float wheelbase = WHEELBASE_M;

	// Weight transfer due to longitudinal acceleration
	_load_state.dynamic_load_transfer = (accel * cog_height) / (9.81f * wheelbase);
	_load_state.dynamic_load_transfer = math::constrain(_load_state.dynamic_load_transfer,
							    -MAX_WEIGHT_TRANSFER_RATIO, MAX_WEIGHT_TRANSFER_RATIO);
}

void LoadAwareTorqueDistribution::calculate_base_torque_distribution()
{
	calculate_static_weight_distribution();

	// Base distribution follows weight distribution
	float front_weight_ratio = _load_state.front_axle_load_kg / _load_state.total_mass_kg;
	_torque_distribution.base_distribution = front_weight_ratio;
}

void LoadAwareTorqueDistribution::apply_dynamic_adjustments()
{
	calculate_dynamic_weight_transfer();

	// Apply dynamic weight transfer to torque distribution
	float dynamic_gain = _dynamic_gain.get();
	_torque_distribution.dynamic_adjustment = _load_state.dynamic_load_transfer * dynamic_gain;
}

void LoadAwareTorqueDistribution::apply_stability_corrections()
{
	// Check stability margins
	check_stability_margins();

	float stability_gain = _stability_gain.get();
	float pitch_correction = -_dynamics_state.pitch_angle_rad * 0.1f;  // Pitch compensation

	_torque_distribution.stability_correction = pitch_correction * stability_gain;
}

void LoadAwareTorqueDistribution::apply_terrain_adaptations()
{
	// Get traction control feedback
	if (_traction_control_sub.updated()) {
		traction_control_s traction;
		_traction_control_sub.copy(&traction);

		_traction_state.traction_control_active = traction.traction_control_active;
		_traction_state.slip_front = traction.slip_ratio_front;
		_traction_state.slip_rear = traction.slip_ratio_rear;
	}

	float terrain_gain = _terrain_gain.get();
	float slip_difference = _traction_state.slip_front - _traction_state.slip_rear;

	// Adjust distribution based on slip difference
	_torque_distribution.terrain_adaptation = -slip_difference * terrain_gain;
}

void LoadAwareTorqueDistribution::validate_torque_distribution()
{
	// Combine all adjustments
	_torque_distribution.final_distribution = _torque_distribution.base_distribution +
						  _torque_distribution.dynamic_adjustment +
						  _torque_distribution.stability_correction +
						  _torque_distribution.terrain_adaptation;

	// Apply safety limits
	apply_safety_limits();

	// Apply rate limiting and filtering
	float dt = CONTROL_INTERVAL_US / 1e6f;
	float alpha = dt / (_distribution_filter_tc.get() + dt);

	_filter_state.filtered_distribution = (1.0f - alpha) * _filter_state.filtered_distribution +
					      alpha * _torque_distribution.final_distribution;

	// Apply rate limiting
	float rate_limit = _distribution_rate_limit.get() * dt;
	float distribution_change = _filter_state.filtered_distribution - _filter_state.previous_distribution;
	distribution_change = math::constrain(distribution_change, -rate_limit, rate_limit);

	_torque_distribution.final_distribution = _filter_state.previous_distribution + distribution_change;
	_filter_state.previous_distribution = _torque_distribution.final_distribution;
}

void LoadAwareTorqueDistribution::apply_safety_limits()
{
	// Constrain to parameter limits
	_torque_distribution.final_distribution = math::constrain(_torque_distribution.final_distribution,
								  _min_front_ratio.get(), _max_front_ratio.get());

	// Update safety state
	_safety_state.torque_distribution_valid = true;
	_safety_state.stability_ok = (_performance.stability_margin > STABILITY_MARGIN_MIN);
}

void LoadAwareTorqueDistribution::check_stability_margins()
{
	// Calculate stability margin based on weight distribution and dynamics
	float weight_distribution_error = fabsf(_torque_distribution.final_distribution - 0.5f);
	float dynamic_factor = fabsf(_load_state.dynamic_load_transfer);

	_performance.stability_margin = 1.0f - (weight_distribution_error + dynamic_factor);
	_performance.stability_margin = math::constrain(_performance.stability_margin, 0.0f, 1.0f);
}

void LoadAwareTorqueDistribution::optimize_for_efficiency()
{
	// Optimize for fuel efficiency by favoring rear-wheel drive when possible
	float efficiency_weight = _efficiency_weight.get();
	float efficiency_bias = -0.1f * efficiency_weight;  // Slight rear bias for efficiency

	_torque_distribution.final_distribution += efficiency_bias;
	_performance.efficiency_factor = 1.0f + (efficiency_bias * 0.1f);
}

void LoadAwareTorqueDistribution::optimize_for_traction()
{
	// Optimize for maximum traction utilization
	float traction_weight = _traction_weight.get();
	float avg_slip = (_traction_state.slip_front + _traction_state.slip_rear) / 2.0f;

	// Apply traction weight to optimization
	_performance.traction_utilization = math::constrain(1.0f - avg_slip * traction_weight, 0.0f, 1.0f);
}

void LoadAwareTorqueDistribution::balance_axle_loading()
{
	// Balance loading between axles for optimal tire wear
	_torque_distribution.front_torque_ratio = _torque_distribution.final_distribution;
	_torque_distribution.rear_torque_ratio = 1.0f - _torque_distribution.final_distribution;
}

void LoadAwareTorqueDistribution::update_performance_metrics()
{
	// Update performance counters
	static float prev_distribution = 0.5f;
	if (fabsf(_torque_distribution.final_distribution - prev_distribution) > 0.01f) {
		_performance.distribution_changes++;
	}
	prev_distribution = _torque_distribution.final_distribution;

	// Update slip averages
	_performance.avg_front_slip = 0.9f * _performance.avg_front_slip + 0.1f * _traction_state.slip_front;
	_performance.avg_rear_slip = 0.9f * _performance.avg_rear_slip + 0.1f * _traction_state.slip_rear;
}

void LoadAwareTorqueDistribution::detect_load_anomalies()
{
	// Detect sudden load changes or instabilities
	static float prev_load = 0.0f;
	float load_change_rate = fabsf(_load_state.payload_mass_kg - prev_load);

	if (load_change_rate > 100.0f) {  // kg/cycle
		_load_state.load_stable = false;
	} else {
		_load_state.load_stable = true;
	}

	prev_load = _load_state.payload_mass_kg;
}

void LoadAwareTorqueDistribution::publish_torque_commands()
{
	// Update torque output message
	_torque_output.timestamp = hrt_absolute_time();
	_torque_output.load_aware_active = true;
	_torque_output.bucket_load_kg = _load_state.payload_mass_kg;
	_torque_output.total_vehicle_mass_kg = _load_state.total_mass_kg;
	_torque_output.center_of_gravity_x = _load_state.cog_x_m;
	_torque_output.center_of_gravity_z = _load_state.cog_z_m;
	_torque_output.front_axle_load_n = _load_state.front_axle_load_kg * 9.81f;
	_torque_output.rear_axle_load_n = _load_state.rear_axle_load_kg * 9.81f;
	_torque_output.weight_distribution = (_torque_distribution.final_distribution - 0.5f) * 2.0f;
	_torque_output.articulation_angle_rad = 0.0f; // TODO: Get from chassis articulation sensor
	_torque_output.optimal_torque_split = _torque_distribution.final_distribution;
	_torque_output.traction_limit_front = _performance.traction_utilization * _torque_distribution.final_distribution;
	_torque_output.traction_limit_rear = _performance.traction_utilization * (1.0f - _torque_distribution.final_distribution);
	_torque_output.stability_margin = _performance.stability_margin;
	_torque_output.stability_warning = (_performance.stability_margin < STABILITY_MARGIN_MIN);
	_torque_output.load_shift_detected = !_load_state.load_stable;
	_torque_output.load_estimation_mode = 1;  // Dynamic estimation
	_torque_output.load_estimation_confidence = _load_state.load_data_valid ? 0.8f : 0.3f;

	// Publish the message
	_load_aware_torque_pub.publish(_torque_output);

	// Update module status
	_status.timestamp = hrt_absolute_time();
	_module_status_pub.publish(_status);
}

void LoadAwareTorqueDistribution::process_chassis_trajectory()
{
	// Check for new chassis trajectory setpoint
	if (_chassis_trajectory_sub.updated()) {
		chassis_trajectory_setpoint_s trajectory;
		_chassis_trajectory_sub.copy(&trajectory);

		if (trajectory.valid) {
			// Update trajectory state
			_trajectory_state.target_x_velocity = trajectory.x_velocity;
			_trajectory_state.target_y_velocity = trajectory.y_velocity;
			_trajectory_state.target_yaw_rate = trajectory.yaw_rate;
			_trajectory_state.trajectory_valid = true;
			_trajectory_state.last_trajectory_time = hrt_absolute_time();

			// Predict future accelerations based on velocity changes
			predict_load_transfer_from_trajectory();

			// Calculate trajectory curvature for stability assessment
			if (fabsf(trajectory.x_velocity) > 0.1f) {
				_trajectory_state.trajectory_curvature = trajectory.yaw_rate / trajectory.x_velocity;
			} else {
				_trajectory_state.trajectory_curvature = 0.0f;
			}

			// Calculate stability demand based on trajectory aggressiveness
			float velocity_magnitude = sqrtf(trajectory.x_velocity * trajectory.x_velocity +
											trajectory.y_velocity * trajectory.y_velocity);
			float lateral_acceleration_demand = fabsf(_trajectory_state.trajectory_curvature *
													velocity_magnitude * velocity_magnitude);

			_trajectory_state.stability_demand = math::constrain(lateral_acceleration_demand / 5.0f, 0.0f, 1.0f);
		}
	}

	// Check trajectory timeout
	if (hrt_elapsed_time(&_trajectory_state.last_trajectory_time) > 500000) { // 0.5 seconds
		_trajectory_state.trajectory_valid = false;
		_trajectory_state.stability_demand = 0.0f;
		_trajectory_state.load_transfer_prediction = 0.0f;
	}
}

void LoadAwareTorqueDistribution::predict_load_transfer_from_trajectory()
{
	if (!_trajectory_state.trajectory_valid) {
		_trajectory_state.load_transfer_prediction = 0.0f;
		return;
	}

	// Calculate predicted longitudinal acceleration
	float dt = 0.1f; // Prediction horizon in seconds
	_trajectory_state.predicted_accel_x = (_trajectory_state.target_x_velocity - _dynamics_state.vehicle_speed_ms) / dt;

	// Calculate predicted lateral acceleration from yaw rate and velocity
	float velocity_magnitude = sqrtf(_trajectory_state.target_x_velocity * _trajectory_state.target_x_velocity +
									_trajectory_state.target_y_velocity * _trajectory_state.target_y_velocity);
	_trajectory_state.predicted_accel_y = _trajectory_state.target_yaw_rate * velocity_magnitude;

	// Calculate predicted load transfer based on accelerations and vehicle geometry
	float load_transfer_long = (_trajectory_state.predicted_accel_x * _load_state.total_mass_kg * _load_state.cog_z_m) /
							  (9.81f * WHEELBASE_M);

	float load_transfer_lat = fabsf(_trajectory_state.predicted_accel_y * _load_state.total_mass_kg * _load_state.cog_z_m) /
							 (9.81f * WHEELBASE_M);

	// Combine longitudinal and lateral load transfer
	_trajectory_state.load_transfer_prediction = sqrtf(load_transfer_long * load_transfer_long +
													  load_transfer_lat * load_transfer_lat);

	// Limit to reasonable values
	_trajectory_state.load_transfer_prediction = math::constrain(_trajectory_state.load_transfer_prediction,
																-MAX_WEIGHT_TRANSFER_RATIO,
																MAX_WEIGHT_TRANSFER_RATIO);
}

void LoadAwareTorqueDistribution::apply_trajectory_based_distribution()
{
	if (!_trajectory_state.trajectory_valid) {
		return;
	}

	// Adjust distribution based on predicted load transfer
	float trajectory_adjustment = 0.0f;

	// For forward acceleration, shift torque to rear (negative adjustment)
	// For braking, shift torque to front (positive adjustment)
	if (fabsf(_trajectory_state.predicted_accel_x) > 0.5f) {
		trajectory_adjustment = -_trajectory_state.predicted_accel_x * 0.1f; // Scale factor
	}

	// For high yaw rates (turning), adjust for stability
	if (fabsf(_trajectory_state.target_yaw_rate) > 0.2f) {
		// Slight bias toward rear for better stability during turns
		trajectory_adjustment -= fabsf(_trajectory_state.target_yaw_rate) * 0.05f;
	}

	// Apply trajectory-based adjustment with gains
	float trajectory_gain = 0.3f; // Parameter for trajectory influence
	_torque_distribution.final_distribution += trajectory_adjustment * trajectory_gain;

	// Ensure we stay within limits
	_torque_distribution.final_distribution = math::constrain(_torque_distribution.final_distribution,
															 MIN_FRONT_TORQUE_RATIO,
															 MAX_FRONT_TORQUE_RATIO);
}
