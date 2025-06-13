#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <matrix/matrix.hpp>

// uORB message includes
#include <uORB/topics/wheel_speeds_setpoint.h>
#include <uORB/topics/load_aware_torque.h>
#include <uORB/topics/traction_control.h>
#include <uORB/topics/module_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_status.h>

using namespace time_literals;
using namespace matrix;

/**
 * @brief Load-Aware Torque Distribution for articulated wheel loader
 *
 * Dynamically distributes torque between front and rear axles based on:
 * - Payload mass and center of gravity
 * - Dynamic weight transfer during acceleration/braking
 * - Terrain conditions and stability requirements
 * - Real-time axle load measurements
 */
class LoadAwareTorqueDistribution : public ModuleBase<LoadAwareTorqueDistribution>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	LoadAwareTorqueDistribution();
	~LoadAwareTorqueDistribution() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static LoadAwareTorqueDistribution *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	static constexpr float CONTROL_RATE_HZ = 50.0f;
	static constexpr uint64_t CONTROL_INTERVAL_US = 1_s / CONTROL_RATE_HZ;

	// Vehicle configuration
	static constexpr float EMPTY_MASS_KG = 12000.0f;
	static constexpr float MAX_PAYLOAD_KG = 8000.0f;
	static constexpr float WHEELBASE_M = 2.5f;
	static constexpr float EMPTY_COG_X_M = 0.2f;  // COG ahead of center when empty
	static constexpr float EMPTY_COG_Z_M = 1.2f;  // COG height when empty

	// Torque distribution limits
	static constexpr float MIN_FRONT_TORQUE_RATIO = 0.2f;
	static constexpr float MAX_FRONT_TORQUE_RATIO = 0.8f;
	static constexpr float DISTRIBUTION_FILTER_TC = 0.1f;  // Time constant for filtering

	// Stability thresholds
	static constexpr float MAX_WEIGHT_TRANSFER_RATIO = 0.3f;
	static constexpr float STABILITY_MARGIN_MIN = 0.1f;

	void Run() override;

	/**
	 * Load analysis functions
	 */
	void update_load_state();
	void calculate_static_weight_distribution();
	void calculate_dynamic_weight_transfer();
	void estimate_center_of_gravity();

	/**
	 * Torque distribution calculation
	 */
	void calculate_base_torque_distribution();
	void apply_dynamic_adjustments();
	void apply_stability_corrections();
	void apply_terrain_adaptations();

	/**
	 * Safety and validation
	 */
	void validate_torque_distribution();
	void check_stability_margins();
	void apply_safety_limits();

	/**
	 * Performance optimization
	 */
	void optimize_for_efficiency();
	void optimize_for_traction();
	void balance_axle_loading();

	/**
	 * Monitoring and diagnostics
	 */
	void update_performance_metrics();
	void detect_load_anomalies();
	void publish_torque_commands();

	// uORB subscriptions
	uORB::Subscription _wheel_speeds_sub{ORB_ID(wheel_speeds_setpoint)};
	uORB::Subscription _traction_control_sub{ORB_ID(traction_control)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
	uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};

	// uORB publications
	uORB::Publication<load_aware_torque_s> _load_aware_torque_pub{ORB_ID(load_aware_torque)};
	uORB::Publication<wheel_speeds_setpoint_s> _wheel_speeds_pub{ORB_ID(wheel_speeds_setpoint)};
	uORB::Publication<module_status_s> _module_status_pub{ORB_ID(module_status)};

	// Load state
	struct LoadState {
		float payload_mass_kg{0.0f};
		float total_mass_kg{EMPTY_MASS_KG};
		float cog_x_m{EMPTY_COG_X_M};      // Center of gravity X position
		float cog_z_m{EMPTY_COG_Z_M};      // Center of gravity height
		float front_axle_load_kg{6000.0f}; // Static front axle load
		float rear_axle_load_kg{6000.0f};  // Static rear axle load
		float dynamic_load_transfer{0.0f}; // Dynamic weight transfer ratio
		uint8_t load_state{0};             // 0: empty, 1: loading, 2: loaded, 3: dumping
		bool load_stable{true};
		bool load_data_valid{false};
		uint64_t last_update_time{0};
	} _load_state;

	// Vehicle dynamics state
	struct DynamicsState {
		float longitudinal_accel_ms2{0.0f};
		float lateral_accel_ms2{0.0f};
		float vehicle_speed_ms{0.0f};
		float pitch_angle_rad{0.0f};
		float roll_angle_rad{0.0f};
		float slope_angle_rad{0.0f};
		float yaw_rate_rad_s{0.0f};
		bool dynamics_valid{false};
	} _dynamics_state;

	// Traction state
	struct TractionState {
		float slip_front{0.0f};
		float slip_rear{0.0f};
		float friction_coefficient{0.6f};
		float surface_roughness{0.0f};
		uint8_t surface_type{0};
		bool traction_control_active{false};
		float traction_limit_factor{1.0f};
	} _traction_state;

	// Torque distribution output
	struct TorqueDistribution {
		float base_distribution{0.5f};        // Static distribution (0=rear, 1=front)
		float dynamic_adjustment{0.0f};       // Dynamic adjustment
		float stability_correction{0.0f};     // Stability correction
		float terrain_adaptation{0.0f};       // Terrain-based adjustment
		float final_distribution{0.5f};       // Final distribution ratio
		float front_torque_ratio{0.5f};       // Front axle torque ratio
		float rear_torque_ratio{0.5f};        // Rear axle torque ratio
		float front_torque_nm{0.0f};          // Commanded front torque
		float rear_torque_nm{0.0f};           // Commanded rear torque
		float total_torque_request_nm{0.0f};  // Total requested torque
	} _torque_distribution;

	// Performance metrics
	struct PerformanceMetrics {
		float efficiency_factor{1.0f};        // Fuel efficiency factor
		float traction_utilization{0.5f};     // Average traction utilization
		float stability_margin{1.0f};         // Stability margin (0-1)
		float weight_distribution_error{0.0f}; // Error from optimal distribution
		uint32_t distribution_changes{0};     // Number of distribution changes
		uint32_t stability_interventions{0};  // Stability interventions count
		float avg_front_slip{0.0f};           // Average front slip
		float avg_rear_slip{0.0f};            // Average rear slip
		float power_distribution_efficiency{1.0f}; // Power distribution efficiency
	} _performance;

	// Filtering and smoothing
	struct FilterState {
		float filtered_distribution{0.5f};
		float distribution_rate_limit{0.0f};
		float previous_distribution{0.5f};
		uint64_t last_filter_time{0};
	} _filter_state;

	// Safety monitoring
	struct SafetyState {
		bool torque_distribution_valid{true};
		bool load_measurement_valid{true};
		bool stability_ok{true};
		bool emergency_distribution_active{false};
		float safety_factor{1.0f};
		uint32_t safety_violations{0};
	} _safety_state;

	// Module status
	module_status_s _status{};
	load_aware_torque_s _torque_output{};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::LAT_MIN_FRONT_RATIO>) _min_front_ratio,
		(ParamFloat<px4::params::LAT_MAX_FRONT_RATIO>) _max_front_ratio,
		(ParamFloat<px4::params::LAT_FILTER_TC>) _distribution_filter_tc,
		(ParamFloat<px4::params::LAT_RATE_LIMIT>) _distribution_rate_limit,
		(ParamFloat<px4::params::LAT_STABILITY_GAIN>) _stability_gain,
		(ParamFloat<px4::params::LAT_DYNAMIC_GAIN>) _dynamic_gain,
		(ParamFloat<px4::params::LAT_TERRAIN_GAIN>) _terrain_gain,
		(ParamFloat<px4::params::LAT_EFFICIENCY_WEIGHT>) _efficiency_weight,
		(ParamFloat<px4::params::LAT_TRACTION_WEIGHT>) _traction_weight,
		(ParamFloat<px4::params::LAT_COG_X_OFFSET>) _cog_x_offset,
		(ParamFloat<px4::params::LAT_COG_Z_OFFSET>) _cog_z_offset,
		(ParamBool<px4::params::LAT_ADAPTIVE_EN>) _adaptive_enable,
		(ParamBool<px4::params::LAT_EFFICIENCY_OPT>) _efficiency_optimize
	)
};
