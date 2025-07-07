#pragma once

// System includes
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_arch/quad_encoder.h>
#include <px4_arch/board_encoder.h>

// Library includes
#include <lib/perf/perf_counter.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_quad_encoder.h>

// Standard includes
#include <stdint.h>

// Using declarations
using namespace time_literals;

// Encoder operating modes (moved to app level)
enum encoder_mode_t {
	ENCODER_MODE_RELATIVE = 0,      // Relative positioning mode
	ENCODER_MODE_ABSOLUTE = 1       // Absolute positioning mode
};

/**
 * @brief GPIO-based quadrature encoder driver
 *
 * This driver provides a universal interface for GPIO-based quadrature encoders
 * with support for multiple instances, configurable modes, and high-precision
 * position and velocity measurements.
 */
class QuadratureEncoder : public ModuleBase<QuadratureEncoder>,
                          public ModuleParams,
                          public px4::ScheduledWorkItem
{
public:
	/**
	 * @brief Constructor
	 *
	 * @param encoder_id Encoder instance ID
	 */
	QuadratureEncoder(uint8_t encoder_id);

	/**
	 * @brief Destructor
	 */
	~QuadratureEncoder() override;

	/**
	 * @brief Initialize the encoder
	 *
	 * @return true on success, false on failure
	 */
	bool init();

	/**
	 * @brief Get driver info
	 *
	 * @return Driver info string
	 */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief Custom command handler
	 *
	 * @param argc Argument count
	 * @param argv Arguments
	 * @return 0 on success, negative on error
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @brief Get instance by ID
	 *
	 * @param encoder_id Encoder ID
	 * @return Encoder instance or nullptr
	 */
	static QuadratureEncoder *get_instance(uint8_t encoder_id);

	/**
	 * @brief Start encoder instance
	 *
	 * @param encoder_id Encoder ID
	 * @return 0 on success, negative on error
	 */
	static int start_instance(uint8_t encoder_id);

	/**
	 * @brief Stop encoder instance
	 *
	 * @param encoder_id Encoder ID
	 * @return 0 on success, negative on error
	 */
	static int stop_instance(uint8_t encoder_id);

protected:
	/**
	 * @brief Scheduled work item execution
	 */
	void Run() override;

private:
	// Private methods
	void publish_encoder_data();
	void update_params();
	void process_raw_encoder_data(const encoder_raw_data_t &raw_data);

	// Parameter helper methods
	void init_instance_parameters();
	int32_t get_param_int(param_t param_handle, int32_t default_value = 0);
	float get_param_float(param_t param_handle, float default_value = 0.0f);

	// Static instance management
	static QuadratureEncoder *_instances[ENCODER_MAX_INSTANCES];
	static uint8_t _instance_count;

	// Instance identification
	uint8_t _encoder_id;
	bool _initialized;
	bool _running;

	// Configuration
	encoder_mode_t _mode;

	// Instance-specific parameters (constructed dynamically)
	param_t _param_pulses_per_revolution_handle;
	param_t _param_mode_handle;
	param_t _param_vel_filter_handle;
	param_t _param_rate_handle;

	// Current state (updated from processed data)
	int64_t _position_raw;
	float _position_rad;
	float _velocity_rad_s;
	bool _direction_forward;

	// Counters and timing
	uint64_t _pulse_count;
	uint32_t _reset_count;
	uint64_t _last_update_time;

	// uORB publishing
	uORB::Publication<sensor_quad_encoder_s> _encoder_pub;

	// Performance monitoring
	perf_counter_t _cycle_perf;
	perf_counter_t _interval_perf;
	perf_counter_t _error_perf;
};
