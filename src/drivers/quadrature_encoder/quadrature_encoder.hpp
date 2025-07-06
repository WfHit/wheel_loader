#pragma once

// System includes
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/encoder_hw.h>
#include <px4_platform_common/board_encoder_interface.h>

// Library includes
#include <lib/perf/perf_counter.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_quad_encoder.h>

// Using declarations
using namespace time_literals;

// Module name
#define MODULE_NAME "quadrature_encoder"

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

	/** @see ModuleBase */
	static QuadratureEncoder *instantiate(int argc, char *argv[]);

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
	// Static instance management
	static QuadratureEncoder *_instances[ENCODER_MAX_INSTANCES];
	static uint8_t _instance_count;

	// Instance identification
	uint8_t _encoder_id;
	uint8_t _platform_encoder_id;
	encoder_hw_config_t _config;
	bool _initialized;
	bool _running;

	// Current state (updated from processed data)
	int64_t _position_raw;
	int64_t _position_offset;
	float _position_rad;
	float _position_deg;
	float _velocity_rad_s;
	float _velocity_rpm;
	bool _direction_forward;
	bool _index_detected_this_revolution;

	// Counters and timing
	uint64_t _pulse_count;
	uint32_t _direction_changes;
	uint32_t _error_count;
	uint64_t _last_update_time;

	// uORB publishing
	uORB::Publication<sensor_quad_encoder_s> _encoder_pub;

	// Performance monitoring
	perf_counter_t _cycle_perf;
	perf_counter_t _interval_perf;
	perf_counter_t _error_perf;

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::QENC_ENABLE>) _param_enable,
		(ParamInt<px4::params::QENC_PPR>) _param_pulses_per_revolution,
		(ParamInt<px4::params::QENC_MODE>) _param_mode,
		(ParamInt<px4::params::QENC_SWAP_CH>) _param_swap_channels,
		(ParamFloat<px4::params::QENC_POS_OFF>) _param_position_offset,
		(ParamFloat<px4::params::QENC_VEL_FILT>) _param_vel_filter,
		(ParamInt<px4::params::QENC_RATE>) _param_rate,
		(ParamInt<px4::params::QENC_DEBUG>) _param_debug
	)

	// Methods
	void publish_encoder_data();
	void update_params();
};
