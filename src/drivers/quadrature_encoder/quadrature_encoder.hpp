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
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/parameter_update.h>

// Standard includes
#include <stdint.h>

// Using declarations
using namespace time_literals;

// Configuration constants
#define SCHEDULE_INTERVAL 20000  // 50 Hz default

// Encoder operating modes
enum encoder_mode_t {
	ENCODER_MODE_RELATIVE = 0,      // Relative positioning mode
	ENCODER_MODE_ABSOLUTE = 1       // Absolute positioning mode
};

/**
 * @brief GPIO-based quadrature encoder driver
 *
 * This driver provides a universal interface for GPIO-based quadrature encoders
 * with support for multiple encoder channels managed by a single instance.
 */
class QuadratureEncoder : public ModuleBase<QuadratureEncoder>,
                          public ModuleParams,
                          public px4::ScheduledWorkItem
{
public:
	/**
	 * @brief Constructor
	 */
	QuadratureEncoder();

	/**
	 * @brief Destructor
	 */
	~QuadratureEncoder() override;

	/**
	 * @brief Initialize the encoder driver
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

	/** @see ModuleBase */
	int print_status();

protected:
	/**
	 * @brief Scheduled work item execution
	 */
	void Run() override;

private:
	struct EncoderChannel {
		bool initialized{false};
		bool enabled{false};
		encoder_mode_t mode{ENCODER_MODE_RELATIVE};

		// State variables
		int64_t position_raw{0};
		float position_rad{0.0f};
		float velocity_rad_s{0.0f};
		bool direction_forward{true};

		// Counters and timing
		uint64_t pulse_count{0};
		uint32_t reset_count{0};
		uint64_t last_update_time{0};

		// Parameters
		param_t param_pulses_per_revolution{PARAM_INVALID};
		param_t param_mode{PARAM_INVALID};
		param_t param_vel_filter{PARAM_INVALID};
		param_t param_enabled{PARAM_INVALID};
	};

	// Private methods
	bool configure_encoder(int encoder_id);
	void process_encoders();
	void process_encoder_data(int encoder_id, const encoder_raw_data_t &raw_data);
	void publish_encoder_data();
	void parameters_update();
	void init_encoder_parameters(int encoder_id);

	// Parameter helper methods
	int32_t get_param_int(param_t param_handle, int32_t default_value = 0) const;
	float get_param_float(param_t param_handle, float default_value = 0.0f) const;

	// Encoder channels
	EncoderChannel _encoders[sensor_quad_encoder_s::MAX_ENCODERS];
	int _num_encoders{0};
	bool _is_running{false};

	// uORB subscriptions and publications
	uORB::Publication<sensor_quad_encoder_s> _encoder_pub{ORB_ID(sensor_quad_encoder)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Performance monitoring
	perf_counter_t _loop_perf{nullptr};
	perf_counter_t _encoder_perf{nullptr};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::QENC_RATE>) _param_rate
	)
};
