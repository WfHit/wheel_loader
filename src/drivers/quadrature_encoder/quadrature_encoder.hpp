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

#pragma once

/**
 * @file quadrature_encoder.hpp
 * @brief High-performance quadrature encoder driver using GPIO interrupts
 *
 * This driver provides direct hardware access for reading quadrature encoders
 * with comprehensive error detection, digital filtering, and real-time performance.
 */

#include <atomic>
#include <cstdint>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_quad_encoder.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <stm32_gpio.h>

using namespace time_literals;

namespace quadrature_encoder
{

/**
 * @brief Encoder configuration structure
 */
struct EncoderConfig {
	uint32_t gpio_a{0};                    ///< GPIO pin for phase A
	uint32_t gpio_b{0};                    ///< GPIO pin for phase B
	uint32_t pulses_per_revolution{1024};  ///< Encoder resolution (PPR)
	bool invert_direction{false};          ///< Invert counting direction
	bool enable_filtering{true};           ///< Enable digital filtering
	uint8_t filter_window{3};              ///< Filter window size
};

/**
 * @brief Encoder state information
 */
struct EncoderState {
	std::atomic<int32_t> position{0};      ///< Current position (counts)
	std::atomic<float> velocity{0.0f};     ///< Current velocity (rad/s)
	std::atomic<float> angle{0.0f};        ///< Cumulative angle (rad)
	std::atomic<bool> healthy{false};      ///< Health status

	// Quadrature state tracking
	std::atomic<uint8_t> quadrature_state{0};  ///< Current quadrature state (2-bit)
	std::atomic<uint32_t> transitions{0};      ///< Total state transitions
	std::atomic<uint32_t> errors{0};           ///< Total errors detected

	// Timing
	std::atomic<uint64_t> last_update{0};      ///< Last update timestamp
	hrt_abstime velocity_timestamp{0};         ///< Timestamp for velocity calculation
	int32_t velocity_position{0};              ///< Position for velocity calculation
};

/**
 * @brief Digital filter for noise reduction
 */
class SignalFilter {
public:
	SignalFilter(uint8_t window_size = 3);

	/**
	 * @brief Update filter with new samples
	 * @return True if output is stable
	 */
	bool update(bool a_state, bool b_state);

	/**
	 * @brief Get filtered outputs
	 */
	bool get_filtered_a() const { return _output_a; }
	bool get_filtered_b() const { return _output_b; }

	/**
	 * @brief Reset filter state
	 */
	void reset();

private:
	static constexpr uint8_t MAX_WINDOW = 5;
	uint8_t _window_size;
	uint8_t _index{0};
	bool _samples_a[MAX_WINDOW]{};
	bool _samples_b[MAX_WINDOW]{};
	bool _output_a{false};
	bool _output_b{false};
	uint8_t _stable_count{0};
};

/**
 * @brief Quadrature Encoder Driver
 *
 * High-performance driver for reading quadrature encoders using GPIO interrupts.
 * Features include:
 * - Multi-instance support (up to 4 encoders)
 * - Digital filtering for noise immunity
 * - Comprehensive error detection
 * - Real-time velocity calculation
 * - Thread-safe operation
 */
class QuadratureEncoder : public ModuleBase<QuadratureEncoder>,
                          public ModuleParams,
                          public px4::ScheduledWorkItem
{
public:
	static constexpr uint8_t MAX_INSTANCES = 4;

	/**
	 * @brief Constructor
	 * @param instance Instance ID (0-3)
	 */
	explicit QuadratureEncoder(uint8_t instance = 0);

	/**
	 * @brief Destructor
	 */
	~QuadratureEncoder() override;

	// Disable copy/move
	QuadratureEncoder(const QuadratureEncoder&) = delete;
	QuadratureEncoder& operator=(const QuadratureEncoder&) = delete;

	/**
	 * @brief Initialize the driver
	 * @return True on success
	 */
	bool init();

	/**
	 * @brief Module entry point
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief Instantiate specific instance
	 */
	static QuadratureEncoder *instantiate(int instance);

	/**
	 * @brief Print usage information
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief Print status information
	 */
	int print_status() override;

	/**
	 * @brief Custom command handler
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @brief Get encoder configuration
	 */
	const EncoderConfig& get_config() const { return _config; }

	/**
	 * @brief Check if encoder is healthy
	 */
	bool is_healthy() const { return _state.healthy.load(); }

	/**
	 * @brief Get current position
	 */
	int32_t get_position() const { return _state.position.load(); }

	/**
	 * @brief Get current velocity
	 */
	float get_velocity() const { return _state.velocity.load(); }

	/**
	 * @brief Reset encoder position
	 */
	void reset_position();

private:
	/**
	 * @brief Main work function
	 */
	void Run() override;

	/**
	 * @brief Initialize hardware
	 */
	bool initialize_hardware();

	/**
	 * @brief Cleanup hardware
	 */
	void cleanup_hardware();

	/**
	 * @brief Attach interrupt handlers
	 */
	bool attach_interrupts();

	/**
	 * @brief Detach interrupt handlers
	 */
	void detach_interrupts();

	/**
	 * @brief Update parameters
	 */
	void update_parameters();

	/**
	 * @brief Process encoder state change
	 */
	void process_state_change(bool a_state, bool b_state);

	/**
	 * @brief Calculate velocity from position changes
	 */
	void calculate_velocity();

	/**
	 * @brief Publish encoder data
	 */
	void publish_data();

	/**
	 * @brief Check encoder health
	 */
	void check_health();

	/**
	 * @brief Static interrupt handler
	 */
	static int interrupt_handler(int irq, void *context, void *arg);

	/**
	 * @brief Get PPR parameter value for instance
	 */
	int32_t get_instance_ppr() const;

	/**
	 * @brief Get invert parameter value for instance
	 */
	bool get_instance_invert() const;

	// Instance identification
	const uint8_t _instance;

	// Configuration and state
	EncoderConfig _config;
	EncoderState _state;
	SignalFilter _filter;

	// Hardware state
	std::atomic<bool> _initialized{false};
	std::atomic<bool> _interrupts_attached{false};

	// Publications
	orb_advert_t _encoder_pub{nullptr};
	int _pub_instance{-1};

	// Subscriptions
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Performance monitoring
	perf_counter_t _cycle_perf;
	perf_counter_t _interrupt_perf;
	perf_counter_t _error_perf;

		// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::QE_UPDATE_RATE>) _param_update_rate,
		(ParamBool<px4::params::QE_FILTER_EN>) _param_filter_enabled,
		(ParamFloat<px4::params::QE_MAX_ERR_RT>) _param_max_error_rate,

		// Per-instance parameters
		(ParamInt<px4::params::QE_PPR_0>) _param_ppr_0,
		(ParamBool<px4::params::QE_INVERT_0>) _param_invert_0,
		(ParamInt<px4::params::QE_PPR_1>) _param_ppr_1,
		(ParamBool<px4::params::QE_INVERT_1>) _param_invert_1,
		(ParamInt<px4::params::QE_PPR_2>) _param_ppr_2,
		(ParamBool<px4::params::QE_INVERT_2>) _param_invert_2,
		(ParamInt<px4::params::QE_PPR_3>) _param_ppr_3,
		(ParamBool<px4::params::QE_INVERT_3>) _param_invert_3
	)

	// Static instance storage
	static QuadratureEncoder *_instances[MAX_INSTANCES];

	// Quadrature state transition table
	static const int8_t _state_table[16];

	// Health monitoring
	static constexpr uint32_t HEALTH_CHECK_INTERVAL_US = 1_s;
	static constexpr float MAX_ERROR_RATE_DEFAULT = 5.0f;
	hrt_abstime _last_health_check{0};
};

} // namespace quadrature_encoder
