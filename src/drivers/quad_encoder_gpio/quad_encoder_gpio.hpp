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

#include <atomic>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/drivers/device/device.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_quad_encoder.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <chip.h>
#include <stm32_gpio.h>

using namespace time_literals;

/*
 * GPIO-based Quadrature Encoder Driver
 *
 * This driver provides direct GPIO interrupt-based quadrature encoder reading
 * without relying on STM32 timer hardware. It supports:
 * - Multiple encoder instances with parameter-based configuration
 * - Digital filtering for noise reduction
 * - Comprehensive error detection and health monitoring
 * - High-resolution 4x counting mode
 * - Real-time velocity calculation
 * - Thread-safe operation using atomic variables
 *
 * Configuration is handled through:
 * - Board-specific configs (board_quad_encoder_gpio_config.cpp)
 * - Runtime parameters (QE_PPR_x, QE_INVERT_x, etc.)
 * - Dynamic parameter updates via uORB
 */

/**
 * @brief GPIO-based quad encoder configuration structure
 */
struct quad_encoder_gpio_config_s
{
	uint32_t gpio_a;           /**< GPIO pin for phase A */
	uint32_t gpio_b;           /**< GPIO pin for phase B */
	uint32_t resolution;       /**< Pulses per revolution (CPR) */
	bool invert_direction;     /**< Invert counting direction */
	bool use_index;           /**< Use index signal (if available) */
	bool x4_mode;             /**< Use 4x counting mode */
	uint32_t max_frequency;   /**< Maximum expected frequency in Hz */
	uint32_t filter_samples;  /**< Number of samples for digital filter */
};

/**
 * @brief Encoder error types for diagnostics
 */
enum class EncoderError : uint8_t {
	NONE = 0,
	INVALID_TRANSITION,
	MISSED_PULSE,
	HIGH_ERROR_RATE,
	GPIO_FAILURE,
	INTERRUPT_FAILURE
};

/**
 * @brief GPIO Interrupt-based Quadrature Encoder Driver
 *
 * This driver directly uses GPIO interrupts to decode quadrature encoder signals
 * without relying on NuttX timer drivers. It provides direct hardware access
 * for STM32H7 and avoids syslog issues.
 */
class QuadEncoderGPIO : public ModuleBase<QuadEncoderGPIO>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static constexpr int MAX_INSTANCES = 4; // Maximum number of encoder instances

	/**
	 * @brief Constructor
	 * @param instance_id Encoder instance ID (0-based)
	 */
	QuadEncoderGPIO(int instance_id = 0);

	/**
	 * @brief Destructor
	 */
	~QuadEncoderGPIO() override;

	/**
	 * @brief Module entry point
	 * @param argc Argument count
	 * @param argv Argument vector
	 * @return 0 on success, error code otherwise
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief Instantiate specific instance
	 * @param instance Instance ID
	 * @return QuadEncoderGPIO instance or nullptr
	 */
	static QuadEncoderGPIO *instantiate(int instance);

	/**
	 * @brief Print module usage information
	 */
	static void usage(const char *reason = nullptr);

	/**
	 * @brief Print module usage information
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief Initialize the driver
	 * @return true on success, false otherwise
	 */
	bool init();

	/**
	 * @brief Print driver status information
	 */
	int print_status() override;

	/**
	 * @brief Custom command handler
	 * @param argc Argument count
	 * @param argv Argument vector
	 * @return 0 on success, error code otherwise
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @brief Get PPR parameter for this instance
	 * @return Pulses per revolution for this encoder instance
	 */
	int32_t get_ppr_for_instance() const;

	/**
	 * @brief Get invert parameter for this instance
	 * @return True if direction is inverted for this instance
	 */
	bool get_invert_for_instance() const;

	/**
	 * @brief Get encoder configuration for this instance
	 * @return Encoder configuration structure
	 */
	const quad_encoder_gpio_config_s* get_config_for_instance() const;

	/**
	 * @brief Validate encoder configuration
	 * @param config Configuration to validate
	 * @return True if configuration is valid
	 */
	static bool validate_config(const quad_encoder_gpio_config_s* config);

	/**
	 * @brief Check encoder health status
	 * @return True if encoder is healthy
	 */
	bool is_encoder_healthy() const;

	/**
	 * @brief GPIO interrupt handler (static)
	 * @param irq IRQ number
	 * @param context Context
	 * @param arg Argument (this pointer)
	 * @return IRQ handled status
	 */
	static int gpio_interrupt_handler(int irq, void *context, void *arg);

private:
	/**
	 * @brief Main work loop - scheduled execution
	 */
	void Run() override;

	/**
	 * @brief Update parameters from parameter system
	 */
	void parameters_update();

	/**
	 * @brief Initialize GPIO pins for encoder
	 * @return true on success, false otherwise
	 */
	bool init_gpio();

	/**
	 * @brief Deinitialize GPIO pins
	 */
	void deinit_gpio();

	/**
	 * @brief Attach GPIO interrupt handlers
	 * @return true on success, false otherwise
	 */
	bool attach_interrupts();

	/**
	 * @brief Detach GPIO interrupt handlers
	 */
	void detach_interrupts();

	/**
	 * @brief Process encoder state change with filtering
	 * @param a_state Current state of A signal
	 * @param b_state Current state of B signal
	 */
	void process_encoder_state(bool a_state, bool b_state);

	/**
	 * @brief Apply digital filtering to encoder signals
	 * @param a_state Raw A signal state
	 * @param b_state Raw B signal state
	 * @return True if filtered signals are stable
	 */
	bool apply_signal_filter(bool a_state, bool b_state);

	/**
	 * @brief Validate quadrature state transition
	 * @param old_state Previous state
	 * @param new_state Current state
	 * @return True if transition is valid
	 */
	bool validate_state_transition(uint8_t old_state, uint8_t new_state) const;

	/**
	 * @brief Update encoder diagnostics
	 * @param error_type Type of error encountered
	 */
	void update_diagnostics(EncoderError error_type);

	/**
	 * @brief Check and handle encoder errors
	 */
	void check_encoder_health();

	/**
	 * @brief Reset encoder position
	 */
	void reset_encoder();

	/**
	 * @brief Calculate velocity from position change
	 * @param current_position Current encoder position
	 * @param current_time Current timestamp
	 */
	void calculate_velocity(int32_t current_position, hrt_abstime current_time);

	/**
	 * @brief Read GPIO pin state with error handling
	 * @param gpio_pin GPIO pin to read
	 * @return true if pin is high, false if low
	 */
	bool read_gpio_pin(uint32_t gpio_pin) const;

	/**
	 * @brief Get GPIO pin name for debugging
	 * @param gpio_pin GPIO pin to get name for
	 * @return Human-readable pin name
	 */
	const char* get_gpio_pin_name(uint32_t gpio_pin) const;

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::QE_UPDATE_RATE>) _param_update_rate,
		(ParamBool<px4::params::QE_FILTER_EN>) _param_filter_en,
		(ParamFloat<px4::params::QE_MAX_ERR_RATE>) _param_max_err_rate,
		(ParamInt<px4::params::QE_PPR_0>) _param_ppr_0,
		(ParamBool<px4::params::QE_INVERT_0>) _param_invert_0,
		(ParamInt<px4::params::QE_PPR_1>) _param_ppr_1,
		(ParamBool<px4::params::QE_INVERT_1>) _param_invert_1,
		(ParamInt<px4::params::QE_PPR_2>) _param_ppr_2,
		(ParamBool<px4::params::QE_INVERT_2>) _param_invert_2,
		(ParamInt<px4::params::QE_PPR_3>) _param_ppr_3,
		(ParamBool<px4::params::QE_INVERT_3>) _param_invert_3
	)

	// Instance configuration
	const int _instance_id{0};
	std::atomic<bool> _is_running{false};

	// Encoder state tracking (protected by atomic operations for thread safety)
	struct encoder_state_s {
		std::atomic<int32_t> position{0};
		std::atomic<float> velocity_rad_s{0.0f};     // rad/s
		std::atomic<float> angle_rad{0.0f};          // rad (cumulative angle)
		std::atomic<uint64_t> timestamp{0};
		std::atomic<bool> valid{false};
		int32_t pulses_per_rev{1024};               // PPR (configuration - not frequently changed)
		bool invert_direction{false};               // Direction setting

		// Quadrature state machine
		std::atomic<uint8_t> last_state{0};         // Last quadrature state (0-3)
		std::atomic<uint8_t> current_state{0};      // Current quadrature state (0-3)
		std::atomic<int32_t> position_change{0};    // Position change since last update

		// Error tracking and diagnostics
		std::atomic<uint32_t> error_count{0};       // Quadrature errors detected
		std::atomic<uint32_t> index_count{0};       // Number of index pulses (if supported)
		std::atomic<int32_t> last_index_position{0}; // Position at last index
		std::atomic<bool> index_found{false};       // Index pulse detected
		std::atomic<uint32_t> sample_rate{0};       // Actual sampling rate
		std::atomic<uint32_t> total_transitions{0}; // Total state transitions
		hrt_abstime last_error_time{0};             // Time of last error
		EncoderError last_error_type{EncoderError::NONE}; // Last error type
	};
	encoder_state_s _encoder_state{};

	// Signal filtering for noise reduction (if enabled)
	struct signal_filter_s {
		static constexpr int FILTER_SIZE = 3;
		bool a_samples[FILTER_SIZE]{};
		bool b_samples[FILTER_SIZE]{};
		int sample_index{0};
		int stable_count{0};
		bool last_filtered_a{false};
		bool last_filtered_b{false};
	};
	signal_filter_s _signal_filter{};

	// Previous encoder position for velocity calculation
	std::atomic<int32_t> _prev_position{0};
	std::atomic<hrt_abstime> _prev_timestamp{0};

	// Interrupt handling
	irqstate_t _irq_state{};
	std::atomic<bool> _interrupts_attached{false};

	// Publications
	orb_advert_t _sensor_quad_encoder_pub{nullptr};
	int _sensor_encoder_instance{-1};

	// Subscriptions
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, "quad_encoder_gpio: cycle")};
	perf_counter_t _interrupt_perf{perf_alloc(PC_COUNT, "quad_encoder_gpio: interrupts")};
	perf_counter_t _error_perf{perf_alloc(PC_COUNT, "quad_encoder_gpio: errors")};

	// Statistics and diagnostics
	std::atomic<uint32_t> _error_count{0};
	std::atomic<uint32_t> _interrupt_count{0};
	std::atomic<hrt_abstime> _last_error_time{0};
	std::atomic<uint32_t> _consecutive_errors{0};
	std::atomic<float> _error_rate{0.0f};

	// Timing and configuration (use parameters instead of constants)
	static constexpr uint32_t MAX_INSTANCES_GPIO{4}; // Maximum number of encoder instances
	static constexpr uint32_t DEFAULT_SCHEDULE_INTERVAL{10}; // 10ms = 100 Hz default
	static constexpr uint32_t MAX_CONSECUTIVE_ERRORS{10}; // Maximum consecutive errors before declaring failure
	static constexpr uint32_t ERROR_RATE_WINDOW{1000};   // Window for error rate calculation (ms)
	static constexpr uint32_t HEALTH_CHECK_INTERVAL{5000}; // Health check interval (ms)

	// Last health check time
	hrt_abstime _last_health_check{0};

	// External declarations for board-specific configurations
	extern const struct quad_encoder_gpio_config_s board_quad_encoder_gpio_configs[];
	extern const unsigned int board_quad_encoder_gpio_count;

	// Static storage for multiple instances
	extern QuadEncoderGPIO *_objects_gpio[MAX_INSTANCES_GPIO];
};
