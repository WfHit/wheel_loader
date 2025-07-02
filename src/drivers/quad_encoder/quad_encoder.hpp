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

#include <nuttx/sensors/qencoder.h>

using namespace time_literals;

/**
 * @brief GPIO-based quadrature encoder driver
 *
 * This driver interfaces with the NuttX GPIO-based quadrature encoder framework
 * to provide encoder data for rotary encoders. It supports multiple encoder instances
 * and publishes raw encoder data.
 */
class QuadEncoder : public ModuleBase<QuadEncoder>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static constexpr int MAX_INSTANCES = 4; // Support up to 4 module instances

	/**
	 * @brief Constructor
	 * @param instance_id Encoder instance ID (0-based)
	 * @param device_path Path to the encoder device (e.g., "/dev/qe0")
	 */
	QuadEncoder(int instance_id = 0, const char *device_path = nullptr);

	/**
	 * @brief Destructor
	 */
	~QuadEncoder() override;

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
	 * @param device_path Path to encoder device
	 * @return QuadEncoder instance or nullptr
	 */
	static QuadEncoder *instantiate(int instance, const char *device_path = nullptr);

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
	 */
	int32_t get_ppr_for_instance() const;

	/**
	 * @brief Get invert parameter for this instance
	 */
	bool get_invert_for_instance() const;

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
	 * @brief Read encoder data from NuttX driver
	 */
	void read_encoder();

	/**
	 * @brief Reset encoder position
	 */
	void reset_encoder();

	/**
	 * @brief Open encoder device file
	 * @return true on success, false otherwise
	 */
	bool open_encoder();

	/**
	 * @brief Close encoder device file
	 */
	void close_encoder();

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::QE_UPDATE_RATE>) _param_update_rate,
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
	int _instance_id{0};
	bool _is_running{false};
	char _device_path[32]{};  // Store the device path

	// Device file descriptors - now single device per instance
	int _fd_encoder{-1};

	// Encoder data - simplified to single encoder per instance
	struct encoder_data_s {
		int32_t position;
		float velocity_rad_s;     // rad/s
		float angle_rad;          // rad (cumulative angle)
		uint64_t timestamp;
		bool valid;
		int32_t pulses_per_rev;   // PPR
		bool invert_direction;
	};
	encoder_data_s _encoder_data{};

	// Previous encoder position for velocity calculation
	int32_t _prev_position{0};
	hrt_abstime _prev_timestamp{0};

	// Publishers - use orb_advert_t for multi-instance
	orb_advert_t _sensor_quad_encoder_pub{nullptr};

	// Instance-specific topic instances
	int _sensor_encoder_instance{-1};

	// Subscribers
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _read_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};

	// Statistics
	uint32_t _error_count{0};
	uint32_t _read_count{0};
	hrt_abstime _last_error_time{0};

	// Configuration
	static constexpr uint32_t SCHEDULE_INTERVAL{10_ms}; // 100 Hz default
};
