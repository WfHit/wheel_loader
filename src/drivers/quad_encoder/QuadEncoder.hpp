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
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/vehicle_odometry.h>

#include <nuttx/sensors/qencoder.h>

using namespace time_literals;

/**
 * @brief GPIO-based quadrature encoder driver
 *
 * This driver interfaces with the NuttX GPIO-based quadrature encoder framework
 * to provide encoder data for various applications including wheels, rotary actuators,
 * linear actuators, and other position sensors. It supports multiple encoder instances
 * and publishes raw encoder data. Optional odometry calculation can be enabled for
 * wheel applications.
 */
class QuadEncoder : public ModuleBase<QuadEncoder>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	/**
	 * @brief Constructor
	 * @param instance_id Encoder instance ID (0-based)
	 */
	QuadEncoder(int instance_id = 0);

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
	 * @brief Print module usage information
	 */
	static void usage(const char *reason = nullptr);

	/**
	 * @brief Initialize the driver
	 * @return true on success, false otherwise
	 */
	bool init();

	/**
	 * @brief Print driver status information
	 */
	void print_status() override;

	/**
	 * @brief Custom command handler
	 * @param argc Argument count
	 * @param argv Argument vector
	 * @return 0 on success, error code otherwise
	 */
	static int custom_command(int argc, char *argv[]);

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
	void read_encoders();

	/**
	 * @brief Calculate and publish odometry
	 */
	void calculate_odometry();

	/**
	 * @brief Reset encoder positions and odometry
	 */
	void reset_encoders();

	/**
	 * @brief Open encoder device files
	 * @return true on success, false otherwise
	 */
	bool open_encoders();

	/**
	 * @brief Close encoder device files
	 */
	void close_encoders();

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::QE_UPDATE_RATE>) _param_update_rate,
		(ParamInt<px4::params::QE_NUM_ENCODERS>) _param_num_encoders,
		(ParamInt<px4::params::QE_PPR_0>) _param_ppr_0,
		(ParamInt<px4::params::QE_PPR_1>) _param_ppr_1,
		(ParamInt<px4::params::QE_PPR_2>) _param_ppr_2,
		(ParamInt<px4::params::QE_PPR_3>) _param_ppr_3,
		(ParamBool<px4::params::QE_INVERT_0>) _param_invert_0,
		(ParamBool<px4::params::QE_INVERT_1>) _param_invert_1,
		(ParamBool<px4::params::QE_INVERT_2>) _param_invert_2,
		(ParamBool<px4::params::QE_INVERT_3>) _param_invert_3,
		(ParamInt<px4::params::QE_TYPE_0>) _param_type_0,
		(ParamInt<px4::params::QE_TYPE_1>) _param_type_1,
		(ParamInt<px4::params::QE_TYPE_2>) _param_type_2,
		(ParamInt<px4::params::QE_TYPE_3>) _param_type_3,
		(ParamFloat<px4::params::QE_GEAR_RATIO_0>) _param_gear_ratio_0,
		(ParamFloat<px4::params::QE_GEAR_RATIO_1>) _param_gear_ratio_1,
		(ParamFloat<px4::params::QE_GEAR_RATIO_2>) _param_gear_ratio_2,
		(ParamFloat<px4::params::QE_GEAR_RATIO_3>) _param_gear_ratio_3,
		(ParamBool<px4::params::QE_ENABLE_ODOM>) _param_enable_odom,
		(ParamFloat<px4::params::QE_WHEEL_BASE>) _param_wheel_base,
		(ParamFloat<px4::params::QE_WHEEL_RADIUS>) _param_wheel_radius
	)

	// Instance configuration
	int _instance_id{0};
	bool _is_running{false};

	// Device file descriptors
	int _fd_encoders[8]{-1, -1, -1, -1, -1, -1, -1, -1}; // Up to 8 encoders
	static constexpr int MAX_ENCODERS = 8;
	int _num_active_encoders{4}; // Default to 4 for compatibility

	// Encoder data
	struct encoder_data_s {
		int32_t position;
		float velocity_rad_s;     // rad/s for rotary, m/s for linear
		float angle_rad;          // rad for rotary, m for linear (distance)
		uint64_t timestamp;
		bool valid;
		int32_t pulses_per_rev;   // PPR for rotary, pulses per mm for linear
		bool invert_direction;
		uint8_t encoder_type;     // 0=rotary, 1=linear
		float gear_ratio;         // gear ratio for rotary, screw pitch for linear
	};
	encoder_data_s _encoder_data[MAX_ENCODERS]{};

	// Previous encoder positions for velocity calculation
	int32_t _prev_position[MAX_ENCODERS]{};
	hrt_abstime _prev_timestamp{0};

	// Odometry state
	double _position_x{0.0};
	double _position_y{0.0};
	double _heading{0.0};
	double _linear_velocity{0.0};
	double _angular_velocity{0.0};

	// Publishers
	uORB::Publication<sensor_quad_encoder_s> _sensor_quad_encoder_pub{ORB_ID(sensor_quad_encoder)};
	uORB::Publication<vehicle_odometry_s> _vehicle_odometry_pub{ORB_ID(vehicle_odometry)};

	// Subscribers
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _read_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _odom_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": odometry")};

	// Statistics
	uint32_t _error_count{0};
	uint32_t _read_count{0};
	hrt_abstime _last_error_time{0};

	// Configuration
	static constexpr uint32_t SCHEDULE_INTERVAL{10_ms}; // 100 Hz default
	static constexpr const char *ENCODER_DEVICE_PATHS[MAX_ENCODERS] = {
		"/dev/qe0", "/dev/qe1", "/dev/qe2", "/dev/qe3",
		"/dev/qe4", "/dev/qe5", "/dev/qe6", "/dev/qe7"
	};
};
