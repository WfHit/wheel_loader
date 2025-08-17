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

// System includes
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/bucket_command.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/parameter_update.h>

// Include the state manager header for the StateInfo struct
#include "bucket_state_manager.hpp"
#include "bucket_hardware_interface.hpp"

// Forward declarations for clean separation
class BucketKinematics;
class BucketHardwareInterface;
class BucketMotionController;
class BucketStateManager;

/**
 * @brief Main bucket control module for PX4
 *
 * This module orchestrates bucket control by coordinating:
 * - Kinematic calculations for bucket linkage with integrated boom compensation
 * - Hardware interface management
 * - Motion control with trajectory planning
 * - State management and fault handling
 */
class BucketControl final : public ModuleBase<BucketControl>,
                            public ModuleParams,
                            public px4::ScheduledWorkItem
{
public:
	BucketControl();
	~BucketControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:

	void Run() override;

	/**
	 * @brief Main control loop stages
	 */
	void update_sensor_data();
	void process_commands();
	void execute_control();
	void publish_telemetry();

	/**
	 * @brief Parameter management
	 */
	void update_parameters();

	/**
	 * @brief Control execution helper methods
	 */
	bool determine_target_position(
		const BucketStateManager::StateInfo& state_info,
		const BucketHardwareInterface::SensorData& sensor_data,
		float& target_actuator_position);
	void send_zero_command();
	void execute_motion_control(
		float target_actuator_position,
		const BucketHardwareInterface::SensorData& sensor_data);

	// Core components
	BucketKinematics* _kinematics{nullptr};
	BucketHardwareInterface* _hardware_interface{nullptr};
	BucketMotionController* _motion_controller{nullptr};
	BucketStateManager* _state_manager{nullptr};

	// uORB interface
	uORB::Subscription _bucket_command_sub{ORB_ID(bucket_command)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Publication<bucket_status_s> _bucket_status_pub{ORB_ID(bucket_status)};

	// Control state
	float _target_bucket_angle_chassis{0.0f};   // Target bucket angle (chassis-relative)
	float _commanded_actuator_position{0.0f};   // Direct actuator position command
	hrt_abstime _last_command_time{0};          // Last command timestamp

	// Performance monitoring
	perf_counter_t _cycle_perf{nullptr};        // Cycle performance counter

	// Module parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BCT_ENABLED>) _param_enabled,
		(ParamFloat<px4::params::BCT_UPDATE_RATE>) _param_update_rate,
		(ParamInt<px4::params::BCT_MOTOR_IDX>) _param_motor_index,
		(ParamInt<px4::params::BCT_ENC_IDX>) _param_encoder_index,

		// Hardware configuration parameters
		(ParamFloat<px4::params::BCT_HBG_MIN_LEN>) _param_hbg_min_len,
		(ParamFloat<px4::params::BCT_HBG_MAX_LEN>) _param_hbg_max_len,
		(ParamFloat<px4::params::BCT_DUTY_LIM>) _param_max_duty
	)	// Control loop timing
	static constexpr uint32_t CONTROL_INTERVAL_US = 20000;  // 50 Hz (20ms)
	static constexpr uint32_t COMMAND_TIMEOUT_US = 1000000;  // Command timeout (1s)
	static constexpr float MICROSECONDS_TO_SECONDS = 1e-6f; // Conversion factor
};
