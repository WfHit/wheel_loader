/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/err.h>

#include "common/trajectory_types.hpp"
#include "modes/operation_mode_base.hpp"
#include "modes/manual_mode/manual_mode.hpp"
#include "modes/vla_mode/vla_mode.hpp"

namespace wheel_loader
{

/**
 * @class OperationModeManager
 * @brief Main operation mode manager for wheel loader robot
 *
 * This class manages the operation modes for a wheel loader robot, including:
 * - Manual mode: RC input with trajectory generation
 * - VLA mode: Vision-Language-Action autonomous operation
 *
 * The manager handles mode switching, safety checks, and coordinates between
 * different operation modes. It publishes trajectory setpoints that are
 * consumed by the trajectory followers.
 *
 * Architecture:
 * - Operation modes generate trajectory setpoints in world coordinates
 * - Trajectory followers (chassis/bucket) subscribe to setpoints and control actuators
 * - Manager handles mode transitions and safety oversight
 */
class OperationModeManager : public ModuleBase<OperationModeManager>,
                           public ModuleParams,
                           public px4::ScheduledWorkItem
{
public:
	OperationModeManager();
	~OperationModeManager() override;

	bool init() override;

	int print_status() override;

	static int task_spawn(int argc, char *argv[]);

	static OperationModeManager *instantiate(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

private:
	static constexpr int RUN_INTERVAL = 100000; // 10Hz in microseconds

	/**
	 * Operation modes enumeration
	 */
	enum class OperationMode : uint8_t {
		MANUAL = 0,   ///< Manual RC control
		VLA = 1,      ///< Vision-Language-Action autonomous
		EMERGENCY = 2 ///< Emergency stop mode
	};

	void Run() override;

	/**
	 * Update all subscriptions
	 */
	void update_subscriptions();

	/**
	 * Update vehicle status and check for safety conditions
	 */
	void update_vehicle_status();

	/**
	 * Check if mode switching is requested and safe
	 */
	void check_mode_switching();

	/**
	 * Switch to a new operation mode
	 * @param new_mode Target operation mode
	 * @return true if switch was successful
	 */
	bool switch_mode(OperationMode new_mode);

	/**
	 * Update the current active mode
	 */
	void update_current_mode();

	/**
	 * Publish trajectory setpoints from current mode
	 */
	void publish_trajectory_setpoints();

	/**
	 * Handle emergency conditions
	 */
	void handle_emergency();

	/**
	 * Reset all modes and setpoints
	 */
	void reset_all_modes();

	/**
	 * Check safety conditions for mode switching
	 */
	bool check_safety_conditions();

	/**
	 * Get mode name string for logging
	 */
	const char* get_mode_name(OperationMode mode);

	// uORB subscriptions
	uORB::Subscription manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription vla_command_sub{ORB_ID(vla_command)};

	// uORB publications
	uORB::Publication<chassis_trajectory_setpoint_s> chassis_setpoint_pub{ORB_ID(chassis_trajectory_setpoint)};
	uORB::Publication<bucket_trajectory_setpoint_s> bucket_setpoint_pub{ORB_ID(bucket_trajectory_setpoint)};
	uORB::Publication<operation_mode_status_s> mode_status_pub{ORB_ID(operation_mode_status)};

	// Operation modes
	std::unique_ptr<ManualMode> manual_mode{nullptr};
	std::unique_ptr<VlaMode> vla_mode{nullptr};
	OperationModeBase* current_mode{nullptr};

	// Current operation mode
	OperationMode current_operation_mode{OperationMode::MANUAL};
	OperationMode requested_mode{OperationMode::MANUAL};

	// Vehicle state
	manual_control_setpoint_s manual_control{};
	vehicle_status_s vehicle_status{};
	vehicle_local_position_s vehicle_position{};
	vla_command_s vla_command{};

	// Mode switching
	bool mode_switch_pending{false};
	hrt_abstime mode_switch_time{0};
	hrt_abstime last_mode_switch{0};
	static constexpr hrt_abstime MODE_SWITCH_DELAY = 500000; // 0.5 seconds
	static constexpr hrt_abstime MIN_MODE_SWITCH_INTERVAL = 2000000; // 2 seconds

	// Safety and status
	bool emergency_stop{false};
	bool system_armed{false};
	bool position_valid{false};
	bool manual_control_valid{false};
	bool vla_command_valid{false};

	// Performance counters
	perf_counter_t loop_perf{nullptr};
	perf_counter_t mode_switch_perf{nullptr};

	// Parameters
	DEFINE_PARAMETERS(
		// Mode selection
		(ParamInt<px4::params::OMM_DEFAULT_MODE>) param_default_mode,
		(ParamInt<px4::params::OMM_MODE_SWITCH_CH>) param_mode_switch_channel,
		(ParamFloat<px4::params::OMM_MODE_SWITCH_THRESH>) param_mode_switch_threshold,

		// Safety parameters
		(ParamInt<px4::params::OMM_SAFETY_ENABLE>) param_safety_enable,
		(ParamFloat<px4::params::OMM_EMERGENCY_TIMEOUT>) param_emergency_timeout,
		(ParamFloat<px4::params::OMM_MIN_POSITION_ACCURACY>) param_min_position_accuracy,

		// System parameters
		(ParamInt<px4::params::OMM_DEBUG_ENABLE>) param_debug_enable,
		(ParamFloat<px4::params::OMM_UPDATE_RATE>) param_update_rate
	)
};

} // namespace wheel_loader
