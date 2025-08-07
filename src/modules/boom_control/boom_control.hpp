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

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/boom_command.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/parameter_update.h>

#include <perf/perf_counter.h>

using namespace time_literals;

// Forward declarations
class BoomKinematics;
class BoomSensorInterface;
class BoomMotionController;
class BoomActuatorInterface;
class BoomStateManager;

/**
 * @brief Main boom control module for wheel loader lifting system
 *
 * Coordinates boom actuator components:
 * - Sensor reading and processing
 * - Kinematic calculations
 * - Motion planning and control
 * - Actuator commanding
 * - State management and safety
 */
class BoomControl final : public ModuleBase<BoomControl>,
                          public ModuleParams,
                          public px4::ScheduledWorkItem
{
public:
	static constexpr uint32_t CONTROL_INTERVAL_US = 20000; // 50 Hz

	BoomControl();
	~BoomControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * @brief Main control pipeline stages
	 */
	void update_sensors();
	void process_commands();
	void update_motion_planning();
	void execute_control();
	void publish_telemetry();

	/**
	 * @brief System management
	 */
	void update_parameters();
	void handle_emergency_stop();
	bool check_system_health();

	// Core components
	BoomKinematics* _kinematics{nullptr};
	BoomSensorInterface* _sensor_interface{nullptr};
	BoomMotionController* _motion_controller{nullptr};
	BoomActuatorInterface* _actuator_interface{nullptr};
	BoomStateManager* _state_manager{nullptr};

	// Current system state
	float _current_boom_angle{0.0f};          // rad
	float _current_actuator_length{0.0f};     // mm
	float _target_boom_angle{0.0f};           // rad
	hrt_abstime _last_command_time{0};

	// uORB interface
	uORB::Subscription _boom_command_sub{ORB_ID(boom_command)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Publication<boom_status_s> _boom_status_pub{ORB_ID(boom_status)};

	// Performance monitoring
	perf_counter_t _cycle_perf;
	perf_counter_t _control_latency_perf;

	// Module parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BOOM_EN>) _param_enabled,
		(ParamFloat<px4::params::BOOM_RATE>) _param_update_rate
	)
};
