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

// System includes first
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// Library includes
#include <perf/perf_counter.h>

// uORB includes (use lowercase topic names)
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/boom_command.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/parameter_update.h>

// Using declarations
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
 * Coordinates boom actuator components through a well-defined architecture:
 * - Sensor reading and processing (BoomSensorInterface)
 * - Kinematic calculations (BoomKinematics) 
 * - Motion planning and control (BoomMotionController)
 * - Actuator commanding (BoomActuatorInterface)
 * - State management and safety (BoomStateManager)
 *
 * The module uses composition over inheritance for better maintainability
 * and follows RAII principles for resource management. All components are
 * constructed as member objects to ensure proper lifetime management.
 *
 * @note This implementation follows PX4 coding standards including:
 * - Tab-based indentation (4-space display width)
 * - Composition over inheritance architecture
 * - Proper uORB message handling
 * - Parameter validation and bounds checking
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

	// Core components (using composition instead of raw pointers)
	BoomKinematics _kinematics;
	BoomSensorInterface _sensor_interface;
	BoomMotionController _motion_controller;
	BoomActuatorInterface _actuator_interface;
	BoomStateManager _state_manager;

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
