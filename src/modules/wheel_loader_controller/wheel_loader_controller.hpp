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

// System includes first
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// Library includes
#include <lib/perf/perf_counter.h>

// uORB includes (use lowercase topic names)
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/boom_command.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_command.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/slip_estimation.h>
#include <uORB/topics/vla_command.h>
#include <uORB/topics/steering_command.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/task_execution_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wheel_loader_command.h>
#include <uORB/topics/wheel_loader_status.h>
#include <uORB/topics/wheel_speeds_setpoint.h>
#include <uORB/topics/wheel_status.h>

using namespace time_literals;

// Component includes (required for composition)
#include "wheel_loader_safety_manager.hpp"
#include "wheel_loader_mode_manager.hpp"

/**
 * @brief Wheel Loader Controller Module
 *
 * Central coordination and control module for wheel loader operations.
 * Manages command arbitration, subsystem coordination, state management,
 * and safety oversight for the complete wheel loader system.
 *
 * The module supports dual operation modes:
 * - Manual RC Control: Direct control via RC transmitter/joystick
 * - VLA Autonomous: Vision-Language-Action autonomous operation
 *
 * Key features:
 * - Command arbitration with safety priority (manual always overrides autonomous)
 * - Subsystem health monitoring and fault detection
 * - Emergency stop functionality accessible from any state
 * - Smooth mode transitions with timeout protection
 * - Slip detection and traction control integration
 *
 * @note This implementation follows PX4 coding standards including:
 * - Tab-based indentation (4-space display width)
 * - Proper uORB message handling
 * - Parameter validation and bounds checking
 * - Performance monitoring and safety oversight
 */
class WheelLoaderController : public ModuleBase<WheelLoaderController>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	static constexpr float CONTROL_RATE_HZ = 50.0f;
	static constexpr uint64_t CONTROL_INTERVAL_US = 1_s / CONTROL_RATE_HZ;
	static constexpr float COMMAND_TIMEOUT_S = 0.5f;
	static constexpr float HEALTH_TIMEOUT_S = 1.0f;
	static constexpr float MAX_EMERGENCY_STOP_TIME_S = 0.1f;

	WheelLoaderController();
	~WheelLoaderController() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void Run() override;
	bool init();

private:
	// Control states
	enum class ControlState : uint8_t {
		INITIALIZING = 0,
		IDLE = 1,
		MANUAL_OPERATION = 2,       // RC/joystick direct control (replaces MANUAL_CONTROL)
		AUTO_OPERATION = 3,         // VLA-driven autonomous control
		TASK_EXECUTION = 4,         // Traditional task execution (kept for compatibility)
		MODE_TRANSITION = 5,        // Safe switching between operation modes
		EMERGENCY_STOP = 6,
		ERROR = 7
	};

	// Command source identification
	enum class CommandSource : uint8_t {
		NONE = 0,
		MANUAL = 1,             // RC/joystick control
		VLA = 2,          // VLA autonomous control
		TASK_EXECUTION = 3,     // Traditional task execution
		EXTERNAL = 4            // External system commands
	};

	// Import types from components
	using OperationMode = WheelLoaderModeManager::OperationMode;
	using HealthState = WheelLoaderSafetyManager::HealthState;

	// Core processing functions
	void processWheelLoaderCommand();
	void processTaskExecution();
	void processVehicleCommand();
	void processVlaCommand();

	// Mode management functions (delegated to mode manager)
	void processModeSwitch();

	// Auto operation functions (VLA interface)
	void processAutoLoadSequence();
	void processAutoDumpSequence();
	void convertVlaToWheelLoaderCommand(const vla_command_s &vla_cmd, wheel_loader_command_s &wl_cmd);

	// Command processing and arbitration
	CommandSource selectActiveCommandSource();
	bool validateCommand(const wheel_loader_command_s &cmd);
	void applyCommandLimits(wheel_loader_command_s &cmd);
	void generateSubsystemCommands(const wheel_loader_command_s &cmd);

	// State management
	void updateControlState();
	void transitionToState(ControlState new_state);
	bool isValidStateTransition(ControlState from, ControlState to);
	void resetControlState();
	void publishCommands();
	void publishStatus();

	// Parameter updates
	void updateParams();

	// Core components (using composition for better maintainability and testing)
	WheelLoaderSafetyManager _safety_manager;           // Safety and health monitoring
	WheelLoaderModeManager _mode_manager;               // Operation mode management

	// uORB subscriptions
	uORB::Subscription _wheel_loader_command_sub{ORB_ID(wheel_loader_command)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _task_execution_command_sub{ORB_ID(task_execution_command)};
	uORB::Subscription _vla_command_sub{ORB_ID(vla_command)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// uORB publications
	uORB::Publication<wheel_loader_status_s> _wheel_loader_status_pub{ORB_ID(wheel_loader_status)};

	// Subsystem command publications
	uORB::PublicationMulti<wheel_speeds_setpoint_s> _front_wheel_setpoint_pub{ORB_ID(wheel_speeds_setpoint)};
	uORB::PublicationMulti<wheel_speeds_setpoint_s> _rear_wheel_setpoint_pub{ORB_ID(wheel_speeds_setpoint)};
	uORB::Publication<boom_command_s> _boom_command_pub{ORB_ID(boom_command)};
	uORB::Publication<bucket_command_s> _bucket_command_pub{ORB_ID(bucket_command)};
	uORB::Publication<steering_command_s> _steering_command_pub{ORB_ID(steering_command)};

	// Control state
	ControlState _control_state{ControlState::INITIALIZING};
	ControlState _previous_state{ControlState::INITIALIZING};
	CommandSource _active_command_source{CommandSource::NONE};
	hrt_abstime _state_entered_time{0};
	hrt_abstime _last_command_time{0};

	// Command storage
	wheel_loader_command_s _current_command{};
	wheel_loader_command_s _manual_command{};
	wheel_loader_command_s _vla_command{};
	wheel_loader_command_s _task_command{};
	wheel_loader_command_s _external_command{};

	// VLA interface data
	vla_command_s _current_vla_data{};
	hrt_abstime _last_vla_time{0};
	bool _vla_valid{false};

	// Performance counters
	perf_counter_t _cycle_perf;

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::WLC_MAX_SPEED>) _max_speed,
		(ParamFloat<px4::params::WLC_MAX_ACCEL>) _max_accel,
		(ParamFloat<px4::params::WLC_CMD_TIMEOUT>) _cmd_timeout,
		(ParamFloat<px4::params::WLC_HEALTH_TO>) _health_timeout,
		(ParamInt<px4::params::WLC_ESTOP_EN>) _estop_enable,
		(ParamInt<px4::params::WLC_DIAG_EN>) _diagnostic_enable,
		(ParamInt<px4::params::WLC_FRONT_WHEEL>) _front_wheel_idx,
		(ParamInt<px4::params::WLC_REAR_WHEEL>) _rear_wheel_idx,
		(ParamFloat<px4::params::WLC_CTRL_RATE>) _control_rate,
		(ParamFloat<px4::params::WLC_SAFE_ACCEL>) _safe_accel,
		(ParamFloat<px4::params::WLC_SAFE_SPEED>) _safe_speed,
		// New parameters for dual operation modes
		(ParamInt<px4::params::WLC_OP_MODE>) _operation_mode_param,
		(ParamInt<px4::params::WLC_VLA_EN>) _vla_enable,
		(ParamFloat<px4::params::WLC_VLA_TO>) _vla_timeout,
		(ParamFloat<px4::params::WLC_MODE_TRANS_T>) _mode_transition_time,
		(ParamInt<px4::params::WLC_AUTO_LOAD_EN>) _auto_load_enable,
		(ParamInt<px4::params::WLC_AUTO_DUMP_EN>) _auto_dump_enable
	)
};
