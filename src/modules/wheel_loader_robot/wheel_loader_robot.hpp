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

#include <lib/perf/perf_counter.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

// uORB includes - lowercase topic names
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/boom_command.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_command.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/chassis_command.h>
#include <uORB/topics/chassis_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vla_command.h>
#include <uORB/topics/wheel_loader_command.h>
#include <uORB/topics/wheel_loader_status.h>

using namespace time_literals;

/**
 * @brief Wheel Loader Robot Module - Central coordinator with power management
 *
 * Manages:
 * - Command arbitration between manual and autonomous sources
 * - Battery power management and optimization
 * - Subsystem coordination (chassis, boom, bucket)
 * - Safety monitoring and emergency response
 */
class WheelLoaderRobot : public ModuleBase<WheelLoaderRobot>,
			 public ModuleParams,
			 public px4::ScheduledWorkItem
{
public:
	WheelLoaderRobot();
	~WheelLoaderRobot() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void Run() override;
	bool init();

private:
	// System states
	enum class SystemState : uint8_t {
		INIT = 0,
		STANDBY = 1,
		OPERATING = 2,
		POWER_LIMITED = 3,	// Low battery, reduced performance
		EMERGENCY = 4,
		FAULT = 5
	};

	enum class OperationMode : uint8_t {
		MANUAL = 0,
		AUTONOMOUS = 1
	};

	enum class PowerMode : uint8_t {
		NORMAL = 0,
		ECO = 1,		// Economy mode - reduced power consumption
		BOOST = 2,		// High performance - max power allowed
		CRITICAL = 3		// Battery critical - minimum operation only
	};

	// Power management data
	struct PowerState {
		float battery_voltage_v{0.0f};
		float battery_current_a{0.0f};
		float battery_remaining_pct{0.0f};
		float power_consumption_w{0.0f};
		float power_budget_w{0.0f};
		float efficiency_factor{1.0f};
		PowerMode mode{PowerMode::NORMAL};
		bool is_charging{false};

		bool is_critical() const { return battery_remaining_pct < 10.0f; }
		bool is_low() const { return battery_remaining_pct < 25.0f; }
	};

	// Command input structure
	struct CommandInput {
		wheel_loader_command_s command{};
		hrt_abstime timestamp{0};
		bool valid{false};
	};

	// Subsystem health
	struct SubsystemHealth {
		bool chassis_ok{false};
		bool boom_ok{false};
		bool bucket_ok{false};
		bool battery_ok{false};
		hrt_abstime last_update{0};

		bool is_healthy() const {
			return chassis_ok && boom_ok && bucket_ok && battery_ok;
		}
	};

	// Core processing methods
	void update_inputs();
	void update_power_state();
	void update_system_state();
	void execute_commands();
	void publish_status();
	void update_parameters();

	// Power management
	void calculate_power_budget();
	void apply_power_limits(wheel_loader_command_s &cmd);
	void optimize_power_distribution();
	PowerMode determine_power_mode();
	float calculate_efficiency_factor();

	// Command processing
	CommandInput process_manual_input(const manual_control_setpoint_s &manual);
	CommandInput process_vla_input(const vla_command_s &vla);
	wheel_loader_command_s arbitrate_commands();
	void apply_command_limits(wheel_loader_command_s &cmd);

	// Subsystem commands
	void send_chassis_command(const wheel_loader_command_s &cmd);
	void send_boom_command(const wheel_loader_command_s &cmd);
	void send_bucket_command(const wheel_loader_command_s &cmd);

	// State management
	void transition_to(SystemState new_state);
	bool can_transition_to(SystemState new_state) const;

	// Health monitoring
	void update_subsystem_health();
	bool check_emergency_conditions();
	bool validate_command(const wheel_loader_command_s &cmd);

	// State variables
	SystemState _state{SystemState::INIT};
	OperationMode _mode{OperationMode::MANUAL};
	PowerState _power_state{};
	SubsystemHealth _health{};

	// Command inputs
	CommandInput _manual_input{};
	CommandInput _vla_input{};
	wheel_loader_command_s _active_command{};

	// Power tracking
	float _chassis_power_w{0.0f};
	float _boom_power_w{0.0f};
	float _bucket_power_w{0.0f};
	float _total_power_request_w{0.0f};

	// Timing
	hrt_abstime _state_entry_time{0};
	hrt_abstime _last_health_update{0};
	hrt_abstime _last_power_update{0};
	static constexpr uint32_t CONTROL_PERIOD_US = 20_ms;
	static constexpr uint32_t POWER_UPDATE_PERIOD_US = 100_ms;

	// uORB subscriptions
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vla_command_sub{ORB_ID(vla_command)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _power_monitor_sub{ORB_ID(power_monitor)};
	uORB::Subscription _chassis_status_sub{ORB_ID(chassis_status)};
	uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
	uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// uORB publications
	uORB::Publication<wheel_loader_status_s> _status_pub{ORB_ID(wheel_loader_status)};
	uORB::Publication<chassis_command_s> _chassis_cmd_pub{ORB_ID(chassis_command)};
	uORB::Publication<boom_command_s> _boom_cmd_pub{ORB_ID(boom_command)};
	uORB::Publication<bucket_command_s> _bucket_cmd_pub{ORB_ID(bucket_command)};

	// Performance monitoring
	perf_counter_t _loop_perf{nullptr};
	perf_counter_t _power_perf{nullptr};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::WLR_MAX_PWR>) _param_max_power,
		(ParamFloat<px4::params::WLR_ECO_PWR>) _param_eco_power,
		(ParamFloat<px4::params::WLR_CRIT_BAT>) _param_critical_battery,
		(ParamFloat<px4::params::WLR_LOW_BAT>) _param_low_battery,
		(ParamInt<px4::params::WLR_PWR_OPT>) _param_power_optimize,
		(ParamFloat<px4::params::WLR_MAX_SPD>) _param_max_speed,
		(ParamFloat<px4::params::WLR_MAX_ACC>) _param_max_accel,
		(ParamFloat<px4::params::WLR_CMD_TO>) _param_cmd_timeout,
		(ParamInt<px4::params::WLR_ESTOP_EN>) _param_estop_enable
	)
};
