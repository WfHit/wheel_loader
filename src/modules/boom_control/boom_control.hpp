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

#include <lib/pid/PID.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionData.hpp>
#include <uORB/topics/hbridge_cmd.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_mag_encoder.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

using namespace time_literals;

/**
 * @brief Boom Control Module for Wheel Loader
 *
 * Controls the boom angle using AS5600 sensor feedback and DC motor with H-bridge (DRV8701).
 * Includes proper kinematics, S-curve motion planning, and PID control.
 */
class BoomControl : public ModuleBase<BoomControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BoomControl();
	~BoomControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void Run() override;
	bool init();

private:
	// Control states
	enum class BoomState : uint8_t {
		IDLE = 0,
		MOVING = 1,
		HOLDING = 2,
		ERROR = 3
	};

	// Preset positions
	enum class BoomPreset : uint8_t {
		GROUND = 0,
		CARRY = 1,
		MAX_HEIGHT = 2,
		CUSTOM = 3
	};

	// Kinematic parameters for triangle linkage
	struct Kinematics {
		float pivot_to_actuator_base;    // Distance from boom pivot to actuator base mount [mm]
		float pivot_to_actuator_attach;  // Distance from boom pivot to actuator attachment point [mm]
		float actuator_base_angle;       // Angle of actuator base mount from horizontal [rad]
		float min_actuator_length;       // Minimum actuator length (fully retracted) [mm]
		float max_actuator_length;       // Maximum actuator length (fully extended) [mm]
	};

	// Parameters
	DEFINE_PARAMETERS(
		// PID gains for position control
		(ParamFloat<px4::params::BOOM_P>) _param_boom_p,
		(ParamFloat<px4::params::BOOM_I>) _param_boom_i,
		(ParamFloat<px4::params::BOOM_D>) _param_boom_d,

		// Motion planning limits
		(ParamFloat<px4::params::BOOM_MAX_VEL>) _param_boom_max_vel,
		(ParamFloat<px4::params::BOOM_MAX_ACC>) _param_boom_max_acc,
		(ParamFloat<px4::params::BOOM_MAX_JERK>) _param_boom_max_jerk,

		// Boom angle limits
		(ParamFloat<px4::params::BOOM_MIN_ANG>) _param_boom_min_angle,
		(ParamFloat<px4::params::BOOM_MAX_ANG>) _param_boom_max_angle,

		// Preset positions
		(ParamFloat<px4::params::BOOM_POS_GND>) _param_boom_pos_ground,
		(ParamFloat<px4::params::BOOM_POS_CARRY>) _param_boom_pos_carry,
		(ParamFloat<px4::params::BOOM_POS_MAX>) _param_boom_pos_max,

		// Kinematic parameters for triangle calculations
		(ParamFloat<px4::params::BOOM_KIN_L1>) _param_kin_pivot_to_base,
		(ParamFloat<px4::params::BOOM_KIN_L2>) _param_kin_pivot_to_attach,
		(ParamFloat<px4::params::BOOM_KIN_THETA>) _param_kin_base_angle,
		(ParamFloat<px4::params::BOOM_ACT_MIN>) _param_actuator_min_length,
		(ParamFloat<px4::params::BOOM_ACT_MAX>) _param_actuator_max_length,

		// Magnetic encoder sensor calibration
		(ParamFloat<px4::params::BOOM_MAG_ENC_OFF>) _param_mag_encoder_offset,
		(ParamFloat<px4::params::BOOM_MAG_ENC_SCALE>) _param_mag_encoder_scale,
		(ParamInt<px4::params::BOOM_MAG_ENC_ID>) _param_mag_encoder_instance_id,

		// H-bridge motor configuration
		(ParamInt<px4::params::BOOM_HBRIDGE_CH>) _param_hbridge_channel,
		(ParamFloat<px4::params::BOOM_MOTOR_DEADZONE>) _param_motor_deadzone,

		// Update rate
		(ParamInt<px4::params::BOOM_UPDATE_RATE>) _param_update_rate
	)

	// Control system components
	PID _position_pid;
	PositionSmoothing _trajectory_generator;

	// uORB subscriptions
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _hbridge_status_sub{ORB_ID(hbridge_status)};
	uORB::SubscriptionData<sensor_mag_encoder_s> _mag_encoder_sub{ORB_ID(sensor_mag_encoder)};

	// uORB publications
	uORB::PublicationMulti<hbridge_cmd_s> _hbridge_cmd_pub{ORB_ID(hbridge_cmd)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

	// Performance counters
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// State variables
	BoomState _state{BoomState::IDLE};
	float _current_boom_angle{0.0f};
	float _target_boom_angle{0.0f};
	float _current_actuator_length{0.0f};
	float _target_actuator_length{0.0f};
	float _motor_output{0.0f};

	bool _upper_limit_reached{false};
	bool _lower_limit_reached{false};
	bool _sensor_valid{false};

	uint64_t _last_sensor_update{0};
	uint64_t _last_command_time{0};

	Kinematics _kinematics{};

	// Kinematic transformation functions
	float boom_angle_to_actuator_length(float boom_angle);
	float actuator_length_to_boom_angle(float actuator_length);
	float as5600_angle_to_actuator_length(float as5600_angle);

	// Control and sensor functions
	void update_parameters();
	void update_sensor_data();
	void process_hbridge_status();
	void process_manual_control();
	void process_vehicle_commands();
	void update_trajectory();
	void run_position_control();
	void check_limits_and_safety();
	void publish_hbridge_command();
	void publish_command_ack(uint16_t command, uint8_t result);

	// Utility functions
	void set_target_position(BoomPreset preset);
	bool check_actuator_limits(float length);
	bool check_boom_limits(float angle);
	void update_kinematics_from_params();
	void reset_trajectory();

	// Safety functions
	void emergency_stop();
	bool is_sensor_timeout();
};
