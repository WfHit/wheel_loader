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
#include <lib/matrix/matrix/Matrix.hpp>
#include <lib/pid/pid.h>
#include <drivers/drv_hrt.h>

// Leverage articulated chassis components
#include "../articulated_chassis/slip_estimator/slip_estimator.hpp"
#include "../articulated_chassis/load_aware_torque/load_aware_torque_distribution.hpp"

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/chassis_command.h>
#include <uORB/topics/chassis_status.h>
#include <uORB/topics/wheel_speeds_setpoint.h>
#include <uORB/topics/steering_command.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/wheel_encoder.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;
using namespace matrix;

/**
 * @brief Advanced Chassis Control with MPC and Slip-Aware Traction
 *
 * Features:
 * - Model Predictive Control for trajectory tracking
 * - Slip estimation and traction optimization using existing slip_estimator
 * - Load-aware torque distribution using existing load_aware_torque
 * - Power-aware torque distribution
 * - Articulated steering coordination
 */
class ChassisControl : public ModuleBase<ChassisControl>,
		       public ModuleParams,
		       public px4::ScheduledWorkItem
{
public:
	ChassisControl();
	~ChassisControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void Run() override;
	bool init();

private:
	// Control modes
	enum class ControlMode : uint8_t {
		IDLE = 0,
		VELOCITY = 1,
		POSITION = 2,
		TRAJECTORY = 3,
		MPC = 4
	};

	// Chassis state
	enum class ChassisState : uint8_t {
		INIT = 0,
		READY = 1,
		ACTIVE = 2,
		TRACTION_LIMITED = 3,
		POWER_LIMITED = 4,
		FAULT = 5
	};

	// MPC state vector: [x, y, theta, v, omega]
	static constexpr size_t MPC_STATE_DIM = 5;
	static constexpr size_t MPC_CONTROL_DIM = 2;  // [v_cmd, omega_cmd]
	static constexpr size_t MPC_HORIZON = 10;

	// Vehicle model parameters
	struct VehicleModel {
		float mass_kg{5000.0f};
		float inertia_kgm2{8000.0f};
		float wheelbase_m{3.0f};
		float track_width_m{2.0f};
		float wheel_radius_m{0.5f};
		float cog_height_m{1.0f};
		float max_torque_nm{2000.0f};
	};

	// MPC data structures
	struct MPCState {
		Vector<float, MPC_STATE_DIM> x_current{};	// Current state
		Vector<float, MPC_STATE_DIM> x_ref{};		// Reference state
		Matrix<float, MPC_STATE_DIM, MPC_HORIZON> x_pred{};  // Predicted states
		Matrix<float, MPC_CONTROL_DIM, MPC_HORIZON> u_opt{}; // Optimal controls
		float cost{0.0f};
		bool solution_valid{false};
	};

	// Wheel identification
	enum WheelID : uint8_t {
		FRONT_LEFT = 0,
		FRONT_RIGHT = 1,
		REAR_LEFT = 2,
		REAR_RIGHT = 3,
		NUM_WHEELS = 4
	};

	// Core control methods
	void process_chassis_command();
	void update_chassis_state();
	void compute_motion_control();
	void publish_wheel_commands();
	void publish_steering_command();
	void publish_chassis_status();

	// MPC control
	void update_mpc_control();
	void solve_mpc_problem();
	void apply_mpc_control();
	Matrix<float, MPC_STATE_DIM, MPC_STATE_DIM> compute_state_transition_matrix(float dt);
	float compute_mpc_cost(const Matrix<float, MPC_CONTROL_DIM, MPC_HORIZON> &u);

	// Motion control algorithms
	void compute_velocity_control(const chassis_command_s &cmd);
	void compute_position_control(const chassis_command_s &cmd);
	void coordinate_wheel_speeds(float linear_vel, float angular_vel);

	// Traction and stability control
	void update_traction_control();
	void apply_anti_slip_regulation();
	void apply_stability_control();
	void compute_torque_distribution();

	// Power management
	void apply_power_limits(float wheel_torques[NUM_WHEELS]);
	void distribute_power_optimally();

	// Safety and limits
	void apply_motion_limits(float &linear_vel, float &angular_vel);
	void enforce_wheel_torque_limits(float torques[NUM_WHEELS]);
	bool check_system_stability();

	// State estimation
	void update_vehicle_state();
	void update_wheel_encoders();
	void estimate_vehicle_motion();

	// Utility functions
	void update_parameters();

	// Existing module components
	SlipEstimator *_slip_estimator{nullptr};
	LoadAwareTorqueDistribution *_torque_distributor{nullptr};

	// Control state
	ControlMode _control_mode{ControlMode::IDLE};
	ChassisState _chassis_state{ChassisState::INIT};
	chassis_command_s _current_command{};

	// MPC state
	MPCState _mpc_state{};
	bool _mpc_enabled{false};

	// Vehicle model and state
	VehicleModel _vehicle_model{};
	float _current_velocity{0.0f};
	float _current_angular_velocity{0.0f};
	float _current_steering_angle{0.0f};
	float _wheel_speeds[NUM_WHEELS]{};
	float _wheel_torques[NUM_WHEELS]{};

	// Power management
	float _available_power_w{0.0f};
	float _power_limit_factor{1.0f};

	// PID controllers
	PID_t _velocity_pid{};
	PID_t _steering_pid{};

	// Timing
	hrt_abstime _last_command_time{0};
	hrt_abstime _last_mpc_update{0};
	hrt_abstime _last_slip_update{0};
	static constexpr uint32_t CONTROL_PERIOD_US = 10_ms;	// 100Hz
	static constexpr uint32_t MPC_PERIOD_US = 50_ms;	// 20Hz
	static constexpr uint32_t SLIP_PERIOD_US = 20_ms;	// 50Hz

	// uORB subscriptions
	uORB::Subscription _chassis_command_sub{ORB_ID(chassis_command)};
	uORB::Subscription _steering_status_sub{ORB_ID(steering_status)};
	uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::SubscriptionMultiArray<wheel_encoder_s, NUM_WHEELS> _wheel_encoder_subs{ORB_ID::wheel_encoder};

	// uORB publications
	uORB::Publication<chassis_status_s> _chassis_status_pub{ORB_ID(chassis_status)};
	uORB::PublicationMulti<wheel_speeds_setpoint_s> _fl_wheel_pub{ORB_ID(wheel_speeds_setpoint)};
	uORB::PublicationMulti<wheel_speeds_setpoint_s> _fr_wheel_pub{ORB_ID(wheel_speeds_setpoint)};
	uORB::PublicationMulti<wheel_speeds_setpoint_s> _rl_wheel_pub{ORB_ID(wheel_speeds_setpoint)};
	uORB::PublicationMulti<wheel_speeds_setpoint_s> _rr_wheel_pub{ORB_ID(wheel_speeds_setpoint)};
	uORB::Publication<steering_command_s> _steering_command_pub{ORB_ID(steering_command)};

	// Performance monitoring
	perf_counter_t _loop_perf{nullptr};
	perf_counter_t _mpc_perf{nullptr};
	perf_counter_t _slip_perf{nullptr};

	// Parameters
	DEFINE_PARAMETERS(
		// Vehicle model
		(ParamFloat<px4::params::CC_MASS>) _param_mass,
		(ParamFloat<px4::params::CC_INERTIA>) _param_inertia,
		(ParamFloat<px4::params::CC_WHL_RAD>) _param_wheel_radius,
		(ParamFloat<px4::params::CC_TRACK_W>) _param_track_width,
		(ParamFloat<px4::params::CC_WHLBASE>) _param_wheelbase,

		// MPC parameters
		(ParamInt<px4::params::CC_MPC_EN>) _param_mpc_enable,
		(ParamFloat<px4::params::CC_MPC_Q_X>) _param_mpc_q_x,
		(ParamFloat<px4::params::CC_MPC_Q_Y>) _param_mpc_q_y,
		(ParamFloat<px4::params::CC_MPC_Q_TH>) _param_mpc_q_theta,
		(ParamFloat<px4::params::CC_MPC_R_V>) _param_mpc_r_v,
		(ParamFloat<px4::params::CC_MPC_R_W>) _param_mpc_r_omega,

		// Slip control
		(ParamInt<px4::params::CC_SLIP_EN>) _param_slip_enable,
		(ParamFloat<px4::params::CC_SLIP_THR>) _param_slip_threshold,

		// Traction control
		(ParamInt<px4::params::CC_ASR_EN>) _param_asr_enable,
		(ParamInt<px4::params::CC_ESP_EN>) _param_esp_enable,
		(ParamFloat<px4::params::CC_TRQ_MAX>) _param_max_torque,

		// PID gains
		(ParamFloat<px4::params::CC_VEL_P>) _param_velocity_p,
		(ParamFloat<px4::params::CC_VEL_I>) _param_velocity_i,
		(ParamFloat<px4::params::CC_VEL_D>) _param_velocity_d,
		(ParamFloat<px4::params::CC_STR_P>) _param_steering_p,
		(ParamFloat<px4::params::CC_STR_I>) _param_steering_i,
		(ParamFloat<px4::params::CC_STR_D>) _param_steering_d,

		// Limits
		(ParamFloat<px4::params::CC_MAX_SPD>) _param_max_speed,
		(ParamFloat<px4::params::CC_MAX_ACC>) _param_max_accel,
		(ParamFloat<px4::params::CC_MAX_STR>) _param_max_steering
	)
};
