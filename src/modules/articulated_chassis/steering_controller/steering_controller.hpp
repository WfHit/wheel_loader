#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>

// uORB message includes
#include <uORB/topics/steering_setpoint.h>
#include <uORB/topics/steering_status.h>

// Component includes
#include "steering_servo_controller.hpp"
#include "steering_slip_compensator.hpp"
#include "steering_limit_sensor.hpp"
#include "steering_safety_manager.hpp"

using namespace time_literals;

/**
 * @brief Steering Controller for articulated wheel loader
 *
 * Refactored modular design with composition-based architecture:
 * - SteeringServoController: ST3125 servo communication
 * - SteeringSlipCompensator: Slip compensation logic
 * - SteeringLimitSensor: Limit sensor monitoring
 * - SteeringSafetyManager: Safety management and emergency stops
 */
class SteeringController : public ModuleBase<SteeringController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SteeringController();
	~SteeringController() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SteeringController *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	static constexpr float CONTROL_RATE_HZ = 50.0f;
	static constexpr uint64_t CONTROL_INTERVAL_US = 1_s / CONTROL_RATE_HZ;

	void Run() override;  // ScheduledWorkItem interface
	void run() override;  // ModuleBase interface

	/**
	 * Core control functions
	 */
	void process_steering_command();
	float apply_feedforward(float target_angle, float current_angle);
	float rate_limit(float target, float current);

	/**
	 * Status and monitoring
	 */
	void check_command_timeout();
	void update_status();

	// Component instances (composition-based design)
	SteeringServoController _servo_controller;
	SteeringSlipCompensator _slip_compensator;
	SteeringLimitSensor _limit_sensor;
	SteeringSafetyManager _safety_manager;

	// uORB subscriptions
	uORB::Subscription _steering_setpoint_sub{ORB_ID(steering_setpoint)};

	// uORB publications
	uORB::Publication<steering_status_s> _steering_status_pub{ORB_ID(steering_status)};

	// Control state variables
	float _current_angle_rad{0.0f};
	float _commanded_angle_rad{0.0f};
	float _target_angle_rad{0.0f};

	// Timing
	uint64_t _last_command_time{0};
	uint64_t _last_update_time{0};
	float _dt{0.02f};  // 50Hz nominal

	// Vehicle state
	float _vehicle_speed_mps{0.0f};

	// Performance metrics
	struct {
		float max_error_rad{0.0f};
		float avg_error_rad{0.0f};
		uint32_t command_count{0};
		uint32_t timeout_count{0};
	} _metrics;

	// Control parameters
	DEFINE_PARAMETERS(
		// Basic control
		(ParamFloat<px4::params::STEER_MAX_ANG>) _max_steering_angle,
		(ParamFloat<px4::params::STEER_MAX_RATE>) _max_steering_rate,
		(ParamFloat<px4::params::STEER_DEADBAND>) _steering_deadband,
		(ParamFloat<px4::params::STEER_TRIM>) _steering_trim,
		(ParamInt<px4::params::STEER_REVERSE>) _steering_reverse,

		// Feedforward control
		(ParamFloat<px4::params::STEER_FF_GAIN>) _feedforward_gain,
		(ParamFloat<px4::params::STEER_FF_SPD_SC>) _feedforward_speed_scale,

		// Timeouts
		(ParamFloat<px4::params::STEER_CMD_TO>) _command_timeout_ms,

		// Legacy parameters (deprecated but kept for compatibility)
		(ParamFloat<px4::params::STEER_PWM_MIN>) _servo_pwm_min,
		(ParamFloat<px4::params::STEER_PWM_MAX>) _servo_pwm_max,
		(ParamFloat<px4::params::STEER_CURR_LT>) _servo_current_limit
	)
};
