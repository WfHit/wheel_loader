#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/status_lamp_command.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

class StatusLamp : public ModuleBase<StatusLamp>, public ModuleParams
{
public:
	StatusLamp();
	~StatusLamp() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int run_trampoline(int argc, char *argv[]);
	void run();
	int print_status() override;

	// Public methods for manual control
	uint8_t get_current_state() const { return _current_state; }
	void manual_change_state(uint8_t target_state);

private:
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _command_perf{perf_alloc(PC_COUNT, MODULE_NAME ": commands")};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _lamp_command_sub{ORB_ID(status_lamp_command)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SLAMP_PULSE_MS>) _param_pulse_width,
		(ParamInt<px4::params::SLAMP_INVERT>) _param_invert_output,
		(ParamInt<px4::params::SLAMP_INT_MS>) _param_state_interval
	)

	uint8_t _current_state{0};
	static constexpr uint8_t MAX_STATES = 8;

	void parameters_update();
	void process_commands();
	void change_lamp_state();
	int calculate_steps_to_target(uint8_t target_state);
};
