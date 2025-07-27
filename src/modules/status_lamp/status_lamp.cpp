#include "status_lamp.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <board_config.h>

#ifndef BOARD_HAS_STATUS_LAMP_TRIGGER
#warning "BOARD_HAS_STATUS_LAMP_TRIGGER not defined - status lamp module will be disabled"
#endif

StatusLamp::StatusLamp() : ModuleParams(nullptr) {}

StatusLamp::~StatusLamp() {
	perf_free(_loop_perf);
	perf_free(_command_perf);
}

void StatusLamp::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void StatusLamp::change_lamp_state()
{
#ifdef BOARD_HAS_STATUS_LAMP_TRIGGER
	const bool active_level = !_param_invert_output.get();
	const bool idle_level = _param_invert_output.get();
	px4_arch_gpiowrite(GPIO_STATUS_LAMP_TRIGGER, active_level);
	px4_usleep(_param_pulse_width.get() * 1000);
	px4_arch_gpiowrite(GPIO_STATUS_LAMP_TRIGGER, idle_level);
	_current_state = (_current_state + 1) % MAX_STATES;
#else
	PX4_WARN("Status lamp GPIO not available on this board");
#endif
}

int StatusLamp::calculate_steps_to_target(uint8_t target_state)
{
	if (target_state >= MAX_STATES) {
		return -1;
	}
	if (target_state == _current_state) {
		return 0;
	}
	if (target_state > _current_state) {
		return target_state - _current_state;
	}
	return (MAX_STATES - _current_state) + target_state;
}

void StatusLamp::process_commands()
{
	status_lamp_command_s cmd;
	if (_lamp_command_sub.update(&cmd)) {
		perf_count(_command_perf);
		const int steps = calculate_steps_to_target(cmd.target_state);
		if (steps < 0) {
			PX4_WARN("Invalid target state: %d", cmd.target_state);
			return;
		}
		if (steps == 0) {
			PX4_DEBUG("Already at target state: %d", cmd.target_state);
			return;
		}
		PX4_INFO("Changing state from %d to %d (%d steps)", _current_state, cmd.target_state, steps);
		for (int i = 0; i < steps; i++) {
			change_lamp_state();
			if (i < steps - 1) {
				px4_usleep(_param_state_interval.get() * 1000);
			}
		}
		PX4_INFO("State change complete, current state: %d", _current_state);
	}
}

void StatusLamp::manual_change_state(uint8_t target_state)
{
	const int steps = calculate_steps_to_target(target_state);

	if (steps <= 0) {
		return;
	}

	PX4_INFO("Manually changing state from %d to %d (%d steps)", _current_state, target_state, steps);

		for (int i = 0; i < (MAX_STATES - _current_state); i++) {
			change_lamp_state();
			if (i < (MAX_STATES - _current_state) - 1) {
				px4_usleep(_param_state_interval.get() * 1000);
			}
		}	PX4_INFO("Manual state change complete, current state: %d", _current_state);
}

void StatusLamp::run()
{
	bool idle_level = _param_invert_output.get();

#ifdef BOARD_HAS_STATUS_LAMP_TRIGGER
	px4_arch_configgpio(GPIO_STATUS_LAMP_TRIGGER);
	px4_arch_gpiowrite(GPIO_STATUS_LAMP_TRIGGER, idle_level);
	PX4_INFO("Status lamp started on GPIO pin");
#else
	PX4_WARN("Status lamp GPIO not available on this board - module disabled");
#endif

	while (!should_exit()) {
		perf_begin(_loop_perf);
		parameters_update();
		process_commands();
		perf_end(_loop_perf);
		px4_usleep(20000);
	}

#ifdef BOARD_HAS_STATUS_LAMP_TRIGGER
	px4_arch_gpiowrite(GPIO_STATUS_LAMP_TRIGGER, idle_level);
#endif
}

int StatusLamp::print_status()
{
	PX4_INFO("Status Lamp");
	PX4_INFO("  Current state: %d", get_current_state());
	PX4_INFO("  Pulse width: %ld ms", (long)_param_pulse_width.get());
	PX4_INFO("  Pulse interval: %ld ms", (long)_param_state_interval.get());
	PX4_INFO("  Logic inverted: %s", _param_invert_output.get() ? "yes" : "no");
#ifdef BOARD_HAS_STATUS_LAMP_TRIGGER
	PX4_INFO("  GPIO available: yes");
#else
	PX4_INFO("  GPIO available: no (board not supported)");
#endif
	perf_print_counter(_loop_perf);
	perf_print_counter(_command_perf);
	return 0;
}

int StatusLamp::task_spawn(int argc, char *argv[])
{
	StatusLamp *instance = new StatusLamp();
	if (instance) {
		_object.store(instance);
		_task_id = px4_task_spawn_cmd("status_lamp",
									  SCHED_DEFAULT,
									  SCHED_PRIORITY_DEFAULT - 5,
									  1024,
									  (px4_main_t)&run_trampoline,
									  (char *const *)argv);
		if (_task_id < 0) {
			PX4_ERR("task start failed");
			delete instance;
			_object.store(nullptr);
			_task_id = -1;
			return PX4_ERROR;
		}
		return PX4_OK;
	}
	PX4_ERR("alloc failed");
	return PX4_ERROR;
}

int StatusLamp::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("module not running");
		return 1;
	}

	if (!strcmp(argv[0], "set")) {
		if (argc < 2) {
			PX4_WARN("Usage: status_lamp set <state>");
			PX4_INFO("Valid states: 0-7 (0=OFF, 1=GREEN, 2=YELLOW, 3=RED, 4=BLUE, 5=WHITE, 6=PURPLE, 7=ORANGE)");
			return 1;
		}

		int target_state = atoi(argv[1]);
		if (target_state < 0 || target_state >= MAX_STATES) {
			PX4_WARN("Invalid state: %d. Valid range: 0-%d", target_state, MAX_STATES - 1);
			return 1;
		}

		StatusLamp *instance = get_instance();

		if (target_state == instance->get_current_state()) {
			PX4_INFO("Already at target state: %d", target_state);
			return 0;
		}

		instance->manual_change_state(target_state);
		return 0;
	}

	return print_usage("unknown command");
}

int StatusLamp::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Status lamp controller that receives state commands via uORB and outputs
trigger pulses to change lamp states sequentially.
The module monitors the status_lamp_command topic and generates pulses
to advance the lamp through 8 states (0-7) sequentially.

### Examples
Manual state change:
$ status_lamp set 3    # Change to RED state
$ status_lamp set 0    # Change to OFF state

)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("status_lamp", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set <state>", "Manually set lamp state (0-7)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int StatusLamp::run_trampoline(int argc, char *argv[])
{
	StatusLamp *instance = get_instance();
	if (instance) {
		instance->run();
	}
	return 0;
}

extern "C" __EXPORT int status_lamp_main(int argc, char *argv[])
{
	return StatusLamp::main(argc, argv);
}
