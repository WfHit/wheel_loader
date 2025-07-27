#include "nav_lamp_controller.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <board_config.h>

#ifndef BOARD_HAS_NAV_LAMP
#warning "BOARD_HAS_NAV_LAMP not defined - nav lamp controller module will be disabled"
#endif

NavLampController::NavLampController() : ModuleParams(nullptr) {}

NavLampController::~NavLampController() {
	perf_free(_loop_perf);
	perf_free(_command_perf);
}

void NavLampController::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void NavLampController::change_state_lamp()
{
#ifdef BOARD_HAS_NAV_LAMP
	const bool active_level = !_param_state_invert.get();
	const bool idle_level = _param_state_invert.get();

	// Generate state indicator trigger pulse
	STATUS_LAMP_STATE_PULSE(active_level);
	px4_usleep(_param_pulse_width.get() * 1000);
	STATUS_LAMP_STATE_PULSE(idle_level);
	_current_state = (_current_state + 1) % MAX_STATES;
#else
	PX4_WARN("Navigation lamp GPIO not available on this board");
#endif
}

int NavLampController::calculate_steps_to_target(uint8_t target_state)
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

void NavLampController::handle_state_change(uint8_t target_state)
{
	const int steps = calculate_steps_to_target(target_state);

	if (steps < 0) {
		PX4_WARN("Invalid target state: %d", target_state);
		return;
	}

	if (steps == 0) {
		PX4_DEBUG("Already at target state: %d", target_state);
		return;
	}

	PX4_INFO("Changing state from %d to %d (%d steps)", _current_state, target_state, steps);

	for (int i = 0; i < steps; i++) {
		change_state_lamp();
		if (i < steps - 1) {
			px4_usleep(_param_state_interval.get() * 1000);
		}
	}

	PX4_INFO("State change complete, current state: %d", _current_state);
}

void NavLampController::process_commands()
{
	nav_lamp_command_s cmd;
	if (_lamp_command_sub.update(&cmd)) {
		perf_count(_command_perf);

		switch (cmd.command) {
		case nav_lamp_command_s::CMD_SET_POWER:
			set_power_state(cmd.option == nav_lamp_command_s::OPTION_ON);
			break;

		case nav_lamp_command_s::CMD_SET_STATE:
			handle_state_change(cmd.option);
			break;

		case nav_lamp_command_s::CMD_SET_ILLUMINATION:
			set_light_state(cmd.option == nav_lamp_command_s::OPTION_ON);
			break;

		default:
			PX4_WARN("Unknown command: %d", cmd.command);
			break;
		}
	}
}

void NavLampController::manual_change_state(uint8_t target_state)
{
	PX4_INFO("Manual state change requested to state: %d", target_state);
	handle_state_change(target_state);
}

void NavLampController::set_light_state(bool enable)
{
#ifdef BOARD_HAS_NAV_LAMP
	const bool output_level = _param_light_invert.get() ? !enable : enable;
	STATUS_LAMP_LIGHT_EN(output_level);
	PX4_INFO("Nav lamp illumination %s", enable ? "enabled" : "disabled");
#else
	PX4_WARN("Nav lamp GPIO not available on this board");
#endif
}

void NavLampController::set_power_state(bool enable)
{
#ifdef BOARD_HAS_NAV_LAMP
	const bool output_level = _param_power_invert.get() ? !enable : enable;
	STATUS_LAMP_VCC_EN(output_level);
	PX4_INFO("Nav lamp power %s", enable ? "enabled" : "disabled");
	if (enable) {
		px4_usleep(10000); // 10ms delay for power stabilization
	}
#else
	PX4_WARN("Nav lamp GPIO not available on this board");
#endif
}

void NavLampController::run()
{
#ifdef BOARD_HAS_NAV_LAMP
	// Initialize all navigation lamp GPIOs
	px4_arch_configgpio(GPIO_STATUS_LAMP_VCC);
	px4_arch_configgpio(GPIO_STATUS_LAMP_STATE);
	px4_arch_configgpio(GPIO_STATUS_LAMP_LIGHT);

	// Power on the lamp system (considering power invert parameter)
	const bool power_on_level = _param_power_invert.get() ? false : true;
	STATUS_LAMP_VCC_EN(power_on_level);
	px4_usleep(10000); // 10ms delay for power stabilization

	// Set initial states
	const bool idle_level = _param_state_invert.get();
	STATUS_LAMP_STATE_PULSE(idle_level);

	// Light off initially (considering light invert parameter)
	const bool light_off_level = _param_light_invert.get() ? true : false;
	STATUS_LAMP_LIGHT_EN(light_off_level);

	PX4_INFO("Nav lamp started with dual-lamp configuration:");
	PX4_INFO("  Power GPIO (VCC): enabled");
	PX4_INFO("  State indicator GPIO: configured");
	PX4_INFO("  Illumination GPIO: configured");
#else
	PX4_WARN("Nav lamp GPIO not available on this board - module disabled");
#endif

	while (!should_exit()) {
		perf_begin(_loop_perf);
		parameters_update();
		process_commands();
		perf_end(_loop_perf);
		px4_usleep(20000);
	}

#ifdef BOARD_HAS_NAV_LAMP
	// Shutdown sequence
	const bool shutdown_light_off_level = _param_light_invert.get() ? true : false;
	STATUS_LAMP_LIGHT_EN(shutdown_light_off_level);

	const bool shutdown_idle_level = _param_state_invert.get();
	STATUS_LAMP_STATE_PULSE(shutdown_idle_level);
	px4_usleep(10000);

	// Power off the lamp system (considering power invert parameter)
	const bool shutdown_power_off_level = _param_power_invert.get() ? true : false;
	STATUS_LAMP_VCC_EN(shutdown_power_off_level);
	PX4_INFO("Nav lamp system powered down");
#endif
}

int NavLampController::print_status()
{
	PX4_INFO("Navigation Lamp Controller");
	PX4_INFO("  Current state indicator: %d", get_current_state());
	PX4_INFO("  State pulse width: %ld ms", (long)_param_pulse_width.get());
	PX4_INFO("  State pulse interval: %ld ms", (long)_param_state_interval.get());
	PX4_INFO("  State trigger inverted: %s", _param_state_invert.get() ? "yes" : "no");
	PX4_INFO("  Power control inverted: %s", _param_power_invert.get() ? "yes" : "no");
	PX4_INFO("  Light control inverted: %s", _param_light_invert.get() ? "yes" : "no");
#ifdef BOARD_HAS_NAV_LAMP
	PX4_INFO("  Dual-Lamp GPIO Configuration:");
	PX4_INFO("    Power GPIO (PH12): controls lamp system power");
	PX4_INFO("    State GPIO (PH11): triggers state indicator changes");
	PX4_INFO("    Illumination GPIO (PH10): controls illumination lamp");
#else
	PX4_INFO("  GPIO available: no (board not supported)");
#endif
	perf_print_counter(_loop_perf);
	perf_print_counter(_command_perf);
	return 0;
}

int NavLampController::task_spawn(int argc, char *argv[])
{
	NavLampController *instance = new NavLampController();
	if (instance) {
		_object.store(instance);
		_task_id = px4_task_spawn_cmd("nav_lamp_controller",
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

int NavLampController::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("module not running");
		return 1;
	}

	if (!strcmp(argv[0], "state")) {
		if (argc < 2) {
			PX4_WARN("Usage: nav_lamp_controller state <state>");
			PX4_INFO("Valid states: 0-7 (0=OFF, 1=GREEN, 2=YELLOW, 3=RED, 4=BLUE, 5=WHITE, 6=PURPLE, 7=ORANGE)");
			return 1;
		}

		int target_state = atoi(argv[1]);
		if (target_state < 0 || target_state >= MAX_STATES) {
			PX4_WARN("Invalid state: %d. Valid range: 0-%d", target_state, MAX_STATES - 1);
			return 1;
		}

		NavLampController *instance = get_instance();

		if (target_state == instance->get_current_state()) {
			PX4_INFO("Already at target state: %d", target_state);
			return 0;
		}

		instance->manual_change_state(target_state);
		return 0;
	}

	if (!strcmp(argv[0], "light")) {
		if (argc < 2) {
			PX4_WARN("Usage: nav_lamp_controller light <on|off>");
			return 1;
		}

		NavLampController *instance = get_instance();

		if (!strcmp(argv[1], "on")) {
			instance->set_light_state(true);
			return 0;
		} else if (!strcmp(argv[1], "off")) {
			instance->set_light_state(false);
			return 0;
		} else {
			PX4_WARN("Invalid light command: %s. Use 'on' or 'off'", argv[1]);
			return 1;
		}
	}

	if (!strcmp(argv[0], "power")) {
		if (argc < 2) {
			PX4_WARN("Usage: nav_lamp_controller power <on|off>");
			return 1;
		}

		NavLampController *instance = get_instance();

		if (!strcmp(argv[1], "on")) {
			instance->set_power_state(true);
			return 0;
		} else if (!strcmp(argv[1], "off")) {
			instance->set_power_state(false);
			return 0;
		} else {
			PX4_WARN("Invalid power command: %s. Use 'on' or 'off'", argv[1]);
			return 1;
		}
	}

	return print_usage("unknown command");
}

int NavLampController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Navigation lamp controller with dual-lamp configuration that receives commands via uORB
and controls two separate lamps with different control logic.

The system has two lamps controlled by three GPIOs:
1. State Indicator Lamp: Cycles through 8 states (0-7) using trigger pulses
2. Illumination Lamp: Simple on/off control

GPIO Configuration:
- Power GPIO (PH12): Controls power to entire lamp system (HIGH=on, LOW=off)
- State GPIO (PH11): Generates trigger pulses to change state indicator lamp
- Illumination GPIO (PH10): Controls illumination lamp on/off (HIGH=on, LOW=off)

The module monitors the nav_lamp_command topic and generates pulses
to advance the state indicator through 8 states (0-7) sequentially.

### Examples
Manual state change:
$ nav_lamp_controller state 3    # Change state indicator to state 3
$ nav_lamp_controller state 0    # Change state indicator to OFF state

Illumination control:
$ nav_lamp_controller light on   # Turn illumination lamp on
$ nav_lamp_controller light off  # Turn illumination lamp off

Power control:
$ nav_lamp_controller power on   # Turn lamp system power on
$ nav_lamp_controller power off  # Turn lamp system power off

)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("nav_lamp_controller", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("state <state>", "Manually set state indicator lamp (0-7)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("light <on|off>", "Control illumination lamp");
	PRINT_MODULE_USAGE_COMMAND_DESCR("power <on|off>", "Control lamp system power");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int NavLampController::run_trampoline(int argc, char *argv[])
{
	NavLampController *instance = get_instance();
	if (instance) {
		instance->run();
	}
	return 0;
}

extern "C" __EXPORT int nav_lamp_controller_main(int argc, char *argv[])
{
	return NavLampController::main(argc, argv);
}
