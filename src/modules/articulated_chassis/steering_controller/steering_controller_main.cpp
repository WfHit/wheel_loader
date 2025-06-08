#include "steering_controller.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

int SteeringController::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Simplified steering controller for articulated wheel loader with ST3125 servo.

Features:
- Direct position commands to ST3125 (servo handles internal PID)
- Slip compensation using PredictiveTraction data
- Feedforward control for improved dynamic response
- Rate limiting and safety monitoring

### Implementation
The module runs at 50Hz and provides:
- Direct servo position control via RoboticServoCommand
- Real-time servo feedback monitoring via ServoFeedback
- Advanced slip compensation using slip estimator data
- Safety monitoring and emergency stop functionality

### Examples
Start steering controller:
$ steering_controller start

Check status:
$ steering_controller status
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("steering_controller", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status information");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int SteeringController::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("module not running");
        return 1;
    }

    if (!strcmp(argv[0], "status")) {
        return get_instance()->print_status();
    }

    return print_usage("unknown command");
}

int SteeringController::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("steering_controller",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_FAST_DRIVER,
                                  2048,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

extern "C" __EXPORT int steering_controller_main(int argc, char *argv[])
{
    return SteeringController::main(argc, argv);
}
