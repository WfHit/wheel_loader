#include "SteeringController.hpp"

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
Steering controller for articulated wheel loader.
Controls steering servo via ST3125 servo controller with position feedback.

### Implementation
The module runs at 50Hz and provides:
- PID position control with rate limiting
- Articulation angle feedback monitoring
- Safety monitoring and current limiting
- Controller health reporting

### Examples
Start steering controller:
$ steering_controller start

Run steering test:
$ steering_controller test
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("steering_controller", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Run steering test sequence");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int SteeringController::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("module not running");
        return 1;
    }

    if (!strcmp(argv[0], "test")) {
        get_instance()->request_test();
        return 0;
    }

    return print_usage("unknown command");
}

int SteeringController::task_spawn(int argc, char *argv[])
{
    SteeringController *controller = new SteeringController();

    if (!controller) {
        PX4_ERR("alloc failed");
        return -1;
    }

    _object.store(controller);
    _task_id = task_id_is_work_queue;

    if (!controller->init()) {
        PX4_ERR("init failed");
        delete controller;
        _object.store(nullptr);
        _task_id = -1;
        return -1;
    }

    return 0;
}

extern "C" __EXPORT int steering_controller_main(int argc, char *argv[])
{
    return SteeringController::main(argc, argv);
}
