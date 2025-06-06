#include "WheelController.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

int WheelController::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Unified wheel controller for front and rear wheels of articulated wheel loader.
Supports velocity control with PID feedback from quad encoder.
Integrates with traction control system and safety monitoring.

### Implementation
The module runs at 100Hz and supports multiple instances:
- Instance 0: Front wheel controller
- Instance 1: Rear wheel controller

### Examples
Start front wheel controller:
$ wheel_controller start -i 0 -f

Start rear wheel controller:
$ wheel_controller start -i 1
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("wheel_controller", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Front wheel instance", true);
    PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 1, "Instance number", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int WheelController::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int WheelController::task_spawn(int argc, char *argv[])
{
    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    uint8_t instance = 0;
    bool is_front = false;

    while ((ch = px4_getopt(argc, argv, "fi:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
            case 'f':
                is_front = true;
                break;

            case 'i':
                instance = atoi(myoptarg);
                break;

            default:
                return print_usage("unrecognized flag");
        }
    }

    if (instance > 1) {
        return print_usage("instance must be 0 or 1");
    }

    // If instance is 0, it's front wheel by default
    if (instance == 0) {
        is_front = true;
    }

    WheelController *controller = new WheelController(instance, is_front);

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

extern "C" __EXPORT int wheel_controller_main(int argc, char *argv[])
{
    return WheelController::main(argc, argv);
}
