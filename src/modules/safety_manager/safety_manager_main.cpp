#include "SafetyManager.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

int SafetyManager::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Advanced safety manager for chassis control system.
Monitors safety-critical parameters and implements fail-safe behaviors.

### Implementation
The module runs at 100Hz and provides:
- Real-time safety parameter monitoring
- Fail-safe behavior implementation
- Independent safety override capability
- Safety interlocks and permits management
- Continuous safety risk assessment
- Emergency response procedures

### Examples
Start safety manager:
$ safety_manager start

Check safety status:
$ safety_manager status

Trigger emergency stop:
$ safety_manager emergency_stop

Reset safety system:
$ safety_manager reset
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("safety_manager", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("emergency_stop", "Trigger emergency stop");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset safety system");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int SafetyManager::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("module not running");
        return 1;
    }

    if (!strcmp(argv[0], "emergency_stop")) {
        get_instance()->trigger_emergency_stop();
        return 0;
    }

    if (!strcmp(argv[0], "reset")) {
        get_instance()->reset_safety_system();
        return 0;
    }

    return print_usage("unknown command");
}

int SafetyManager::task_spawn(int argc, char *argv[])
{
    SafetyManager *manager = new SafetyManager();

    if (!manager) {
        PX4_ERR("alloc failed");
        return -1;
    }

    _object.store(manager);
    _task_id = task_id_is_work_queue;

    if (!manager->init()) {
        PX4_ERR("init failed");
        delete manager;
        _object.store(nullptr);
        _task_id = -1;
        return -1;
    }

    return 0;
}

extern "C" __EXPORT int safety_manager_main(int argc, char *argv[])
{
    return SafetyManager::main(argc, argv);
}
