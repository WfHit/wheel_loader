#include "LoadAwareTorqueDistribution.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

int LoadAwareTorqueDistribution::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Load-aware torque distribution for articulated wheel loader.
Optimizes torque allocation between front and rear wheels based on load and conditions.

### Implementation
The module runs at 50Hz and provides:
- Dynamic torque distribution based on load conditions
- Stability-aware torque allocation
- Efficiency optimization for fuel/energy savings
- Terrain-adaptive distribution strategies
- Real-time load monitoring and adaptation

### Examples
Start load aware torque distribution:
$ load_aware_torque start

Enable adaptive mode:
$ load_aware_torque adaptive on

Reset to default distribution:
$ load_aware_torque reset
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("load_aware_torque", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("adaptive", "Control adaptive mode (on/off)");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset to default distribution");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int LoadAwareTorqueDistribution::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("module not running");
        return 1;
    }

    if (!strcmp(argv[0], "adaptive")) {
        if (argc < 2) {
            print_usage("adaptive command requires on/off argument");
            return 1;
        }

        bool enable = (!strcmp(argv[1], "on") || !strcmp(argv[1], "true"));
        get_instance()->set_adaptive_enabled(enable);
        PX4_INFO("Adaptive distribution %s", enable ? "enabled" : "disabled");
        return 0;
    }

    if (!strcmp(argv[0], "reset")) {
        get_instance()->reset_distribution();
        PX4_INFO("Distribution reset to default");
        return 0;
    }

    return print_usage("unknown command");
}

int LoadAwareTorqueDistribution::task_spawn(int argc, char *argv[])
{
    LoadAwareTorqueDistribution *controller = new LoadAwareTorqueDistribution();

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

extern "C" __EXPORT int load_aware_torque_main(int argc, char *argv[])
{
    return LoadAwareTorqueDistribution::main(argc, argv);
}
