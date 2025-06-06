#include "PredictiveTractionControl.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

int PredictiveTractionControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Predictive traction control using Model Predictive Control (MPC).
Optimizes traction performance while preventing wheel slip and maintaining stability.

### Implementation
The module runs at 50Hz and provides:
- Real-time slip prediction and prevention
- Model Predictive Control optimization
- Adaptive learning of vehicle and terrain parameters
- Multi-objective optimization (slip, stability, control effort)
- Proactive traction intervention

### Examples
Start predictive traction control:
$ predictive_traction start

Enable learning mode:
$ predictive_traction learning on

Reset learned parameters:
$ predictive_traction reset
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("predictive_traction", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("learning", "Control learning mode (on/off)");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset learned parameters");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int PredictiveTractionControl::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("module not running");
        return 1;
    }

    if (!strcmp(argv[0], "learning")) {
        if (argc < 2) {
            print_usage("learning command requires on/off argument");
            return 1;
        }

        bool enable = (!strcmp(argv[1], "on") || !strcmp(argv[1], "true"));
        get_instance()->set_learning_enabled(enable);
        PX4_INFO("Learning %s", enable ? "enabled" : "disabled");
        return 0;
    }

    if (!strcmp(argv[0], "reset")) {
        get_instance()->reset_learned_parameters();
        PX4_INFO("Learned parameters reset");
        return 0;
    }

    return print_usage("unknown command");
}

int PredictiveTractionControl::task_spawn(int argc, char *argv[])
{
    PredictiveTractionControl *controller = new PredictiveTractionControl();

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

extern "C" __EXPORT int predictive_traction_main(int argc, char *argv[])
{
    return PredictiveTractionControl::main(argc, argv);
}
