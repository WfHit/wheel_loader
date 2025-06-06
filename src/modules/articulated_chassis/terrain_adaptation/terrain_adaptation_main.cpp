#include "TerrainAdaptation.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

int TerrainAdaptation::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Intelligent terrain adaptation system for articulated wheel loader.
Automatically detects terrain conditions and adapts vehicle behavior.

### Implementation
The module runs at 50Hz and provides:
- Real-time terrain classification
- Surface roughness detection
- Friction coefficient estimation
- Slope and stability monitoring
- Adaptive control parameter adjustment
- Predictive terrain analysis

### Examples
Start terrain adaptation:
$ terrain_adaptation start

Enable advanced classification:
$ terrain_adaptation classifier 2

Reset terrain memory:
$ terrain_adaptation reset
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("terrain_adaptation", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("classifier", "Set classification method (0=basic, 1=ML, 2=advanced)");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset terrain memory and calibration");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int TerrainAdaptation::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("module not running");
        return 1;
    }

    if (!strcmp(argv[0], "classifier")) {
        if (argc < 2) {
            print_usage("classifier command requires method argument (0-2)");
            return 1;
        }

        int method = atoi(argv[1]);
        if (method < 0 || method > 2) {
            print_usage("classifier method must be 0, 1, or 2");
            return 1;
        }

        get_instance()->set_classifier_method(method);
        PX4_INFO("Classifier method set to %d", method);
        return 0;
    }

    if (!strcmp(argv[0], "reset")) {
        get_instance()->reset_terrain_memory();
        PX4_INFO("Terrain memory and calibration reset");
        return 0;
    }

    return print_usage("unknown command");
}

int TerrainAdaptation::task_spawn(int argc, char *argv[])
{
    TerrainAdaptation *controller = new TerrainAdaptation();

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

extern "C" __EXPORT int terrain_adaptation_main(int argc, char *argv[])
{
    return TerrainAdaptation::main(argc, argv);
}
