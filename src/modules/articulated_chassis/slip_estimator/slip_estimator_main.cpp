#include "SlipEstimator.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

int SlipEstimator::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Advanced slip estimation for articulated wheel loader.
Provides real-time wheel slip estimation using sensor fusion.

### Implementation
The module runs at 50Hz and provides:
- Longitudinal slip estimation for front and rear wheels
- Lateral slip estimation
- Surface friction coefficient estimation
- Tire-road contact state monitoring

Methods available:
- Basic kinematic estimation
- Extended Kalman Filter with IMU fusion

### Examples
Start slip estimator:
$ slip_estimator start

Start with specific method:
$ slip_estimator start -m 1
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("slip_estimator", "estimator");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_INT('m', 0, 0, 1, "Estimation method (0=basic, 1=EKF)", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int SlipEstimator::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int SlipEstimator::task_spawn(int argc, char *argv[])
{
    int method = 1; // Default to EKF

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "m:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'm':
            method = atoi(myoptarg);
            break;
        case '?':
            return print_usage("unknown option");
        default:
            PX4_WARN("unrecognized flag");
            return print_usage(nullptr);
        }
    }

    SlipEstimator *estimator = new SlipEstimator();

    if (!estimator) {
        PX4_ERR("alloc failed");
        return -1;
    }

    _object.store(estimator);
    _task_id = task_id_is_work_queue;

    if (!estimator->init()) {
        PX4_ERR("init failed");
        delete estimator;
        _object.store(nullptr);
        _task_id = -1;
        return -1;
    }

    return 0;
}

extern "C" __EXPORT int slip_estimator_main(int argc, char *argv[])
{
    return SlipEstimator::main(argc, argv);
}
