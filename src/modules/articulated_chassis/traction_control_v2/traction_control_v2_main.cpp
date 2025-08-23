#include "core/TractionControlManager.hpp"
#include "modules/SlipEstimatorModule.hpp"
#include "modules/PredictiveTractionModule.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

using namespace traction_control;

/**
 * @brief Main traction control application
 *
 * This is the entry point for the refactored traction control system.
 * It manages the lifecycle of all traction control modules.
 */
class TractionControlApp : public ModuleBase<TractionControlApp>, public px4::ScheduledWorkItem {
public:
    TractionControlApp();
    ~TractionControlApp() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static TractionControlApp *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    bool init();

    int print_status() override;

private:
    void Run() override;

    static constexpr uint32_t SCHEDULE_INTERVAL{20_ms}; // 50Hz control loop

    std::unique_ptr<TractionControlManager> _manager;
    bool _initialized{false};

    // Performance monitoring
    perf_counter_t _loop_perf{nullptr};
    perf_counter_t _interval_perf{nullptr};

    void registerModules();
    void setupDataProviders();
    void setupCommandExecutors();
};

TractionControlApp::TractionControlApp()
    : ModuleBase(MODULE_NAME)
    , ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
    , _manager(std::make_unique<TractionControlManager>())
{
    _loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle");
    _interval_perf = perf_alloc(PC_INTERVAL, MODULE_NAME ": interval");
}

bool TractionControlApp::init()
{
    if (_initialized) {
        return true;
    }

    PX4_INFO("Initializing Traction Control V2 system");

    // Register all available modules
    registerModules();

    // Setup data providers and command executors
    setupDataProviders();
    setupCommandExecutors();

    // Initialize the manager
    if (!_manager->initialize()) {
        PX4_ERR("Failed to initialize traction control manager");
        return false;
    }

    // Start the scheduled work item
    ScheduleOnInterval(SCHEDULE_INTERVAL);

    _initialized = true;
    PX4_INFO("Traction Control V2 system initialized successfully");
    return true;
}

void TractionControlApp::Run()
{
    if (should_exit()) {
        ScheduleClear();
        return;
    }

    perf_begin(_loop_perf);
    perf_count(_interval_perf);

    // Get current traction state (simplified - would come from sensors)
    TractionState state{};
    state.timestamp = hrt_absolute_time();
    state.dynamics.dynamics_valid = true;
    // ... populate state from actual sensors

    // Process traction control
    TractionCommand command{};
    if (_manager->process(state, command)) {
        // Command will be executed by registered executors
    } else {
        PX4_DEBUG("Traction control processing failed");
    }

    perf_end(_loop_perf);
}

void TractionControlApp::registerModules()
{
    // Register slip estimator module
    auto slip_estimator = std::make_shared<SlipEstimatorModule>();
    if (!_manager->registerModule(slip_estimator)) {
        PX4_WARN("Failed to register slip estimator module");
    }

    // Register predictive traction module
    auto predictive_traction = std::make_shared<PredictiveTractionModule>();
    if (!_manager->registerModule(predictive_traction)) {
        PX4_WARN("Failed to register predictive traction module");
    }

    // Additional modules would be registered here...
    // auto terrain_adaptation = std::make_shared<TerrainAdaptationModule>();
    // auto load_aware_torque = std::make_shared<LoadAwareTorqueModule>();
}

void TractionControlApp::setupDataProviders()
{
    // Data providers would be created and registered here
    // These would interface with actual sensors and vehicle systems

    // Example:
    // auto encoder_provider = std::make_shared<EncoderDataProvider>();
    // _manager->registerDataProvider(encoder_provider);

    // auto attitude_provider = std::make_shared<AttitudeDataProvider>();
    // _manager->registerDataProvider(attitude_provider);
}

void TractionControlApp::setupCommandExecutors()
{
    // Command executors would be created and registered here
    // These would interface with actuators and control systems

    // Example:
    // auto motor_executor = std::make_shared<MotorCommandExecutor>();
    // _manager->registerCommandExecutor(motor_executor);

    // auto brake_executor = std::make_shared<BrakeCommandExecutor>();
    // _manager->registerCommandExecutor(brake_executor);
}

int TractionControlApp::print_status()
{
    if (!_initialized) {
        PX4_INFO("Traction Control V2: Not initialized");
        return 0;
    }

    // Print overall status
    PX4_INFO("Traction Control V2 Status:");
    PX4_INFO("  Active modules: %zu", _manager->getActiveModuleCount());
    PX4_INFO("  Arbitration: %s", _manager->isInitialized() ? "enabled" : "disabled");

    // Print module status
    auto modules = _manager->getAllModules();
    for (const auto& module : modules) {
        auto metrics = module->getMetrics();
        PX4_INFO("  Module: %s v%s", module->getName(), module->getVersion());
        PX4_INFO("    Enabled: %s", module->isEnabled() ? "yes" : "no");
        PX4_INFO("    Priority: %d", module->getPriority());
        PX4_INFO("    Process count: %u", metrics.process_count);
        PX4_INFO("    Success rate: %.1f%%",
                metrics.process_count > 0 ?
                (100.0f * metrics.success_count / metrics.process_count) : 0.0f);
        PX4_INFO("    Avg processing time: %.1f μs", metrics.avg_processing_time_us);
        PX4_INFO("    Effectiveness: %.1f%%", metrics.effectiveness_score * 100.0f);
    }

    // Print combined metrics
    auto combined_metrics = _manager->getCombinedMetrics();
    PX4_INFO("Combined Metrics:");
    PX4_INFO("  Total processes: %u", combined_metrics.process_count);
    PX4_INFO("  Overall success rate: %.1f%%",
            combined_metrics.process_count > 0 ?
            (100.0f * combined_metrics.success_count / combined_metrics.process_count) : 0.0f);
    PX4_INFO("  Average processing time: %.1f μs", combined_metrics.avg_processing_time_us);
    PX4_INFO("  CPU usage: %.1f%%", combined_metrics.cpu_usage_percent);

    // Print performance counters
    perf_print_counter(_loop_perf);
    perf_print_counter(_interval_perf);

    return 0;
}

int TractionControlApp::task_spawn(int argc, char *argv[])
{
    TractionControlApp *instance = new TractionControlApp();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }

    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

TractionControlApp *TractionControlApp::instantiate(int argc, char *argv[])
{
    return new TractionControlApp();
}

int TractionControlApp::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("not running");
        return 1;
    }

    TractionControlApp *instance = get_instance();

    if (argc > 0) {
        if (strcmp(argv[0], "status") == 0) {
            return instance->print_status();
        }

        if (strcmp(argv[0], "reset") == 0) {
            instance->_manager->reset();
            PX4_INFO("Traction control system reset");
            return 0;
        }

        if (strcmp(argv[0], "emergency_stop") == 0) {
            instance->_manager->emergencyStop();
            PX4_WARN("Emergency stop activated");
            return 0;
        }

        if (strcmp(argv[0], "enable") == 0 && argc > 1) {
            if (instance->_manager->setModuleEnabled(argv[1], true)) {
                PX4_INFO("Module %s enabled", argv[1]);
                return 0;
            } else {
                PX4_ERR("Failed to enable module %s", argv[1]);
                return 1;
            }
        }

        if (strcmp(argv[0], "disable") == 0 && argc > 1) {
            if (instance->_manager->setModuleEnabled(argv[1], false)) {
                PX4_INFO("Module %s disabled", argv[1]);
                return 0;
            } else {
                PX4_ERR("Failed to disable module %s", argv[1]);
                return 1;
            }
        }
    }

    print_usage("unknown command");
    return 1;
}

int TractionControlApp::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Traction Control V2 System

This module provides advanced traction control for articulated wheel loaders using a modular architecture.
It coordinates multiple traction control strategies including slip estimation, predictive control,
terrain adaptation, and load-aware torque distribution.

### Implementation
The system uses a manager-module pattern where:
- TractionControlManager coordinates all modules
- Individual modules implement specific traction control aspects
- Strategies can be dynamically selected based on conditions
- Commands are fused using configurable algorithms

### Examples
Start the traction control system:
$ traction_control_v2 start

Show system status:
$ traction_control_v2 status

Enable/disable specific modules:
$ traction_control_v2 enable SlipEstimator
$ traction_control_v2 disable PredictiveTraction

Emergency stop:
$ traction_control_v2 emergency_stop

Reset the system:
$ traction_control_v2 reset
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("traction_control_v2", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Show system status");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset all modules");
    PRINT_MODULE_USAGE_COMMAND_DESCR("emergency_stop", "Activate emergency stop");
    PRINT_MODULE_USAGE_COMMAND_DESCR("enable <module>", "Enable specific module");
    PRINT_MODULE_USAGE_COMMAND_DESCR("disable <module>", "Disable specific module");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int traction_control_v2_main(int argc, char *argv[])
{
    return TractionControlApp::main(argc, argv);
}
