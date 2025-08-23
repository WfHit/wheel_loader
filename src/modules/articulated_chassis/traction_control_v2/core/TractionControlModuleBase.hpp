#pragma once

#include "../interfaces/ITractionControlModule.hpp"
#include "../interfaces/ITractionInterfaces.hpp"
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

namespace traction_control {

/**
 * @brief Base class for all traction control modules
 *
 * Provides common functionality for parameter handling, performance monitoring,
 * and basic module lifecycle management.
 */
class TractionControlModuleBase : public ITractionControlModule, public ModuleParams {
public:
    explicit TractionControlModuleBase(const char* module_name, const char* version = "1.0.0");
    virtual ~TractionControlModuleBase();

    // ITractionControlModule interface
    bool initialize() override;
    bool process(const TractionState& state, TractionCommand& command) override;
    bool updateParameters() override;
    const char* getName() const override { return _module_name; }
    const char* getVersion() const override { return _version; }
    bool isEnabled() const override { return _enabled; }
    void setEnabled(bool enabled) override { _enabled = enabled; }
    int getPriority() const override { return _priority; }
    void reset() override;
    TractionModuleMetrics getMetrics() const override { return _metrics; }

protected:
    /**
     * @brief Module-specific initialization
     * Override this method to implement module-specific initialization logic.
     * @return true if initialization successful
     */
    virtual bool doInitialize() = 0;

    /**
     * @brief Module-specific processing
     * Override this method to implement the core module logic.
     * @param state Current traction state
     * @param command Output traction command
     * @return true if processing successful
     */
    virtual bool doProcess(const TractionState& state, TractionCommand& command) = 0;

    /**
     * @brief Module-specific parameter update
     * Override this method to handle module-specific parameter updates.
     * @return true if update successful
     */
    virtual bool doUpdateParameters() { return true; }

    /**
     * @brief Module-specific reset
     * Override this method to implement module-specific reset logic.
     */
    virtual void doReset() {}

    /**
     * @brief Update performance metrics
     * @param processing_time_us Processing time in microseconds
     * @param success True if processing was successful
     */
    void updateMetrics(uint64_t processing_time_us, bool success);

    /**
     * @brief Check if state data is fresh enough for processing
     * @param state Traction state to check
     * @param max_age_us Maximum acceptable age in microseconds
     * @return true if data is fresh enough
     */
    bool isDataFresh(const TractionState& state, uint64_t max_age_us = 100000) const;

    /**
     * @brief Log module activity
     * @param level Log level (PX4_LOG_LEVEL_*)
     * @param format Printf-style format string
     * @param ... Printf-style arguments
     */
    void logActivity(int level, const char* format, ...) const;

    // Common parameters
    const char* _module_name;
    const char* _version;
    bool _enabled{true};
    int _priority{0};

    // Performance monitoring
    TractionModuleMetrics _metrics{};
    perf_counter_t _perf_counter{nullptr};
    uint64_t _last_process_time{0};
    uint64_t _init_time{0};

    // State tracking
    bool _initialized{false};
    uint32_t _error_count{0};
    uint64_t _last_error_time{0};

private:
    static constexpr uint32_t MAX_ERROR_COUNT = 100;
    static constexpr uint64_t ERROR_RESET_TIME_US = 10000000; // 10 seconds

    void resetErrorCount();
};

/**
 * @brief Template base class for modules with specific parameter types
 *
 * This template class provides type-safe parameter handling for modules
 * that need to define their own parameter structures.
 */
template<typename ParamStruct>
class ParameterizedTractionModule : public TractionControlModuleBase {
public:
    explicit ParameterizedTractionModule(const char* module_name, const char* version = "1.0.0")
        : TractionControlModuleBase(module_name, version) {}

protected:
    /**
     * @brief Get current parameter values
     * @return Reference to parameter structure
     */
    const ParamStruct& getParameters() const { return _params; }

    /**
     * @brief Update parameter structure from PX4 parameters
     * Override this method to implement parameter loading logic.
     */
    virtual void loadParameters() = 0;

    bool doUpdateParameters() override {
        try {
            loadParameters();
            return true;
        } catch (...) {
            logActivity(4, "Failed to update parameters"); // PX4_LOG_LEVEL_ERROR
            return false;
        }
    }

    ParamStruct _params{};
};

/**
 * @brief Utility macros for parameter definition
 */
#define TRACTION_PARAM_FLOAT(name, default_val) \
    (ParamFloat<px4::params::name>) _param_##name{default_val}

#define TRACTION_PARAM_INT(name, default_val) \
    (ParamInt<px4::params::name>) _param_##name{default_val}

#define TRACTION_PARAM_BOOL(name, default_val) \
    (ParamBool<px4::params::name>) _param_##name{default_val}

/**
 * @brief Performance monitoring helper
 */
class PerformanceMonitor {
public:
    explicit PerformanceMonitor(const char* name);
    ~PerformanceMonitor();

    void startTiming();
    uint64_t stopTiming();
    float getAverageTime() const;
    float getMaxTime() const;
    void reset();

private:
    const char* _name;
    uint64_t _start_time{0};
    uint64_t _total_time{0};
    uint64_t _max_time{0};
    uint32_t _count{0};
    perf_counter_t _perf_elapsed{nullptr};
};

} // namespace traction_control
