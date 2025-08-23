#include "TractionControlModuleBase.hpp"
#include <px4_platform_common/log.h>
#include <cstdarg>

namespace traction_control {

TractionControlModuleBase::TractionControlModuleBase(const char* module_name, const char* version)
    : ModuleParams(nullptr)
    , _module_name(module_name)
    , _version(version)
{
    // Initialize performance counter
    char perf_name[64];
    snprintf(perf_name, sizeof(perf_name), "tc_%s", module_name);
    _perf_counter = perf_alloc(PC_ELAPSED, perf_name);

    // Initialize metrics
    _metrics.timestamp = hrt_absolute_time();
    _init_time = _metrics.timestamp;
}

TractionControlModuleBase::~TractionControlModuleBase()
{
    if (_perf_counter) {
        perf_free(_perf_counter);
        _perf_counter = nullptr;
    }
}

bool TractionControlModuleBase::initialize()
{
    if (_initialized) {
        return true;
    }

    logActivity(6, "Initializing module %s v%s", _module_name, _version); // PX4_LOG_LEVEL_INFO

    // Update parameters first
    if (!updateParameters()) {
        logActivity(4, "Failed to update parameters during initialization"); // PX4_LOG_LEVEL_ERROR
        return false;
    }

    // Call module-specific initialization
    if (!doInitialize()) {
        logActivity(4, "Module-specific initialization failed"); // PX4_LOG_LEVEL_ERROR
        return false;
    }

    _initialized = true;
    _metrics.timestamp = hrt_absolute_time();

    logActivity(6, "Module %s initialized successfully", _module_name); // PX4_LOG_LEVEL_INFO
    return true;
}

bool TractionControlModuleBase::process(const TractionState& state, TractionCommand& command)
{
    if (!_initialized) {
        logActivity(4, "Attempting to process with uninitialized module"); // PX4_LOG_LEVEL_ERROR
        return false;
    }

    if (!_enabled) {
        return true; // Module disabled, but not an error
    }

    uint64_t start_time = hrt_absolute_time();

    // Check if we have too many recent errors
    if (_error_count >= MAX_ERROR_COUNT) {
        resetErrorCount();
        if (_error_count >= MAX_ERROR_COUNT) {
            logActivity(4, "Module has too many errors, skipping processing"); // PX4_LOG_LEVEL_ERROR
            return false;
        }
    }

    // Check data freshness
    if (!isDataFresh(state)) {
        logActivity(5, "Input data is too old for processing"); // PX4_LOG_LEVEL_WARN
        _error_count++;
        _last_error_time = start_time;
        return false;
    }

    // Start performance measurement
    perf_begin(_perf_counter);

    // Call module-specific processing
    bool success = false;
    try {
        success = doProcess(state, command);
    } catch (...) {
        logActivity(4, "Exception occurred during processing"); // PX4_LOG_LEVEL_ERROR
        success = false;
    }

    // End performance measurement
    perf_end(_perf_counter);

    uint64_t processing_time = hrt_absolute_time() - start_time;
    updateMetrics(processing_time, success);

    if (!success) {
        _error_count++;
        _last_error_time = start_time;
        logActivity(5, "Processing failed"); // PX4_LOG_LEVEL_WARN
    }

    _last_process_time = start_time;
    return success;
}

bool TractionControlModuleBase::updateParameters()
{
    try {
        updateParams(); // ModuleParams base class method
        return doUpdateParameters();
    } catch (...) {
        logActivity(4, "Failed to update parameters"); // PX4_LOG_LEVEL_ERROR
        return false;
    }
}

void TractionControlModuleBase::reset()
{
    logActivity(6, "Resetting module %s", _module_name); // PX4_LOG_LEVEL_INFO

    // Reset metrics
    _metrics = {};
    _metrics.timestamp = hrt_absolute_time();
    _metrics.last_reset_time = _metrics.timestamp;

    // Reset error tracking
    _error_count = 0;
    _last_error_time = 0;

    // Reset performance counter
    if (_perf_counter) {
        perf_reset(_perf_counter);
    }

    // Call module-specific reset
    doReset();
}

void TractionControlModuleBase::updateMetrics(uint64_t processing_time_us, bool success)
{
    _metrics.timestamp = hrt_absolute_time();
    _metrics.process_count++;

    if (success) {
        _metrics.success_count++;
    } else {
        _metrics.error_count++;
    }

    // Update timing metrics
    float time_ms = processing_time_us / 1000.0f;
    _metrics.avg_processing_time_us =
        (_metrics.avg_processing_time_us * (_metrics.process_count - 1) + processing_time_us) / _metrics.process_count;

    if (processing_time_us > _metrics.max_processing_time_us) {
        _metrics.max_processing_time_us = processing_time_us;
    }

    // Calculate effectiveness score
    if (_metrics.process_count > 0) {
        _metrics.effectiveness_score =
            (float)_metrics.success_count / (float)_metrics.process_count;
    }
}

bool TractionControlModuleBase::isDataFresh(const TractionState& state, uint64_t max_age_us) const
{
    uint64_t current_time = hrt_absolute_time();
    return (current_time - state.timestamp) <= max_age_us;
}

void TractionControlModuleBase::logActivity(int level, const char* format, ...) const
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Prepend module name to log message
    char full_message[320];
    snprintf(full_message, sizeof(full_message), "[%s] %s", _module_name, buffer);

    // Use PX4 logging based on level
    switch (level) {
    case 3: // PX4_LOG_LEVEL_PANIC
    case 4: // PX4_LOG_LEVEL_ERROR
        PX4_ERR("%s", full_message);
        break;
    case 5: // PX4_LOG_LEVEL_WARN
        PX4_WARN("%s", full_message);
        break;
    case 6: // PX4_LOG_LEVEL_INFO
        PX4_INFO("%s", full_message);
        break;
    case 7: // PX4_LOG_LEVEL_DEBUG
    default:
        PX4_DEBUG("%s", full_message);
        break;
    }
}

void TractionControlModuleBase::resetErrorCount()
{
    uint64_t current_time = hrt_absolute_time();
    if (_last_error_time > 0 && (current_time - _last_error_time) >= ERROR_RESET_TIME_US) {
        _error_count = 0;
        logActivity(6, "Error count reset due to time elapsed"); // PX4_LOG_LEVEL_INFO
    }
}

// PerformanceMonitor implementation

PerformanceMonitor::PerformanceMonitor(const char* name) : _name(name)
{
    char perf_name[64];
    snprintf(perf_name, sizeof(perf_name), "perf_%s", name);
    _perf_elapsed = perf_alloc(PC_ELAPSED, perf_name);
}

PerformanceMonitor::~PerformanceMonitor()
{
    if (_perf_elapsed) {
        perf_free(_perf_elapsed);
    }
}

void PerformanceMonitor::startTiming()
{
    _start_time = hrt_absolute_time();
    if (_perf_elapsed) {
        perf_begin(_perf_elapsed);
    }
}

uint64_t PerformanceMonitor::stopTiming()
{
    uint64_t elapsed = hrt_absolute_time() - _start_time;

    if (_perf_elapsed) {
        perf_end(_perf_elapsed);
    }

    _total_time += elapsed;
    _count++;

    if (elapsed > _max_time) {
        _max_time = elapsed;
    }

    return elapsed;
}

float PerformanceMonitor::getAverageTime() const
{
    return _count > 0 ? (float)_total_time / (float)_count : 0.0f;
}

float PerformanceMonitor::getMaxTime() const
{
    return (float)_max_time;
}

void PerformanceMonitor::reset()
{
    _total_time = 0;
    _max_time = 0;
    _count = 0;

    if (_perf_elapsed) {
        perf_reset(_perf_elapsed);
    }
}

} // namespace traction_control
