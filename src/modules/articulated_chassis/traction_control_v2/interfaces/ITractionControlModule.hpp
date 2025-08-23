#pragma once

#include <memory>
#include "TractionState.hpp"
#include "TractionCommand.hpp"

namespace traction_control {

/**
 * @brief Interface for all traction control modules
 *
 * This interface ensures that all traction control modules have a consistent
 * interface for initialization, processing, and cleanup.
 */
class ITractionControlModule {
public:
    virtual ~ITractionControlModule() = default;

    /**
     * @brief Initialize the module
     * @return true if initialization successful, false otherwise
     */
    virtual bool initialize() = 0;

    /**
     * @brief Process traction control logic
     * @param state Current traction state
     * @param command Output traction command
     * @return true if processing successful, false otherwise
     */
    virtual bool process(const TractionState& state, TractionCommand& command) = 0;

    /**
     * @brief Update module parameters
     * @return true if update successful, false otherwise
     */
    virtual bool updateParameters() = 0;

    /**
     * @brief Get module name
     * @return Module name string
     */
    virtual const char* getName() const = 0;

    /**
     * @brief Get module version
     * @return Module version string
     */
    virtual const char* getVersion() const = 0;

    /**
     * @brief Check if module is enabled
     * @return true if enabled, false otherwise
     */
    virtual bool isEnabled() const = 0;

    /**
     * @brief Set module enabled state
     * @param enabled Module enabled state
     */
    virtual void setEnabled(bool enabled) = 0;

    /**
     * @brief Get module priority (higher number = higher priority)
     * @return Module priority value
     */
    virtual int getPriority() const = 0;

    /**
     * @brief Reset module state
     */
    virtual void reset() = 0;

    /**
     * @brief Get performance metrics
     * @return Performance metrics structure
     */
    virtual TractionModuleMetrics getMetrics() const = 0;
};

/**
 * @brief Smart pointer type for traction control modules
 */
using TractionControlModulePtr = std::shared_ptr<ITractionControlModule>;

} // namespace traction_control
