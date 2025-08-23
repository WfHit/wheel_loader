#pragma once

#include "TractionState.hpp"
#include "TractionCommand.hpp"

namespace traction_control {

/**
 * @brief Interface for state observers
 *
 * Observers are notified when the traction state changes.
 */
class ITractionStateObserver {
public:
    virtual ~ITractionStateObserver() = default;

    /**
     * @brief Called when traction state is updated
     * @param state New traction state
     */
    virtual void onStateUpdate(const TractionState& state) = 0;

    /**
     * @brief Called when an emergency condition is detected
     * @param state Current traction state
     */
    virtual void onEmergencyCondition(const TractionState& state) = 0;
};

/**
 * @brief Interface for command observers
 *
 * Observers are notified when traction commands are generated.
 */
class ITractionCommandObserver {
public:
    virtual ~ITractionCommandObserver() = default;

    /**
     * @brief Called when a new traction command is generated
     * @param command New traction command
     */
    virtual void onCommandUpdate(const TractionCommand& command) = 0;

    /**
     * @brief Called when command validation fails
     * @param command Invalid command
     * @param error_message Error description
     */
    virtual void onCommandError(const TractionCommand& command, const char* error_message) = 0;
};

/**
 * @brief Interface for data providers
 *
 * Data providers supply information to the traction control system.
 */
class ITractionDataProvider {
public:
    virtual ~ITractionDataProvider() = default;

    /**
     * @brief Get current traction state data
     * @param state Output state structure
     * @return true if data is available and valid
     */
    virtual bool getState(TractionState& state) = 0;

    /**
     * @brief Check if data provider is available
     * @return true if provider can supply data
     */
    virtual bool isAvailable() const = 0;

    /**
     * @brief Get data provider name
     * @return Provider name string
     */
    virtual const char* getProviderName() const = 0;

    /**
     * @brief Get data update rate
     * @return Update rate in Hz
     */
    virtual float getUpdateRate() const = 0;
};

/**
 * @brief Interface for command executors
 *
 * Command executors apply traction commands to the vehicle systems.
 */
class ITractionCommandExecutor {
public:
    virtual ~ITractionCommandExecutor() = default;

    /**
     * @brief Execute traction command
     * @param command Command to execute
     * @return true if command was executed successfully
     */
    virtual bool executeCommand(const TractionCommand& command) = 0;

    /**
     * @brief Check if executor is ready
     * @return true if executor can accept commands
     */
    virtual bool isReady() const = 0;

    /**
     * @brief Get executor name
     * @return Executor name string
     */
    virtual const char* getExecutorName() const = 0;

    /**
     * @brief Get maximum command rate
     * @return Maximum command rate in Hz
     */
    virtual float getMaxCommandRate() const = 0;

    /**
     * @brief Emergency stop - immediately halt all controlled actuators
     */
    virtual void emergencyStop() = 0;
};

/**
 * @brief Interface for traction strategies
 *
 * Strategies define how traction control should behave under different conditions.
 */
class ITractionStrategy {
public:
    virtual ~ITractionStrategy() = default;

    /**
     * @brief Calculate traction control response
     * @param state Current traction state
     * @param command Output traction command
     * @return true if strategy was applied successfully
     */
    virtual bool calculateResponse(const TractionState& state, TractionCommand& command) = 0;

    /**
     * @brief Check if strategy is applicable for current conditions
     * @param state Current traction state
     * @return true if strategy can be applied
     */
    virtual bool isApplicable(const TractionState& state) const = 0;

    /**
     * @brief Get strategy name
     * @return Strategy name string
     */
    virtual const char* getStrategyName() const = 0;

    /**
     * @brief Get strategy priority
     * @return Strategy priority (higher number = higher priority)
     */
    virtual int getPriority() const = 0;

    /**
     * @brief Get confidence in strategy's output
     * @param state Current traction state
     * @return Confidence level [0, 1]
     */
    virtual float getConfidence(const TractionState& state) const = 0;
};

} // namespace traction_control
