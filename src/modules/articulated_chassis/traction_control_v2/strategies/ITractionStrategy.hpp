#pragma once

#include "../interfaces/TractionState.hpp"
#include "../interfaces/TractionCommand.hpp"

namespace traction_control {

/**
 * @brief Interface for traction control strategies
 *
 * Strategies encapsulate different approaches to traction control
 * that can be selected based on current conditions.
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
    virtual bool apply(const TractionState& state, TractionCommand& command) = 0;

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
    virtual const char* getName() const = 0;

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

    /**
     * @brief Reset strategy state
     */
    virtual void reset() = 0;

    /**
     * @brief Get strategy type
     * @return Strategy type enumeration
     */
    enum StrategyType {
        EMERGENCY = 0,
        CONSERVATIVE = 1,
        BALANCED = 2,
        AGGRESSIVE = 3,
        ADAPTIVE = 4,
        PREDICTIVE = 5,
        TERRAIN_SPECIFIC = 6
    };

    virtual StrategyType getType() const = 0;
};

/**
 * @brief Base class for traction strategies
 */
class TractionStrategyBase : public ITractionStrategy {
public:
    explicit TractionStrategyBase(const char* name, StrategyType type, int priority);
    virtual ~TractionStrategyBase() = default;

    const char* getName() const override { return _name; }
    int getPriority() const override { return _priority; }
    StrategyType getType() const override { return _type; }
    void reset() override { doReset(); }

protected:
    /**
     * @brief Strategy-specific reset implementation
     */
    virtual void doReset() {}

    /**
     * @brief Check basic applicability conditions
     * @param state Current traction state
     * @return true if basic conditions are met
     */
    bool checkBasicConditions(const TractionState& state) const;

    /**
     * @brief Apply safety limits to command
     * @param command Command to limit
     */
    void applySafetyLimits(TractionCommand& command) const;

    /**
     * @brief Calculate wheel slip from state
     * @param state Current traction state
     * @param wheel_index Wheel index (0-3)
     * @return Slip ratio for specified wheel
     */
    float calculateWheelSlip(const TractionState& state, int wheel_index) const;

    /**
     * @brief Calculate vehicle stability metric
     * @param state Current traction state
     * @return Stability metric [0, 1] where 1 is most stable
     */
    float calculateStabilityMetric(const TractionState& state) const;

private:
    const char* _name;
    StrategyType _type;
    int _priority;
};

} // namespace traction_control
