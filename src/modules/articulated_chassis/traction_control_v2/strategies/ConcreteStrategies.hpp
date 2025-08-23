#pragma once

#include "ITractionStrategy.hpp"
#include <lib/mathlib/mathlib.h>

namespace traction_control {

/**
 * @brief Emergency traction control strategy
 *
 * Used when immediate intervention is required to prevent loss of control.
 * Prioritizes safety over performance.
 */
class EmergencyTractionStrategy : public TractionStrategyBase {
public:
    EmergencyTractionStrategy();
    ~EmergencyTractionStrategy() override = default;

    bool apply(const TractionState& state, TractionCommand& command) override;
    bool isApplicable(const TractionState& state) const override;
    float getConfidence(const TractionState& state) const override;

private:
    static constexpr float EMERGENCY_SLIP_THRESHOLD = 0.3f;
    static constexpr float EMERGENCY_STABILITY_THRESHOLD = 0.2f;
    static constexpr float MAX_EMERGENCY_TORQUE = 500.0f; // Nm
    static constexpr float EMERGENCY_BRAKE_PRESSURE = 50.0f; // bar

    void applyEmergencyBraking(TractionCommand& command) const;
    void limitTorqueOutput(TractionCommand& command) const;
    void applyStabilityCorrection(const TractionState& state, TractionCommand& command) const;
};

/**
 * @brief Conservative traction control strategy
 *
 * Emphasizes smooth, predictable behavior with gentle interventions.
 * Good for normal driving conditions and inexperienced operators.
 */
class ConservativeTractionStrategy : public TractionStrategyBase {
public:
    ConservativeTractionStrategy();
    ~ConservativeTractionStrategy() override = default;

    bool apply(const TractionState& state, TractionCommand& command) override;
    bool isApplicable(const TractionState& state) const override;
    float getConfidence(const TractionState& state) const override;

private:
    static constexpr float CONSERVATIVE_SLIP_THRESHOLD = 0.1f;
    static constexpr float CONSERVATIVE_RESPONSE_GAIN = 0.5f;
    static constexpr float MAX_TORQUE_REDUCTION = 0.3f;

    void applyGradualIntervention(const TractionState& state, TractionCommand& command) const;
    void smoothTorqueTransition(TractionCommand& command) const;
};

/**
 * @brief Balanced traction control strategy
 *
 * Provides a good compromise between performance and safety.
 * Suitable for most operating conditions.
 */
class BalancedTractionStrategy : public TractionStrategyBase {
public:
    BalancedTractionStrategy();
    ~BalancedTractionStrategy() override = default;

    bool apply(const TractionState& state, TractionCommand& command) override;
    bool isApplicable(const TractionState& state) const override;
    float getConfidence(const TractionState& state) const override;

private:
    static constexpr float BALANCED_SLIP_THRESHOLD = 0.15f;
    static constexpr float BALANCED_RESPONSE_GAIN = 0.7f;
    static constexpr float EFFICIENCY_WEIGHT = 0.4f;
    static constexpr float SAFETY_WEIGHT = 0.6f;

    void balancePerformanceAndSafety(const TractionState& state, TractionCommand& command) const;
    void optimizeForEfficiency(const TractionState& state, TractionCommand& command) const;
};

/**
 * @brief Aggressive traction control strategy
 *
 * Maximizes traction and performance, accepting higher slip levels.
 * Suitable for experienced operators and challenging terrain.
 */
class AggressiveTractionStrategy : public TractionStrategyBase {
public:
    AggressiveTractionStrategy();
    ~AggressiveTractionStrategy() override = default;

    bool apply(const TractionState& state, TractionCommand& command) override;
    bool isApplicable(const TractionState& state) const override;
    float getConfidence(const TractionState& state) const override;

private:
    static constexpr float AGGRESSIVE_SLIP_THRESHOLD = 0.25f;
    static constexpr float AGGRESSIVE_RESPONSE_GAIN = 1.2f;
    static constexpr float HIGH_PERFORMANCE_TORQUE_BIAS = 0.7f;

    void maximizeTractionUtilization(const TractionState& state, TractionCommand& command) const;
    void applyPerformanceOptimization(const TractionState& state, TractionCommand& command) const;
};

/**
 * @brief Terrain-specific traction control strategy
 *
 * Adapts behavior based on detected surface conditions.
 * Optimizes for specific terrain types.
 */
class TerrainSpecificStrategy : public TractionStrategyBase {
public:
    TerrainSpecificStrategy();
    ~TerrainSpecificStrategy() override = default;

    bool apply(const TractionState& state, TractionCommand& command) override;
    bool isApplicable(const TractionState& state) const override;
    float getConfidence(const TractionState& state) const override;

private:
    struct TerrainSettings {
        float slip_threshold;
        float response_gain;
        float torque_bias;
        float differential_lock_preference;
        float brake_bias;
    };

    static const TerrainSettings ASPHALT_SETTINGS;
    static const TerrainSettings GRAVEL_SETTINGS;
    static const TerrainSettings MUD_SETTINGS;
    static const TerrainSettings SAND_SETTINGS;
    static const TerrainSettings SNOW_SETTINGS;
    static const TerrainSettings ICE_SETTINGS;

    TerrainSettings getSettingsForSurface(TractionState::SurfaceConditions::SurfaceType surface) const;
    void applyTerrainSpecificControl(const TractionState& state,
                                   TractionCommand& command,
                                   const TerrainSettings& settings) const;
};

/**
 * @brief Load-aware traction control strategy
 *
 * Adjusts traction control behavior based on vehicle load conditions.
 */
class LoadAwareTractionStrategy : public TractionStrategyBase {
public:
    LoadAwareTractionStrategy();
    ~LoadAwareTractionStrategy() override = default;

    bool apply(const TractionState& state, TractionCommand& command) override;
    bool isApplicable(const TractionState& state) const override;
    float getConfidence(const TractionState& state) const override;

private:
    void adjustForLoadConditions(const TractionState& state, TractionCommand& command) const;
    void compensateForCGShift(const TractionState& state, TractionCommand& command) const;
    float calculateLoadFactor(const TractionState& state) const;
};

/**
 * @brief Strategy selector utility
 *
 * Helps select the most appropriate strategy based on current conditions.
 */
class StrategySelector {
public:
    StrategySelector();

    /**
     * @brief Select best strategy for current conditions
     * @param state Current traction state
     * @param available_strategies Available strategies to choose from
     * @return Best strategy or nullptr if none suitable
     */
    std::shared_ptr<ITractionStrategy> selectBestStrategy(
        const TractionState& state,
        const std::vector<std::shared_ptr<ITractionStrategy>>& available_strategies) const;

    /**
     * @brief Get strategy priority for current conditions
     * @param strategy Strategy to evaluate
     * @param state Current traction state
     * @return Priority score (higher is better)
     */
    float calculateStrategyPriority(
        const std::shared_ptr<ITractionStrategy>& strategy,
        const TractionState& state) const;

private:
    float evaluateEmergencyNeed(const TractionState& state) const;
    float evaluateTerrainMatch(const TractionState& state, ITractionStrategy::StrategyType type) const;
    float evaluateLoadMatch(const TractionState& state, ITractionStrategy::StrategyType type) const;
};

} // namespace traction_control
