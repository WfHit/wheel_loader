#pragma once

#include "../interfaces/TractionState.hpp"
#include "../interfaces/TractionCommand.hpp"
#include <lib/matrix/matrix/Matrix.hpp>
#include <cstdint>

namespace traction_control {

/**
 * @brief State estimator utility for traction control
 *
 * Provides common state estimation functionality used across modules.
 */
class StateEstimator {
public:
    StateEstimator();
    ~StateEstimator() = default;

    /**
     * @brief Estimate vehicle velocity from wheel speeds
     * @param wheel_speeds Array of wheel speeds [rad/s]
     * @param wheel_radius Wheel radius [m]
     * @param slip_ratios Array of slip ratios (optional)
     * @return Estimated vehicle velocity [m/s]
     */
    float estimateVehicleVelocity(const float wheel_speeds[4],
                                 float wheel_radius,
                                 const float slip_ratios[4] = nullptr) const;

    /**
     * @brief Estimate slip ratios from wheel and vehicle speeds
     * @param wheel_speeds Array of wheel speeds [rad/s]
     * @param vehicle_velocity Vehicle velocity [m/s]
     * @param wheel_radius Wheel radius [m]
     * @param slip_ratios Output slip ratios
     */
    void estimateSlipRatios(const float wheel_speeds[4],
                           float vehicle_velocity,
                           float wheel_radius,
                           float slip_ratios[4]) const;

    /**
     * @brief Estimate lateral slip angle
     * @param velocity_x Longitudinal velocity [m/s]
     * @param velocity_y Lateral velocity [m/s]
     * @return Lateral slip angle [rad]
     */
    float estimateLateralSlip(float velocity_x, float velocity_y) const;

    /**
     * @brief Estimate surface friction coefficient
     * @param normal_force Normal force [N]
     * @param lateral_force Lateral force [N]
     * @return Friction coefficient estimate
     */
    float estimateFrictionCoefficient(float normal_force, float lateral_force) const;

    /**
     * @brief Estimate vehicle stability metric
     * @param state Current traction state
     * @return Stability metric [0, 1] where 1 is most stable
     */
    float estimateStability(const TractionState& state) const;

private:
    static constexpr float MIN_VELOCITY_THRESHOLD = 0.1f; // m/s
    static constexpr float MAX_SLIP_RATIO = 1.0f;
    static constexpr float MAX_FRICTION_COEFF = 1.5f;
};

/**
 * @brief Command fusion utility
 *
 * Combines commands from multiple modules intelligently.
 */
class CommandFusion {
public:
    CommandFusion();
    ~CommandFusion() = default;

    /**
     * @brief Fuse multiple traction commands
     * @param commands Array of input commands
     * @param command_count Number of input commands
     * @param weights Weights for each command (optional)
     * @param output_command Fused output command
     * @return true if fusion successful
     */
    bool fuseCommands(const TractionCommand* commands,
                     int command_count,
                     const float* weights,
                     TractionCommand& output_command) const;

    /**
     * @brief Priority-based command fusion
     * @param commands Array of input commands
     * @param command_count Number of input commands
     * @param priorities Priority for each command
     * @param output_command Fused output command
     * @return true if fusion successful
     */
    bool priorityBasedFusion(const TractionCommand* commands,
                           int command_count,
                           const int* priorities,
                           TractionCommand& output_command) const;

    /**
     * @brief Weighted average fusion
     * @param commands Array of input commands
     * @param command_count Number of input commands
     * @param weights Weights for each command
     * @param output_command Fused output command
     * @return true if fusion successful
     */
    bool weightedAverageFusion(const TractionCommand* commands,
                             int command_count,
                             const float* weights,
                             TractionCommand& output_command) const;

    /**
     * @brief Confidence-weighted fusion
     * @param commands Array of input commands
     * @param command_count Number of input commands
     * @param confidences Confidence for each command [0, 1]
     * @param output_command Fused output command
     * @return true if fusion successful
     */
    bool confidenceWeightedFusion(const TractionCommand* commands,
                                int command_count,
                                const float* confidences,
                                TractionCommand& output_command) const;

private:
    void fuseFlags(const TractionCommand* commands, int count, TractionCommand& output) const;
    void fuseParameters(const TractionCommand* commands, int count, const float* weights, TractionCommand& output) const;
    void fusePerformanceSettings(const TractionCommand* commands, int count, const float* weights, TractionCommand& output) const;
};

/**
 * @brief Safety validator for traction commands
 *
 * Validates and applies safety limits to traction commands.
 */
class SafetyValidator {
public:
    struct SafetyLimits {
        float max_wheel_torque_nm{2000.0f};      // Maximum wheel torque [Nm]
        float max_brake_pressure_bar{200.0f};    // Maximum brake pressure [bar]
        float max_steering_angle_rad{0.5f};      // Maximum steering correction [rad]
        float max_torque_rate_nm_s{500.0f};      // Maximum torque rate [Nm/s]
        float max_steering_rate_rad_s{2.0f};     // Maximum steering rate [rad/s]
        float min_slip_ratio{0.0f};              // Minimum allowed slip ratio
        float max_slip_ratio{0.3f};              // Maximum allowed slip ratio
        float stability_margin{0.1f};            // Required stability margin
    };

    SafetyValidator();
    explicit SafetyValidator(const SafetyLimits& limits);
    ~SafetyValidator() = default;

    /**
     * @brief Validate traction command for safety
     * @param command Command to validate
     * @return true if command is safe
     */
    bool validateCommand(const TractionCommand& command) const;

    /**
     * @brief Apply safety limits to traction command
     * @param command Command to limit (modified in place)
     * @return true if limits were applied successfully
     */
    bool applySafetyLimits(TractionCommand& command) const;

    /**
     * @brief Check if intervention is safe given current state
     * @param state Current traction state
     * @param command Proposed command
     * @return true if intervention is safe
     */
    bool isInterventionSafe(const TractionState& state, const TractionCommand& command) const;

    /**
     * @brief Update safety limits
     * @param limits New safety limits
     */
    void updateLimits(const SafetyLimits& limits) { _limits = limits; }

    /**
     * @brief Get current safety limits
     * @return Current safety limits
     */
    const SafetyLimits& getLimits() const { return _limits; }

private:
    SafetyLimits _limits;

    bool checkTorqueLimits(const TractionCommand& command) const;
    bool checkBrakeLimits(const TractionCommand& command) const;
    bool checkSteeringLimits(const TractionCommand& command) const;
    bool checkRateLimits(const TractionCommand& command) const;
    bool checkStabilityMargin(const TractionState& state, const TractionCommand& command) const;
};

/**
 * @brief Performance monitor for traction control system
 *
 * Tracks and analyzes traction control performance.
 */
class PerformanceMonitor {
public:
    struct PerformanceMetrics {
        // Effectiveness metrics
        float traction_utilization{0.0f};      // Average traction utilization [0, 1]
        float stability_score{1.0f};           // Average stability score [0, 1]
        float intervention_success_rate{0.0f}; // Success rate of interventions [0, 1]

        // Efficiency metrics
        float energy_efficiency{1.0f};         // Energy efficiency factor [0, 1]
        float response_time_ms{0.0f};          // Average response time [ms]
        float computational_load{0.0f};        // Computational load [0, 1]

        // Safety metrics
        float safety_margin{1.0f};             // Average safety margin [0, 1]
        uint32_t safety_violations{0};         // Number of safety violations
        uint32_t emergency_stops{0};           // Number of emergency stops

        // Overall metrics
        float overall_performance{0.0f};       // Overall performance score [0, 1]
        uint64_t uptime_us{0};                 // System uptime [microseconds]
        uint64_t last_update_time{0};          // Last update timestamp
    };

    PerformanceMonitor();
    ~PerformanceMonitor() = default;

    /**
     * @brief Update performance metrics
     * @param state Current traction state
     * @param command Applied traction command
     * @param processing_time_us Processing time [microseconds]
     */
    void update(const TractionState& state,
               const TractionCommand& command,
               uint64_t processing_time_us);

    /**
     * @brief Get current performance metrics
     * @return Current performance metrics
     */
    const PerformanceMetrics& getMetrics() const { return _metrics; }

    /**
     * @brief Reset performance metrics
     */
    void reset();

    /**
     * @brief Check if performance is acceptable
     * @return true if performance is within acceptable bounds
     */
    bool isPerformanceAcceptable() const;

    /**
     * @brief Get performance score for specific aspect
     * @param aspect Performance aspect to query
     * @return Performance score [0, 1]
     */
    enum PerformanceAspect {
        EFFECTIVENESS,
        EFFICIENCY,
        SAFETY,
        OVERALL
    };
    float getPerformanceScore(PerformanceAspect aspect) const;

private:
    PerformanceMetrics _metrics;
    uint32_t _update_count{0};
    uint64_t _start_time{0};

    static constexpr float MIN_ACCEPTABLE_PERFORMANCE = 0.7f;
    static constexpr float METRIC_ALPHA = 0.95f; // Exponential moving average factor

    void updateTractionUtilization(const TractionState& state);
    void updateStabilityScore(const TractionState& state);
    void updateInterventionSuccess(const TractionCommand& command);
    void updateEfficiencyMetrics(uint64_t processing_time_us);
    void updateSafetyMetrics(const TractionState& state, const TractionCommand& command);
    void calculateOverallPerformance();
};

/**
 * @brief Data validator for traction control inputs
 *
 * Validates sensor data and state information for quality and consistency.
 */
class DataValidator {
public:
    struct ValidationCriteria {
        float max_velocity_ms{50.0f};          // Maximum reasonable velocity [m/s]
        float max_acceleration_ms2{20.0f};     // Maximum reasonable acceleration [m/s²]
        float max_yaw_rate_rad_s{2.0f};        // Maximum reasonable yaw rate [rad/s]
        float max_wheel_speed_rad_s{100.0f};   // Maximum reasonable wheel speed [rad/s]
        float max_data_age_us{200000};         // Maximum acceptable data age [microseconds]
        float min_confidence{0.3f};            // Minimum acceptable confidence
    };

    DataValidator();
    explicit DataValidator(const ValidationCriteria& criteria);
    ~DataValidator() = default;

    /**
     * @brief Validate traction state data
     * @param state State to validate
     * @return true if state data is valid
     */
    bool validateState(const TractionState& state) const;

    /**
     * @brief Validate individual sensor data
     * @param timestamp Data timestamp
     * @param value Data value
     * @param min_value Minimum acceptable value
     * @param max_value Maximum acceptable value
     * @return true if data is valid
     */
    bool validateSensorData(uint64_t timestamp, float value, float min_value, float max_value) const;

    /**
     * @brief Check data freshness
     * @param timestamp Data timestamp
     * @param max_age_us Maximum acceptable age [microseconds]
     * @return true if data is fresh
     */
    bool isDataFresh(uint64_t timestamp, uint64_t max_age_us) const;

    /**
     * @brief Update validation criteria
     * @param criteria New validation criteria
     */
    void updateCriteria(const ValidationCriteria& criteria) { _criteria = criteria; }

private:
    ValidationCriteria _criteria;

    bool validateDynamics(const TractionState::VehicleDynamics& dynamics) const;
    bool validateWheels(const TractionState::WheelState& wheels) const;
    bool validateSurface(const TractionState::SurfaceConditions& surface) const;
    bool validateLoad(const TractionState::LoadInformation& load) const;
    bool validateTrajectory(const TractionState::TrajectoryInformation& trajectory) const;
};

} // namespace traction_control
