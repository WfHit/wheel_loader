#pragma once

#include "../core/TractionControlModuleBase.hpp"
#include "../strategies/ITractionStrategy.hpp"
#include <lib/matrix/matrix/Matrix.hpp>
#include <memory>

namespace traction_control {

/**
 * @brief Refactored predictive traction control module
 *
 * Uses Model Predictive Control (MPC) for advanced traction management with
 * improved modularity and separation of concerns.
 */
class PredictiveTractionModule : public ParameterizedTractionModule<PredictiveTractionModule::Parameters> {
public:
    struct Parameters {
        float prediction_horizon_s{1.0f};        // Prediction horizon [s]
        float control_horizon_s{0.5f};           // Control horizon [s]
        float prediction_dt{0.1f};               // Prediction time step [s]
        float slip_warning_threshold{0.15f};     // Slip warning threshold
        float slip_critical_threshold{0.25f};    // Slip critical threshold
        float stability_threshold{0.3f};         // Stability warning threshold
        float weight_slip_tracking{10.0f};       // MPC weight for slip tracking
        float weight_stability{20.0f};           // MPC weight for stability
        float weight_control_effort{1.0f};       // MPC weight for control effort
        float weight_control_rate{5.0f};         // MPC weight for control rate
        float max_torque_rate{50.0f};            // Maximum torque rate [Nm/s]
        float max_steering_rate{0.5f};           // Maximum steering rate [rad/s]
        bool learning_enabled{true};             // Enable learning
        float adaptation_rate{0.1f};             // Learning adaptation rate
        int max_iterations{50};                  // MPC solver max iterations
        float convergence_tolerance{1e-4f};      // MPC convergence tolerance
        float confidence_threshold{0.6f};        // Minimum confidence for output
    };

    PredictiveTractionModule();
    ~PredictiveTractionModule() override = default;

    const char* getName() const override { return "PredictiveTraction"; }
    int getPriority() const override { return 6; } // Medium-high priority

protected:
    bool doInitialize() override;
    bool doProcess(const TractionState& state, TractionCommand& command) override;
    void doReset() override;
    void loadParameters() override;

private:
    static constexpr int MAX_PREDICTION_STEPS = 20;
    static constexpr int MAX_CONTROL_STEPS = 10;

    /**
     * @brief Vehicle dynamics model for prediction
     */
    class VehicleDynamicsModel {
    public:
        VehicleDynamicsModel();

        void initialize(float wheelbase, float mass, float inertia);
        matrix::Vector<float, 6> predict(const matrix::Vector<float, 6>& state,
                                       const matrix::Vector<float, 3>& control,
                                       float dt) const;

        // State vector: [vx, vy, yaw_rate, wheel_speed_front, wheel_speed_rear, steering_angle]
        // Control vector: [torque_distribution, steering_correction, brake_force]

    private:
        float _wheelbase{2.5f};
        float _mass{12000.0f};
        float _inertia{8000.0f};
        float _tire_radius{0.4f};
        float _cornering_stiffness{50000.0f};
    };

    /**
     * @brief Slip prediction model
     */
    class SlipPredictor {
    public:
        SlipPredictor();

        void initialize(const Parameters& params);
        void predict(const matrix::Vector<float, 6>& state,
                    const matrix::Vector<float, 3>& control,
                    float friction_coeff,
                    float& slip_front,
                    float& slip_rear) const;

        float calculateSlipRisk(float slip_front, float slip_rear) const;

    private:
        Parameters _params;

        float calculateTireSlip(float wheel_speed, float vehicle_speed, float radius) const;
        float calculateLateralSlip(float lateral_velocity, float longitudinal_velocity) const;
    };

    /**
     * @brief MPC optimizer for traction control
     */
    class MPCOptimizer {
    public:
        MPCOptimizer();

        void initialize(const Parameters& params);
        bool solve(const matrix::Vector<float, 6>& initial_state,
                  const matrix::Vector<float, 6>& reference_state,
                  const VehicleDynamicsModel& model,
                  const SlipPredictor& slip_predictor,
                  float friction_coeff,
                  matrix::Matrix<float, 3, MAX_CONTROL_STEPS>& optimal_control);

        float getOptimizationCost() const { return _final_cost; }
        int getIterationCount() const { return _iterations; }
        bool hasConverged() const { return _converged; }

    private:
        Parameters _params;
        int _prediction_steps;
        int _control_steps;
        float _final_cost{0.0f};
        int _iterations{0};
        bool _converged{false};

        float evaluateCostFunction(const matrix::Vector<float, 6>& initial_state,
                                 const matrix::Vector<float, 6>& reference_state,
                                 const matrix::Matrix<float, 3, MAX_CONTROL_STEPS>& control_sequence,
                                 const VehicleDynamicsModel& model,
                                 const SlipPredictor& slip_predictor,
                                 float friction_coeff) const;

        bool checkConstraints(const matrix::Matrix<float, 3, MAX_CONTROL_STEPS>& control_sequence) const;

        void gradientDescent(const matrix::Vector<float, 6>& initial_state,
                           const matrix::Vector<float, 6>& reference_state,
                           const VehicleDynamicsModel& model,
                           const SlipPredictor& slip_predictor,
                           float friction_coeff,
                           matrix::Matrix<float, 3, MAX_CONTROL_STEPS>& control_sequence);
    };

    /**
     * @brief Learning and adaptation system
     */
    class AdaptationSystem {
    public:
        AdaptationSystem();

        void initialize(const Parameters& params);
        void update(const TractionState& state,
                   const TractionCommand& command,
                   float prediction_error);

        void adaptParameters(Parameters& params) const;
        float getAdaptationConfidence() const { return _confidence; }

    private:
        Parameters _base_params;
        float _confidence{0.5f};
        uint32_t _adaptation_count{0};

        struct AdaptationHistory {
            static constexpr int HISTORY_SIZE = 100;
            float prediction_errors[HISTORY_SIZE];
            float surface_friction[HISTORY_SIZE];
            float control_effectiveness[HISTORY_SIZE];
            int index{0};
            int count{0};
        } _history;

        void addToHistory(float prediction_error, float friction, float effectiveness);
        float calculateAdaptationFactor() const;
    };

    /**
     * @brief Risk assessment system
     */
    class RiskAssessment {
    public:
        struct RiskFactors {
            float slip_risk{0.0f};           // Risk from wheel slip
            float stability_risk{0.0f};      // Risk from stability issues
            float terrain_risk{0.0f};        // Risk from terrain conditions
            float load_risk{0.0f};           // Risk from load conditions
            float trajectory_risk{0.0f};     // Risk from planned trajectory
            float overall_risk{0.0f};        // Combined risk assessment
        };

        RiskAssessment();

        void initialize(const Parameters& params);
        RiskFactors assessRisk(const TractionState& state) const;

        bool requiresIntervention(const RiskFactors& risk) const;
        uint8_t getRecommendedInterventionLevel(const RiskFactors& risk) const;

    private:
        Parameters _params;

        float calculateSlipRisk(const TractionState& state) const;
        float calculateStabilityRisk(const TractionState& state) const;
        float calculateTerrainRisk(const TractionState& state) const;
        float calculateLoadRisk(const TractionState& state) const;
        float calculateTrajectoryRisk(const TractionState& state) const;
    };

    // Core components
    VehicleDynamicsModel _dynamics_model;
    SlipPredictor _slip_predictor;
    MPCOptimizer _mpc_optimizer;
    AdaptationSystem _adaptation_system;
    RiskAssessment _risk_assessment;

    // State tracking
    struct PredictorState {
        matrix::Vector<float, 6> vehicle_state;
        matrix::Vector<float, 6> reference_state;
        matrix::Matrix<float, 3, MAX_CONTROL_STEPS> optimal_control_sequence;
        matrix::Matrix<float, 6, MAX_PREDICTION_STEPS> predicted_states;
        matrix::Vector<float, MAX_PREDICTION_STEPS> predicted_slip_front;
        matrix::Vector<float, MAX_PREDICTION_STEPS> predicted_slip_rear;
        matrix::Vector<float, MAX_PREDICTION_STEPS> predicted_risk;
        RiskAssessment::RiskFactors current_risk;
        float prediction_confidence{0.0f};
        uint64_t last_update_time{0};
        bool prediction_valid{false};
    } _predictor_state;

    // Performance metrics
    struct PredictiveMetrics {
        uint32_t successful_predictions{0};
        uint32_t failed_predictions{0};
        uint32_t interventions_triggered{0};
        float avg_prediction_accuracy{0.0f};
        float avg_optimization_time_us{0.0f};
        float avg_risk_level{0.0f};
        uint64_t last_intervention_time{0};
    } _metrics;

    // Internal methods
    bool updateVehicleState(const TractionState& state);
    bool calculateReferenceState(const TractionState& state);
    bool runMPCOptimization();
    void updatePredictions();
    void adaptControlParameters();
    void publishPredictiveControl(TractionCommand& command);

    float validatePrediction() const;
    void updatePerformanceMetrics(bool success, float processing_time_us);

    // Strategy integration
    std::shared_ptr<ITractionStrategy> _emergency_strategy;
    std::shared_ptr<ITractionStrategy> _conservative_strategy;
    std::shared_ptr<ITractionStrategy> _aggressive_strategy;

    void selectAndApplyStrategy(const TractionState& state, TractionCommand& command);
};

} // namespace traction_control
