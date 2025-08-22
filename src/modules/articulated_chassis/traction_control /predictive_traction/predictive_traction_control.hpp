#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/mathlib/mathlib.h>

// uORB message includes
#include <uORB/topics/wheel_speeds_setpoint.h>
#include <uORB/topics/slip_estimation.h>
#include <uORB/topics/traction_control.h>
#include <uORB/topics/predictive_traction.h>
#include <uORB/topics/module_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/terrain_adaptation.h>
#include <uORB/topics/load_sensing.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;
using namespace matrix;

/**
 * @brief Predictive Traction Control for articulated wheel loader
 *
 * Implements Model Predictive Control (MPC) for advanced traction management:
 * - Multi-step ahead slip prediction
 * - Optimal torque distribution planning
 * - Learning-based terrain adaptation
 * - Risk assessment and intervention timing
 */
class PredictiveTractionControl : public ModuleBase<PredictiveTractionControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    PredictiveTractionControl();
    ~PredictiveTractionControl() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static PredictiveTractionControl *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    bool init();

    int print_status() override;

    /**
     * Reset learned parameters and model adaptations
     */
    void reset_learned_parameters();

private:
    static constexpr float CONTROL_RATE_HZ = 20.0f;
    static constexpr uint64_t CONTROL_INTERVAL_US = 1_s / CONTROL_RATE_HZ;

    // MPC Configuration
    static constexpr int PREDICTION_HORIZON = 10;
    static constexpr int CONTROL_HORIZON = 5;
    static constexpr float PREDICTION_TIME_STEP = 0.1f; // 100ms per step
    static constexpr float MAX_TORQUE_CHANGE_RATE = 50.0f; // Nm/s
    static constexpr float MAX_STEERING_CHANGE_RATE = 0.5f; // rad/s

    // Risk thresholds
    static constexpr float SLIP_WARNING_THRESHOLD = 0.15f;
    static constexpr float SLIP_CRITICAL_THRESHOLD = 0.25f;
    static constexpr float STABILITY_WARNING_THRESHOLD = 0.3f;

    void Run() override;

    /**
     * MPC functions
     */
    void update_system_model();
    void predict_future_states();
    void optimize_control_sequence();
    void apply_mpc_solution();

    /**
     * Vehicle dynamics model
     */
    void update_vehicle_state();
    Vector<float, 6> vehicle_dynamics_model(const Vector<float, 6>& state,
                                           const Vector<float, 3>& control);

    /**
     * Slip prediction and analysis
     */
    void predict_slip_evolution();
    float calculate_slip_risk(float slip_front, float slip_rear);
    void update_learning_model();

    /**
     * Optimization and control
     */
    Matrix<float, 3, CONTROL_HORIZON> solve_mpc_problem();
    void evaluate_cost_function(const Matrix<float, 3, CONTROL_HORIZON>& control_sequence,
                               float& cost);
    bool check_constraints(const Matrix<float, 3, CONTROL_HORIZON>& control_sequence);

    /**
     * Terrain and adaptation
     */
    void update_terrain_model();
    void adapt_control_parameters();

    /**
     * Utility functions
     */
    void reset_mpc_state();
    void publish_prediction_results();

    /**
     * Control interface
     */
    void set_learning_enabled(bool enabled);

    // uORB subscriptions
    uORB::Subscription _slip_estimation_sub{ORB_ID(slip_estimation)};
    uORB::Subscription _wheel_speeds_sub{ORB_ID(wheel_speeds_setpoint)};
    uORB::Subscription _steering_status_sub{ORB_ID(steering_status)};
    uORB::Subscription _terrain_adaptation_sub{ORB_ID(terrain_adaptation)};
    uORB::Subscription _load_sensing_sub{ORB_ID(load_sensing)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

    // uORB publications
    uORB::Publication<predictive_traction_s> _predictive_traction_pub{ORB_ID(predictive_traction)};
    uORB::Publication<traction_control_s> _traction_control_pub{ORB_ID(traction_control)};
    uORB::Publication<module_status_s> _module_status_pub{ORB_ID(module_status)};

    // Vehicle state vector: [vx, vy, yaw_rate, wheel_speed_front, wheel_speed_rear, steering_angle]
    Vector<float, 6> _vehicle_state;
    Vector<float, 6> _vehicle_state_dot;

    // Control vector: [torque_distribution, steering_correction, brake_intervention]
    Vector<float, 3> _current_control;
    Matrix<float, 3, CONTROL_HORIZON> _optimal_control_sequence;

    // Prediction arrays
    Matrix<float, 6, PREDICTION_HORIZON> _predicted_states;
    Vector<float, PREDICTION_HORIZON> _predicted_slip_front;
    Vector<float, PREDICTION_HORIZON> _predicted_slip_rear;
    Vector<float, PREDICTION_HORIZON> _predicted_risk;

    // Vehicle parameters
    struct VehicleParams {
        float mass_kg{12000.0f};          // Vehicle mass
        float wheelbase_m{2.5f};          // Wheelbase
        float track_width_m{1.8f};        // Track width
        float tire_radius_m{0.5f};        // Tire radius
        float moment_inertia_kg_m2{8000.0f}; // Yaw moment of inertia
        float cornering_stiffness_n_rad{50000.0f}; // Tire cornering stiffness
        float max_friction_coeff{0.8f};   // Maximum friction coefficient
    } _vehicle_params;

    // Current sensor data
    struct SensorData {
        float slip_front{0.0f};
        float slip_rear{0.0f};
        float lateral_slip_rad{0.0f};
        float friction_estimate{0.6f};
        float vehicle_speed_ms{0.0f};
        float yaw_rate_rad_s{0.0f};
        float steering_angle_rad{0.0f};
        float terrain_friction{0.6f};
        float slope_angle_rad{0.0f};
        float payload_mass_kg{0.0f};
        uint64_t last_update_time{0};
        bool data_valid{false};
    } _sensor_data;

    // MPC weights and parameters
    struct MPCParams {
        float weight_slip_tracking{10.0f};
        float weight_stability{20.0f};
        float weight_control_effort{1.0f};
        float weight_control_rate{5.0f};
        float weight_risk_penalty{50.0f};
        float slack_variable_weight{100.0f};
        int max_iterations{50};
        float convergence_tolerance{1e-4f};
    } _mpc_params;

    // Learning and adaptation
    struct LearningData {
        float terrain_adaptation_factor{1.0f};
        float learned_friction_model[10]{0.6f}; // Different terrain types
        float prediction_error_history[20]{0.0f};
        float adaptation_rate{0.1f};
        uint32_t learning_samples{0};
        bool learning_active{true};
    } _learning;

    // Module status and performance
    struct PerformanceMetrics {
        float prediction_accuracy{0.0f};
        float intervention_success_rate{0.0f};
        float avg_computation_time_ms{0.0f};
        uint32_t interventions_count{0};
        uint32_t false_positives{0};
        uint32_t missed_events{0};
        float risk_level_avg{0.0f};
    } _performance;

    // Control state
    bool _prediction_active{false};
    bool _intervention_active{false};
    float _current_risk_level{0.0f};
    uint64_t _last_intervention_time{0};
    uint64_t _computation_start_time{0};

    // Module status
    module_status_s _status{};
    predictive_traction_s _prediction_output{};

    // Parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::PTC_SLIP_WARN>) _slip_warning_threshold,
        (ParamFloat<px4::params::PTC_SLIP_CRIT>) _slip_critical_threshold,
        (ParamFloat<px4::params::PTC_PRED_HOR>) _prediction_horizon_s,
        (ParamFloat<px4::params::PTC_WEIGHT_SLIP>) _weight_slip,
        (ParamFloat<px4::params::PTC_WEIGHT_STAB>) _weight_stability,
        (ParamFloat<px4::params::PTC_WEIGHT_CTRL>) _weight_control,
        (ParamFloat<px4::params::PTC_MAX_TQ_RATE>) _max_torque_rate,
        (ParamFloat<px4::params::PTC_MAX_ST_RATE>) _max_steering_rate,
        (ParamBool<px4::params::PTC_LEARNING_EN>) _learning_enable,
        (ParamFloat<px4::params::PTC_ADAPT_RATE>) _adaptation_rate,
        (ParamInt<px4::params::PTC_MAX_ITER>) _max_iterations,
        (ParamFloat<px4::params::PTC_CONV_TOL>) _convergence_tolerance
    )
};
