#pragma once

#include <cstdint>
#include <lib/matrix/matrix/Matrix.hpp>

namespace traction_control {

/**
 * @brief Traction control command output
 *
 * This structure contains the output commands from traction control modules
 * that will be sent to actuators and other systems.
 */
struct TractionCommand {
    // Timestamp
    uint64_t timestamp{0};

    // Torque commands
    struct TorqueCommands {
        float wheel_torque_nm[4]{0.0f};      // Individual wheel torques [Nm] [FL, FR, RL, RR]
        float torque_distribution{0.5f};     // Front/rear distribution [-1=rear, 1=front]
        float total_torque_limit_nm{1000.0f}; // Total torque limit [Nm]
        float torque_rate_limit_nm_s{100.0f}; // Torque rate limit [Nm/s]
        bool torque_valid{true};             // Torque command validity
    } torque;

    // Brake commands
    struct BrakeCommands {
        float wheel_brake_pressure[4]{0.0f}; // Brake pressure [bar] [FL, FR, RL, RR]
        float brake_distribution{0.5f};      // Front/rear brake distribution
        bool abs_enable{false};              // ABS enable flag
        bool brake_valid{false};             // Brake command validity
    } brake;

    // Steering commands
    struct SteeringCommands {
        float steering_correction_rad{0.0f}; // Steering correction [rad]
        float max_steering_rate_rad_s{1.0f}; // Maximum steering rate [rad/s]
        bool steering_valid{false};          // Steering command validity
    } steering;

    // Differential commands
    struct DifferentialCommands {
        bool front_diff_lock{false};         // Front differential lock
        bool rear_diff_lock{false};          // Rear differential lock
        bool center_diff_lock{false};        // Center differential lock
        float diff_bias{0.0f};               // Differential bias [-1, 1]
        bool differential_valid{false};      // Differential command validity
    } differential;

    // Traction control parameters
    struct TractionParameters {
        float target_slip_ratio{0.1f};       // Target slip ratio [0, 1]
        float slip_tolerance{0.05f};         // Slip tolerance [0, 1]
        uint8_t intervention_level{0};       // Intervention level (0=none, 1=mild, 2=moderate, 3=aggressive)
        uint8_t traction_mode{2};            // Traction mode (0=off, 1=eco, 2=normal, 3=sport)
        float response_time_s{0.1f};         // Response time [s]
        bool parameters_valid{true};         // Parameters validity
    } parameters;

    // Control flags
    struct ControlFlags {
        bool traction_control_enable{true};  // Enable traction control
        bool stability_control_enable{true}; // Enable stability control
        bool slip_control_enable{true};      // Enable slip control
        bool load_adaptation_enable{true};   // Enable load adaptation
        bool terrain_adaptation_enable{true}; // Enable terrain adaptation
        bool predictive_control_enable{true}; // Enable predictive control
        bool emergency_override{false};      // Emergency override active
        bool manual_mode{false};             // Manual control mode
    } control;

    // Performance optimization
    struct PerformanceSettings {
        float efficiency_weight{0.3f};       // Efficiency optimization weight [0, 1]
        float traction_weight{0.4f};         // Traction optimization weight [0, 1]
        float comfort_weight{0.2f};          // Comfort optimization weight [0, 1]
        float safety_weight{0.1f};           // Safety optimization weight [0, 1]
        float aggressiveness{0.5f};          // Control aggressiveness [0, 1]
        bool performance_valid{true};        // Performance settings validity
    } performance;

    // Prediction and planning
    struct PredictionCommands {
        matrix::Vector<float, 10> predicted_torque_sequence; // Predicted torque sequence
        matrix::Vector<float, 10> predicted_slip_sequence;   // Predicted slip sequence
        float prediction_horizon_s{1.0f};    // Prediction horizon [s]
        float confidence{0.5f};               // Prediction confidence [0, 1]
        bool prediction_valid{false};         // Prediction validity
    } prediction;

    // Status information
    struct StatusInformation {
        bool intervention_active{false};     // Intervention currently active
        bool slip_detected{false};           // Slip currently detected
        bool stability_threat{false};        // Stability threat detected
        float overall_effectiveness{0.0f};   // Overall effectiveness [0, 1]
        uint32_t intervention_count{0};      // Total intervention count
        float last_intervention_time_s{0.0f}; // Time since last intervention [s]
        uint8_t system_health{100};          // System health [0-100%]
    } status;

    // Command priorities and sources
    struct CommandSources {
        enum SourceType {
            SLIP_ESTIMATOR = 0,
            PREDICTIVE_CONTROL = 1,
            TERRAIN_ADAPTATION = 2,
            LOAD_AWARE_DISTRIBUTION = 3,
            STABILITY_CONTROL = 4,
            EMERGENCY_SYSTEM = 5,
            MANUAL_OVERRIDE = 6
        };

        SourceType primary_source{SLIP_ESTIMATOR};
        SourceType secondary_source{PREDICTIVE_CONTROL};
        float source_confidence[7]{0.5f};    // Confidence in each source [0, 1]
        uint8_t priority_levels[7]{3, 5, 4, 6, 7, 9, 8}; // Priority levels (higher = more important)
    } sources;

    /**
     * @brief Validate all command components
     * @return true if all enabled commands are valid
     */
    bool isValid() const {
        bool valid = true;

        if (control.traction_control_enable && !torque.torque_valid) valid = false;
        if (control.stability_control_enable && !brake.brake_valid && !steering.steering_valid) valid = false;
        if (!parameters.parameters_valid) valid = false;
        if (!performance.performance_valid) valid = false;

        return valid;
    }

    /**
     * @brief Get overall command urgency
     * @return Urgency level [0, 1] where 1 is most urgent
     */
    float getUrgency() const {
        float urgency = 0.0f;

        if (status.stability_threat) urgency += 0.4f;
        if (status.slip_detected) urgency += 0.3f;
        if (status.intervention_active) urgency += 0.2f;
        if (control.emergency_override) urgency += 0.1f;

        return std::min(urgency, 1.0f);
    }

    /**
     * @brief Reset all commands to safe defaults
     */
    void reset() {
        torque = {};
        brake = {};
        steering = {};
        differential = {};
        parameters = {};
        control.emergency_override = false;
        control.manual_mode = false;
        status = {};
    }

    /**
     * @brief Apply safety limits to all commands
     */
    void applySafetyLimits() {
        // Limit torque values
        for (int i = 0; i < 4; i++) {
            torque.wheel_torque_nm[i] = std::max(-2000.0f, std::min(2000.0f, torque.wheel_torque_nm[i]));
            brake.wheel_brake_pressure[i] = std::max(0.0f, std::min(200.0f, brake.wheel_brake_pressure[i]));
        }

        // Limit distribution values
        torque.torque_distribution = std::max(-1.0f, std::min(1.0f, torque.torque_distribution));
        brake.brake_distribution = std::max(-1.0f, std::min(1.0f, brake.brake_distribution));

        // Limit steering
        steering.steering_correction_rad = std::max(-0.5f, std::min(0.5f, steering.steering_correction_rad));
        steering.max_steering_rate_rad_s = std::max(0.1f, std::min(2.0f, steering.max_steering_rate_rad_s));

        // Limit parameters
        parameters.target_slip_ratio = std::max(0.0f, std::min(0.3f, parameters.target_slip_ratio));
        parameters.slip_tolerance = std::max(0.01f, std::min(0.2f, parameters.slip_tolerance));
        parameters.intervention_level = std::min(static_cast<uint8_t>(3), parameters.intervention_level);
        parameters.traction_mode = std::min(static_cast<uint8_t>(3), parameters.traction_mode);
    }
};

} // namespace traction_control
