#pragma once

#include <cstdint>
#include <lib/matrix/matrix/Matrix.hpp>

namespace traction_control {

/**
 * @brief Comprehensive traction state information
 *
 * This structure contains all the information needed by traction control
 * modules to make decisions about torque distribution, slip control, etc.
 */
struct TractionState {
    // Timestamp
    uint64_t timestamp{0};

    // Vehicle dynamics
    struct VehicleDynamics {
        float velocity_x_ms{0.0f};        // Longitudinal velocity [m/s]
        float velocity_y_ms{0.0f};        // Lateral velocity [m/s]
        float acceleration_x_ms2{0.0f};   // Longitudinal acceleration [m/s²]
        float acceleration_y_ms2{0.0f};   // Lateral acceleration [m/s²]
        float yaw_rate_rad_s{0.0f};       // Yaw rate [rad/s]
        float pitch_angle_rad{0.0f};      // Pitch angle [rad]
        float roll_angle_rad{0.0f};       // Roll angle [rad]
        float steering_angle_rad{0.0f};   // Steering angle [rad]
        bool dynamics_valid{false};       // Data validity flag
    } dynamics;

    // Wheel state
    struct WheelState {
        float speed_rad_s[4]{0.0f};       // Wheel speeds [rad/s] [FL, FR, RL, RR]
        float slip_ratio[4]{0.0f};        // Slip ratios [-1, 1] [FL, FR, RL, RR]
        float torque_nm[4]{0.0f};         // Current torques [Nm] [FL, FR, RL, RR]
        float normal_force_n[4]{0.0f};    // Normal forces [N] [FL, FR, RL, RR]
        bool wheel_valid[4]{false};       // Wheel data validity flags
    } wheels;

    // Surface conditions
    struct SurfaceConditions {
        enum SurfaceType {
            ASPHALT = 0,
            GRAVEL = 1,
            MUD = 2,
            SAND = 3,
            SNOW = 4,
            ICE = 5,
            UNKNOWN = 6
        };

        SurfaceType surface_type{UNKNOWN};
        float friction_coefficient{0.6f}; // Surface friction coefficient [0, 1]
        float roughness_level{0.0f};      // Surface roughness [0, 1]
        float moisture_level{0.0f};       // Surface moisture [0, 1]
        float temperature_c{20.0f};       // Surface temperature [°C]
        float slope_angle_rad{0.0f};      // Surface slope angle [rad]
        float confidence{0.0f};           // Classification confidence [0, 1]
        bool conditions_valid{false};     // Data validity flag
    } surface;

    // Load information
    struct LoadInformation {
        float vehicle_mass_kg{12000.0f};  // Total vehicle mass [kg]
        float payload_mass_kg{0.0f};      // Payload mass [kg]
        float cog_x_m{0.0f};              // Center of gravity X [m]
        float cog_y_m{0.0f};              // Center of gravity Y [m]
        float cog_z_m{1.2f};              // Center of gravity Z [m]
        float front_axle_load_kg{6000.0f}; // Front axle load [kg]
        float rear_axle_load_kg{6000.0f};  // Rear axle load [kg]
        bool load_valid{false};           // Data validity flag
    } load;

    // Environmental conditions
    struct EnvironmentalConditions {
        float ambient_temperature_c{20.0f}; // Ambient temperature [°C]
        float humidity_percent{50.0f};       // Humidity [%]
        float wind_speed_ms{0.0f};           // Wind speed [m/s]
        float visibility_m{1000.0f};        // Visibility [m]
        bool weather_valid{false};          // Data validity flag
    } environment;

    // Trajectory information
    struct TrajectoryInformation {
        float target_velocity_x_ms{0.0f}; // Target longitudinal velocity [m/s]
        float target_velocity_y_ms{0.0f}; // Target lateral velocity [m/s]
        float target_yaw_rate_rad_s{0.0f}; // Target yaw rate [rad/s]
        float path_curvature_rad_m{0.0f}; // Path curvature [1/m]
        float lookahead_distance_m{5.0f}; // Lookahead distance [m]
        float predicted_acceleration_x_ms2{0.0f}; // Predicted acceleration [m/s²]
        float predicted_acceleration_y_ms2{0.0f}; // Predicted acceleration [m/s²]
        bool trajectory_valid{false};     // Data validity flag
    } trajectory;

    // System status
    struct SystemStatus {
        bool emergency_stop{false};       // Emergency stop active
        bool manual_override{false};      // Manual override active
        bool system_fault{false};         // System fault detected
        bool calibration_mode{false};     // Calibration mode active
        uint8_t operating_mode{0};        // Operating mode (0=off, 1=eco, 2=normal, 3=sport)
        float battery_voltage_v{24.0f};   // Battery voltage [V]
        float system_temperature_c{40.0f}; // System temperature [°C]
    } system;

    // Data freshness indicators
    struct DataFreshness {
        uint64_t dynamics_timestamp{0};
        uint64_t wheels_timestamp{0};
        uint64_t surface_timestamp{0};
        uint64_t load_timestamp{0};
        uint64_t environment_timestamp{0};
        uint64_t trajectory_timestamp{0};
        uint64_t system_timestamp{0};
    } freshness;

    /**
     * @brief Check if core data is valid and fresh
     * @param max_age_us Maximum age for data to be considered fresh [microseconds]
     * @return true if core data is valid and fresh
     */
    bool isCoreDataValid(uint64_t max_age_us = 100000) const {
        return dynamics.dynamics_valid &&
               system.timestamp > 0 &&
               (timestamp - freshness.dynamics_timestamp) < max_age_us;
    }

    /**
     * @brief Get overall confidence in the state data
     * @return Confidence value [0, 1]
     */
    float getOverallConfidence() const {
        float confidence = 0.0f;
        int valid_components = 0;

        if (dynamics.dynamics_valid) { confidence += 0.3f; valid_components++; }
        if (wheels.wheel_valid[0] || wheels.wheel_valid[1] || wheels.wheel_valid[2] || wheels.wheel_valid[3]) {
            confidence += 0.25f; valid_components++;
        }
        if (surface.conditions_valid) { confidence += 0.2f; valid_components++; }
        if (load.load_valid) { confidence += 0.15f; valid_components++; }
        if (trajectory.trajectory_valid) { confidence += 0.1f; valid_components++; }

        return valid_components > 0 ? confidence : 0.0f;
    }
};

/**
 * @brief Performance metrics for traction control modules
 */
struct TractionModuleMetrics {
    uint64_t timestamp{0};
    uint32_t process_count{0};            // Number of times process() was called
    uint32_t success_count{0};            // Number of successful processes
    uint32_t error_count{0};              // Number of errors
    float avg_processing_time_us{0.0f};   // Average processing time [microseconds]
    float max_processing_time_us{0.0f};   // Maximum processing time [microseconds]
    float cpu_usage_percent{0.0f};        // CPU usage percentage
    float memory_usage_bytes{0.0f};       // Memory usage [bytes]
    float effectiveness_score{0.0f};      // Module effectiveness [0, 1]
    uint64_t last_reset_time{0};          // Last reset timestamp
};

} // namespace traction_control
