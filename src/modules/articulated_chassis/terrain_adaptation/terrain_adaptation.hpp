#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/matrix/matrix/Matrix.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

// uORB message includes
#include <uORB/topics/terrain_adaptation.h>
#include <uORB/topics/slip_estimation.h>
#include <uORB/topics/module_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>

class TerrainAdaptation : public ModuleBase<TerrainAdaptation>, public ModuleParams
{
public:
    TerrainAdaptation();
    ~TerrainAdaptation() = default;

    static int task_spawn(int argc, char *argv[]);
    void run() override;

private:
    // Parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::TA_ROUGHNESS_THRESHOLD>) _param_roughness_threshold,
        (ParamFloat<px4::params::TA_SLOPE_THRESHOLD>) _param_slope_threshold,
        (ParamFloat<px4::params::TA_FRICTION_ALPHA>) _param_friction_alpha,
        (ParamFloat<px4::params::TA_SPEED_REDUCTION>) _param_speed_reduction,
        (ParamFloat<px4::params::TA_STABILITY_MARGIN>) _param_stability_margin,
        (ParamInt<px4::params::TA_CLASSIFIER_METHOD>) _param_classifier_method,
        (ParamBool<px4::params::TA_ENABLE>) _param_enable
    )

    // Subscriptions
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _slip_estimation_sub{ORB_ID(slip_estimation)};
    uORB::Subscription _load_sensing_sub{ORB_ID(load_sensing)};
    uORB::Subscription _wheel_speeds_setpoint_sub{ORB_ID(wheel_speeds_setpoint)};

    // Publications
    uORB::Publication<terrain_adaptation_s> _terrain_adaptation_pub{ORB_ID(terrain_adaptation)};
    uORB::Publication<traction_control_s> _traction_control_pub{ORB_ID(traction_control)};
    uORB::Publication<wheel_speeds_setpoint_s> _wheel_speeds_setpoint_pub{ORB_ID(wheel_speeds_setpoint)};

    // Surface classification system
    class SurfaceClassifier {
    public:
        enum SurfaceType {
            ASPHALT = 0,
            GRAVEL = 1,
            MUD = 2,
            SAND = 3,
            SNOW = 4,
            ICE = 5,
            UNKNOWN = 6
        };

        struct SurfaceFeatures {
            float vibration_amplitude;     // Accelerometer variance
            float vibration_frequency;     // Dominant frequency
            float slip_characteristics;    // Slip pattern
            float friction_coefficient;    // Estimated friction
            float temperature;             // Ambient temperature
            float moisture_indicator;      // Derived from other sensors
        };

        SurfaceClassifier();
        SurfaceType classify(const SurfaceFeatures &features);
        float getSurfaceRoughness(SurfaceType type);
        float getExpectedFriction(SurfaceType type);
        void updateClassificationConfidence(SurfaceType type, float performance);

    private:
        struct SurfaceModel {
            float friction_range[2];       // [min, max] friction
            float roughness_level;         // 0-1 scale
            float vibration_signature[3];  // [amplitude, frequency, variance]
            float confidence;              // Classification confidence
        };

        SurfaceModel _surface_models[7]; // One for each surface type

        // Machine learning classifier (simplified)
        float _classification_weights[6][7]; // 6 features x 7 surface types

        void initializeSurfaceModels();
        float calculateFeatureDistance(const SurfaceFeatures &features, SurfaceType type);
    };

    // Slope estimation and analysis
    class SlopeAnalyzer {
    public:
        struct SlopeInfo {
            float grade_angle_rad;         // Slope angle
            float cross_slope_rad;         // Cross slope (roll)
            float slope_direction_rad;     // Direction relative to vehicle heading
            float slope_change_rate;       // Rate of slope change
            bool uphill;                   // Climbing up or down
        };

        SlopeAnalyzer();
        void update(float pitch, float roll, float heading, float altitude);
        SlopeInfo getCurrentSlope() const { return _current_slope; }
        float predictStabilityRisk(float vehicle_speed, float cog_height);

    private:
        SlopeInfo _current_slope;

        // Slope history for trend analysis
        static constexpr int SLOPE_HISTORY_SIZE = 20;
        float _pitch_history[SLOPE_HISTORY_SIZE];
        float _roll_history[SLOPE_HISTORY_SIZE];
        float _altitude_history[SLOPE_HISTORY_SIZE];
        int _history_index{0};

        float calculateSlopeChangeRate();
        void addToHistory(float pitch, float roll, float altitude);
    };

    // Terrain-adaptive control strategies
    class AdaptiveController {
    public:
        struct ControlStrategy {
            float max_speed_ms;            // Recommended maximum speed
            float torque_distribution;     // Optimal torque split
            float traction_control_gain;   // Aggressiveness of TC
            float stability_threshold;     // When to intervene
            bool differential_lock;        // Enable/disable differential
        };

        AdaptiveController();
        ControlStrategy computeStrategy(SurfaceClassifier::SurfaceType surface,
                                      const SlopeAnalyzer::SlopeInfo &slope,
                                      float load_factor,
                                      float current_speed);
        void updateFromPerformance(float slip_performance, float stability_performance);

    private:
        // Pre-defined strategies for different terrain types
        ControlStrategy _terrain_strategies[7];

        // Adaptive parameters that learn from experience
        struct AdaptiveParams {
            float speed_reduction_factor{1.0f};
            float tc_sensitivity{1.0f};
            float stability_margin{0.1f};
        } _adaptive_params;

        void initializeStrategies();
        ControlStrategy blendStrategies(const ControlStrategy &base,
                                       const SlopeAnalyzer::SlopeInfo &slope,
                                       float load_factor);
    };

    // Environmental sensor fusion
    class EnvironmentalSensing {
    public:
        struct EnvironmentalData {
            float ambient_temperature;     // From external sensor or estimated
            float humidity_estimate;       // Derived from other sensors
            float precipitation_indicator; // Based on slip patterns
            float visibility_factor;       // For future camera integration
        };

        EnvironmentalSensing();
        void update(float friction_estimate, float slip_variance,
                   float vibration_level);
        EnvironmentalData getData() const { return _env_data; }

    private:
        EnvironmentalData _env_data;

        // Simple environmental estimation
        void estimateWeatherConditions(float friction, float slip_variance);
    };

    // Performance monitoring and learning
    class PerformanceMonitor {
    public:
        struct PerformanceMetrics {
            float traction_efficiency;     // How well traction is utilized
            float stability_score;         // Vehicle stability rating
            float energy_efficiency;       // Power consumption efficiency
            float operator_comfort;        // Smoothness of operation
            float task_completion_rate;    // Success in completing maneuvers
        };

        PerformanceMonitor();
        void recordPerformance(const PerformanceMetrics &metrics,
                             SurfaceClassifier::SurfaceType surface,
                             const AdaptiveController::ControlStrategy &strategy);
        void evaluateAdaptationEffectiveness();
        float getAdaptationConfidence() const { return _adaptation_confidence; }

    private:
        static constexpr int PERFORMANCE_HISTORY_SIZE = 100;

        struct PerformanceRecord {
            PerformanceMetrics metrics;
            SurfaceClassifier::SurfaceType surface;
            AdaptiveController::ControlStrategy strategy;
            uint64_t timestamp;
        };

        PerformanceRecord _performance_history[PERFORMANCE_HISTORY_SIZE];
        int _history_count{0};
        float _adaptation_confidence{0.5f};

        float calculateOverallScore(const PerformanceMetrics &metrics);
    };

    // Member variables
    SurfaceClassifier _surface_classifier;
    SlopeAnalyzer _slope_analyzer;
    AdaptiveController _adaptive_controller;
    EnvironmentalSensing _environmental_sensing;
    PerformanceMonitor _performance_monitor;

    // State tracking
    struct TerrainState {
        SurfaceClassifier::SurfaceType current_surface{SurfaceClassifier::UNKNOWN};
        SlopeAnalyzer::SlopeInfo current_slope;
        float current_roughness{0.0f};
        float adaptation_confidence{0.5f};
        bool adaptation_active{false};
        uint64_t last_classification_time{0};
    } _terrain_state;

    // Vibration analysis for surface detection
    struct VibrationAnalyzer {
        static constexpr int FFT_SIZE = 64;
        float _accel_buffer[FFT_SIZE];
        int _buffer_index{0};

        void addSample(float accel_magnitude);
        float computeVibrationAmplitude();
        float computeDominantFrequency();
    } _vibration_analyzer;

    // Methods
    void updateTerrainClassification();
    void updateSlopeAnalysis();
    void computeAdaptiveStrategy(terrain_adaptation_s &output);
    void applyTerrainAdaptation(const terrain_adaptation_s &adaptation);
    void monitorPerformance();
    void publishAdaptationCommands(const AdaptiveController::ControlStrategy &strategy);
};
