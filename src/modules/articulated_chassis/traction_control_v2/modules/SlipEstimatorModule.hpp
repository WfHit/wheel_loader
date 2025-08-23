#pragma once

#include "../core/TractionControlModuleBase.hpp"
#include <lib/matrix/matrix/Matrix.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

namespace traction_control {

/**
 * @brief Refactored slip estimator module
 *
 * Estimates wheel slip using an Extended Kalman Filter and provides
 * slip information to other traction control modules.
 */
class SlipEstimatorModule : public ParameterizedTractionModule<SlipEstimatorModule::Parameters> {
public:
    struct Parameters {
        float wheel_radius_m{0.4f};          // Wheel radius [m]
        float wheelbase_m{2.5f};             // Vehicle wheelbase [m]
        float track_width_m{1.8f};           // Vehicle track width [m]
        float process_noise{0.01f};          // EKF process noise
        float measurement_noise{0.1f};       // EKF measurement noise
        float slip_threshold{0.05f};         // Slip detection threshold
        float friction_init{0.6f};           // Initial friction estimate
        int estimation_method{1};            // Estimation method (0=basic, 1=EKF)
        int front_encoder_idx{0};            // Front encoder index
        int rear_encoder_idx{1};             // Rear encoder index
        int update_hz{50};                   // Update frequency [Hz]
        bool enable_learning{true};          // Enable adaptive learning
        float confidence_threshold{0.7f};    // Minimum confidence for output
    };

    SlipEstimatorModule();
    ~SlipEstimatorModule() override = default;

    const char* getName() const override { return "SlipEstimator"; }
    int getPriority() const override { return 8; } // High priority

protected:
    bool doInitialize() override;
    bool doProcess(const TractionState& state, TractionCommand& command) override;
    void doReset() override;
    void loadParameters() override;

private:
    /**
     * @brief Extended Kalman Filter for slip estimation
     */
    class SlipEKF {
    public:
        SlipEKF();

        void initialize(float wheel_radius, float wheelbase, float process_noise, float measurement_noise);
        void predict(float dt);
        void updateWheelSpeeds(const float wheel_speeds[4]);
        void updateVehicleDynamics(float vx, float vy, float yaw_rate);

        // Getters
        float getSlipFront() const { return _state(2); }
        float getSlipRear() const { return _state(3); }
        float getFrictionEstimate() const { return _state(4); }
        float getConfidence() const { return _confidence; }

        // State vector: [vx, vy, slip_front, slip_rear, friction]
        const matrix::Vector<float, 5>& getState() const { return _state; }

    private:
        matrix::Vector<float, 5> _state;              // State vector
        matrix::SquareMatrix<float, 5> _P;            // Covariance matrix
        matrix::SquareMatrix<float, 5> _Q;            // Process noise
        matrix::SquareMatrix<float, 4> _R_wheels;     // Wheel measurement noise
        matrix::SquareMatrix<float, 3> _R_dynamics;   // Dynamics measurement noise

        float _wheel_radius;
        float _wheelbase;
        float _confidence{0.0f};
        bool _initialized{false};

        void updateCovariance(float dt);
        matrix::Matrix<float, 4, 5> getWheelMeasurementJacobian() const;
        matrix::Matrix<float, 3, 5> getDynamicsMeasurementJacobian() const;
    };

    /**
     * @brief Friction coefficient estimator using Recursive Least Squares
     */
    class FrictionEstimator {
    public:
        FrictionEstimator();

        void initialize(float initial_friction);
        void update(float slip, float normalized_force);

        float getFriction() const { return _friction; }
        float getConfidence() const { return _confidence; }

    private:
        float _friction{0.6f};
        float _confidence{0.0f};
        matrix::Vector<float, 2> _theta;          // RLS parameters
        matrix::SquareMatrix<float, 2> _P_rls;    // RLS covariance
        float _forgetting_factor{0.98f};
        uint32_t _sample_count{0};
    };

    /**
     * @brief Surface change detector
     */
    class SurfaceChangeDetector {
    public:
        void update(float friction, float slip_variance);
        bool hasChanged() const { return _surface_changed; }
        float getChangeConfidence() const { return _change_confidence; }
        void reset();

    private:
        static constexpr int HISTORY_SIZE = 20;
        float _friction_history[HISTORY_SIZE];
        float _slip_variance_history[HISTORY_SIZE];
        int _history_index{0};
        bool _surface_changed{false};
        float _change_confidence{0.0f};

        float calculateVariance(const float* data, int size) const;
    };

    // Core estimation components
    SlipEKF _slip_ekf;
    FrictionEstimator _friction_estimator;
    SurfaceChangeDetector _surface_detector;

    // uORB subscriptions
    uORB::SubscriptionMultiArray<sensor_quad_encoder_s> _encoder_sub{ORB_ID::sensor_quad_encoder};
    uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};

    // State tracking
    struct EstimatorState {
        float wheel_speeds[4]{0.0f};
        float slip_ratios[4]{0.0f};
        float vehicle_velocity_x{0.0f};
        float vehicle_velocity_y{0.0f};
        float yaw_rate{0.0f};
        float friction_estimate{0.6f};
        float estimation_confidence{0.0f};
        uint64_t last_update_time{0};
        bool data_valid{false};
    } _estimator_state;

    // Internal methods
    bool updateSensorData();
    bool updateWheelSpeeds();
    bool updateVehicleDynamics();
    void runSlipEstimation();
    void updateFrictionEstimate();
    void detectSurfaceChanges();
    void publishSlipEstimation(TractionCommand& command);
    float calculateSlipRatio(float wheel_speed, float vehicle_speed, float wheel_radius) const;
    float calculateLateralSlip(float lateral_velocity, float longitudinal_velocity) const;
    bool validateEstimation() const;

    // Performance tracking
    struct PerformanceMetrics {
        uint32_t successful_estimations{0};
        uint32_t failed_estimations{0};
        float avg_confidence{0.0f};
        float estimation_accuracy{0.0f};
        uint64_t last_surface_change{0};
    } _performance;
};

} // namespace traction_control
