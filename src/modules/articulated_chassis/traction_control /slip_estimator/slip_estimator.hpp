#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/Publication.hpp>
#include <lib/matrix/matrix/Matrix.hpp>

// uORB message includes for EKF results (instead of linking to EKF library)
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/slip_estimation.h>
#include <uORB/topics/module_status.h>
#include <uORB/topics/steering_status.h>

class SlipEstimator : public ModuleBase<SlipEstimator>, public ModuleParams
{
public:
    SlipEstimator();
    ~SlipEstimator() = default;

    static int task_spawn(int argc, char *argv[]);
    static SlipEstimator *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void run() override;
    bool init();

private:
    // Parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::SE_WHEEL_R>) _param_wheel_radius,
        (ParamFloat<px4::params::SE_VEHICLE_MASS>) _param_vehicle_mass,
        (ParamFloat<px4::params::SE_COG_HEIGHT>) _param_cog_height,
        (ParamFloat<px4::params::SE_WHEELBASE>) _param_wheelbase,
        (ParamFloat<px4::params::SE_PROC_NOISE>) _param_process_noise,
        (ParamFloat<px4::params::SE_MEAS_NOISE>) _param_measurement_noise,
        (ParamInt<px4::params::SE_EST_METHOD>) _param_estimation_method,
        (ParamInt<px4::params::SE_FRONT_IDX>) _param_front_encoder_idx,
        (ParamInt<px4::params::SE_REAR_IDX>) _param_rear_encoder_idx,
        (ParamFloat<px4::params::SE_SLIP_THRESH>) _param_slip_threshold,
        (ParamFloat<px4::params::SE_FRIC_INIT>) _param_friction_init,
        (ParamInt<px4::params::SE_UPDATE_HZ>) _param_update_hz
    )

    // Subscriptions
    uORB::SubscriptionMultiArray<sensor_quad_encoder_s> _sensor_quad_encoder_sub{ORB_ID::sensor_quad_encoder};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _estimator_states_sub{ORB_ID(estimator_states)};
    uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
    uORB::Subscription _steering_status_sub{ORB_ID(steering_status)};

    // Publications
    uORB::Publication<slip_estimation_s> _slip_estimation_pub{ORB_ID(slip_estimation)};

    // Slip estimation using EKF results from messages (not EKF library)
    class SlipEKF {
    public:
        SlipEKF();
        void init(float wheel_radius, float wheelbase, float process_noise, float measurement_noise, float friction_init);
        void updateFromEKFMessages(const vehicle_attitude_s &attitude,
                                   const vehicle_local_position_s &position,
                                   const estimator_states_s &states);
        void updateWheelSpeed(float front_speed, float rear_speed);
        void updateVehicleState(float ax, float ay, float yaw_rate);

        matrix::Vector<float, 5> getState() const { return _x; }
        float getSlipFront() const;
        float getSlipRear() const;
        float getLateralSlip() const;
        float getFrictionEstimate() const;

    private:
        // State vector: [vx, vy, slip_front, slip_rear, friction_coeff]
        matrix::Vector<float, 5> _x;
        matrix::SquareMatrix<float, 5> _P; // Covariance
        matrix::SquareMatrix<float, 5> _Q; // Process noise
        matrix::SquareMatrix<float, 3> _R_wheel; // Wheel measurement noise
        matrix::SquareMatrix<float, 3> _R_vehicle; // Vehicle measurement noise

        float _wheel_radius;
        float _wheelbase;

        // EKF state from messages
        struct EKFState {
            float vx{0.0f};
            float vy{0.0f};
            float vz{0.0f};
            float ax{0.0f};
            float ay{0.0f};
            float yaw_rate{0.0f};
            matrix::Quatf q;
            bool valid{false};
        } _ekf_state;

    public:
        void predict(float dt);
        void updateCovariance(const matrix::SquareMatrix<float, 5> &F, float dt);
    };

    // Recursive Least Squares friction estimator
    class FrictionEstimator {
    public:
        FrictionEstimator();
        void init(float friction_init);
        void update(float slip, float normalized_force, float slip_threshold = 0.01f);
        float getFrictionCoeff() const { return _mu; }
        float getConfidence() const { return _confidence; }

    private:
        float _mu{0.8f};           // Friction coefficient
        float _confidence{0.0f};   // Estimation confidence
        matrix::Matrix<float, 2, 1> _theta;   // RLS parameters
        matrix::SquareMatrix<float, 2> _P; // RLS covariance
        float _lambda{0.98f};      // Forgetting factor
    };

    // Member variables
    SlipEKF _ekf;
    FrictionEstimator _friction_estimator;

    // State variables
    struct VehicleState {
        float speed_x{0.0f};
        float speed_y{0.0f};
        float accel_x{0.0f};
        float accel_y{0.0f};
        float yaw_rate{0.0f};
        float wheel_speed_front{0.0f};
        float wheel_speed_rear{0.0f};
        float steering_angle{0.0f};
        uint64_t last_update{0};
    } _vehicle_state;

    // Methods
    void updateVehicleState();
    void updateFromEKFMessages();
    void runBasicEstimation(slip_estimation_s &slip);
    void runEKFEstimation(slip_estimation_s &slip);
    float calculateTireModel(float slip, float normal_force);
    void detectSurfaceChange(slip_estimation_s &slip);
};
