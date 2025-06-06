#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/matrix/matrix/Matrix.hpp>
#include <lib/ecl/EKF/ekf.h>

// uORB message includes
#include <uORB/topics/WheelEncoders.h>
#include <uORB/topics/SlipEstimation.h>
#include <uORB/topics/ModuleStatus.h>
#include <uORB/topics/vehicle_status.h>

class SlipEstimator : public ModuleBase<SlipEstimator>, public ModuleParams
{
public:
    SlipEstimator();
    ~SlipEstimator() = default;

    static int task_spawn(int argc, char *argv[]);
    void run() override;

private:
    // Parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::SE_WHEEL_RADIUS>) _param_wheel_radius,
        (ParamFloat<px4::params::SE_VEHICLE_MASS>) _param_vehicle_mass,
        (ParamFloat<px4::params::SE_COG_HEIGHT>) _param_cog_height,
        (ParamFloat<px4::params::SE_WHEELBASE>) _param_wheelbase,
        (ParamFloat<px4::params::SE_PROCESS_NOISE>) _param_process_noise,
        (ParamFloat<px4::params::SE_MEASUREMENT_NOISE>) _param_measurement_noise,
        (ParamInt<px4::params::SE_ESTIMATION_METHOD>) _param_estimation_method
    )

    // Subscriptions
    uORB::Subscription _wheel_odometry_front_sub{ORB_ID(wheel_odometry)};
    uORB::Subscription _wheel_odometry_rear_sub{ORB_ID(wheel_odometry)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _steering_status_sub{ORB_ID(steering_status)};

    // Publications
    uORB::Publication<slip_estimation_s> _slip_estimation_pub{ORB_ID(slip_estimation)};

    // Extended Kalman Filter for slip estimation
    class SlipEKF {
    public:
        SlipEKF();
        void init(float wheel_radius, float wheelbase);
        void predict(float dt);
        void updateWheelSpeed(float front_speed, float rear_speed);
        void updateIMU(float ax, float ay, float yaw_rate);
        void updateGPS(float vx, float vy);

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
        matrix::SquareMatrix<float, 3> _R_imu;   // IMU measurement noise

        float _wheel_radius;
        float _wheelbase;

        void updateCovariance(const matrix::SquareMatrix<float, 5> &F, float dt);
    };

    // Recursive Least Squares friction estimator
    class FrictionEstimator {
    public:
        FrictionEstimator();
        void update(float slip, float normalized_force);
        float getFrictionCoeff() const { return _mu; }
        float getConfidence() const { return _confidence; }

    private:
        float _mu{0.8f};           // Friction coefficient
        float _confidence{0.0f};   // Estimation confidence
        matrix::Vector2f _theta;   // RLS parameters
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
    void runBasicEstimation(slip_estimation_s &slip);
    void runEKFEstimation(slip_estimation_s &slip);
    float calculateTireModel(float slip, float normal_force);
    void detectSurfaceChange(slip_estimation_s &slip);
};
