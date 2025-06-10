#pragma once

// PX4 platform includes
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

// PX4 library includes
#include <lib/pid/PID.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/motion_planning/TrajectoryConstraints.hpp>
#include <matrix/matrix/Euler.hpp>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_command.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/hbridge_command.h>
#include <uORB/topics/limit_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>

using namespace time_literals;

class BucketControl : public ModuleBase<BucketControl>, public ModuleParams, public px4::WorkItem
{
public:
    BucketControl();
    ~BucketControl() override = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;

    enum class State {
        UNINITIALIZED,
        ZEROING,
        READY,
        MOVING,
        ERROR
    };

    enum class ZeroingState {
        MOVE_TO_COARSE_LIMIT,
        SETTLE_AT_COARSE,
        FAST_MOVE_TO_FINE,
        SLOW_APPROACH_FINE,
        COMPLETE
    };

    enum class ControlMode : uint8_t {
        MANUAL = 0,
        AUTO_LEVEL = 1,
        SLOPE_COMPENSATION = 2,
        GRADING = 3,
        TRANSPORT = 4
    };

    // State machine
    State _state{State::UNINITIALIZED};
    ZeroingState _zeroing_state{ZeroingState::MOVE_TO_COARSE_LIMIT};
    void updateStateMachine();

    // Zeroing procedure
    void performZeroing();
    bool _zeroing_complete{false};
    hrt_abstime _zeroing_start_time{0};
    hrt_abstime _zeroing_state_start_time{0};
    int64_t _encoder_zero_offset{0};
    float _zeroing_position{0.0f};
    float _zeroing_velocity{0.0f};
    float _zeroing_target{0.0f};

    // Kinematic model structure - updated names based on image
    struct BucketKinematics {
        // Attachment points (all in mm, relative to boom pivot)
        float actuator_base_x;          // Actuator base attachment X (chassis)
        float actuator_base_y;          // Actuator base attachment Y (chassis)
        float bellcrank_boom_x;         // Bellcrank attachment to boom X
        float bellcrank_boom_y;         // Bellcrank attachment to boom Y
        float bucket_boom_pivot_x;      // Bucket pivot point on boom X
        float bucket_boom_pivot_y;      // Bucket pivot point on boom Y

        // Linkage lengths (mm)
        float bellcrank_length;         // Length of bellcrank arm to coupler attachment
        float coupler_length;           // Length of coupler (connects bellcrank to bucket)
        float actuator_offset;          // Distance from bellcrank pivot to actuator attachment
        float bucket_arm_length;        // Distance from bucket pivot to coupler attachment point

        // Bellcrank and bucket geometry (angles in radians)
        float bellcrank_internal_angle; // Fixed angle between bellcrank arms (actuator to linkage)
        float bucket_offset;            // Angular offset of bucket arm from bucket reference

        // Actuator geometry
        float actuator_min_length;      // Minimum actuator length
        float actuator_max_length;      // Maximum actuator length

        // Boom compensation
        float boom_length;              // Length from boom pivot to bucket pivot
    } _kinematics;

    // Enhanced kinematic calculations with boom compensation
    float actuatorLengthToBucketAngle(float actuator_length, float boom_angle = 0.0f);
    float bucketAngleToActuatorLength(float bucket_angle, float boom_angle = 0.0f);
    float compensateBoomAngle(float target_ground_angle, float boom_angle);
    void updateKinematicParameters();

    // Four-bar linkage solver for bellcrank -> coupler -> bucket system
    bool solveBucketLinkage(float actuator_length, float boom_angle,
                           float &bucket_angle, float &bellcrank_angle, float &coupler_angle);

    // Motion control
    void updateMotionControl();
    void updateTrajectorySetpoint(float dt);

    // Hardware interface through existing drivers
    void setMotorCommand(float command);
    void readEncoderFeedback();
    bool checkLimitSwitches();

    // AHRS Integration methods
    void updateAHRSData();
    float calculateGroundRelativeAngle(float bucket_boom_angle);
    float compensateForSlope(float target_angle);
    void updateStabilityFactor();
    void applyAntiSpillControl();
    float limitMovementForStability(float command);
    void performAutoLevel();
    void performGradingControl();
    void publishStatus();

    // Control variables
    float _current_actuator_length{0.0f};    // mm
    float _target_actuator_length{0.0f};     // mm
    float _current_velocity{0.0f};           // mm/s
    float _target_velocity{0.0f};            // mm/s
    float _control_output{0.0f};             // -1 to 1

    // Current angles for status reporting
    float _current_bucket_angle{0.0f};       // rad (relative to boom)
    float _current_ground_angle{0.0f};       // rad (relative to ground)

    // AHRS data
    float _machine_roll{0.0f};
    float _machine_pitch{0.0f};
    float _machine_yaw{0.0f};
    float _angular_rate_x{0.0f};
    float _angular_rate_y{0.0f};
    float _angular_rate_z{0.0f};
    float _acceleration_x{0.0f};
    float _acceleration_y{0.0f};
    float _acceleration_z{0.0f};

    // Advanced control features
    ControlMode _control_mode{ControlMode::MANUAL};
    float _target_ground_angle{0.0f};        // Target angle relative to ground
    float _grading_angle{0.0f};              // Desired cutting angle for grading
    float _transport_angle{0.0f};            // Safe angle for transport
    float _stability_factor{1.0f};           // Dynamic stability scaling factor
    float _current_boom_angle{0.0f};         // Current boom angle from boom status
    bool _anti_spill_active{false};
    bool _stability_warning{false};

    int64_t _encoder_count{0};
    int64_t _last_encoder_count{0};
    hrt_abstime _last_encoder_time{0};
    bool _limit_switch_coarse{false};        // Down position (coarse)
    bool _limit_switch_fine{false};          // Up position (fine)

    // PID controller
    PID _position_pid;

    // Motion planning components
    VelocitySmoothing _velocity_smoother;
    PositionSmoothing _position_smoother;

    // Motion planning parameters
    float _max_velocity{100.0f};             // mm/s
    float _max_acceleration{200.0f};         // mm/s²
    float _jerk_limit{1000.0f};              // mm/s³

    // uORB subscriptions
    uORB::Subscription _bucket_cmd_sub{ORB_ID(bucket_command)};
    uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
    uORB::Subscription _sensor_quad_encoder_sub{ORB_ID(sensor_quad_encoder)};
    uORB::Subscription _limit_sensor_sub{ORB_ID(limit_sensor)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

    // uORB publications
    uORB::Publication<bucket_status_s> _bucket_status_pub{ORB_ID(bucket_status)};
    uORB::Publication<hbridge_command_s> _hbridge_command_pub{ORB_ID(hbridge_command)};

    // Motor and sensor indices
    uint8_t _motor_index{0};

    // Module parameters - Hardware mapping
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::BCT_HBRIDGE_CH>) _param_motor_index,
        (ParamInt<px4::params::BCT_ENC_IDX>) _param_encoder_index,
        (ParamInt<px4::params::BCT_LIM_COARSE>) _param_limit_coarse_idx,
        (ParamInt<px4::params::BCT_LIM_FINE>) _param_limit_fine_idx,

        // Actuator attachment point (relative to chassis/boom base)
        (ParamFloat<px4::params::BCT_ACT_BASE_X>) _param_actuator_base_x,
        (ParamFloat<px4::params::BCT_ACT_BASE_Y>) _param_actuator_base_y,

        // Drive linkage attachment to boom
        (ParamFloat<px4::params::BCT_BCRK_BOOM_X>) _param_bellcrank_boom_x,
        (ParamFloat<px4::params::BCT_BCRK_BOOM_Y>) _param_bellcrank_boom_y,

        // Bucket pivot on boom
        (ParamFloat<px4::params::BCT_BKT_BOOM_X>) _param_bucket_boom_pivot_x,
        (ParamFloat<px4::params::BCT_BKT_BOOM_Y>) _param_bucket_boom_pivot_y,

        // Linkage dimensions
        (ParamFloat<px4::params::BCT_BCRK_LENGTH>) _param_bellcrank_length,
        (ParamFloat<px4::params::BCT_COUP_LENGTH>) _param_coupler_length,
        (ParamFloat<px4::params::BCT_ACT_OFFSET>) _param_actuator_offset,
        (ParamFloat<px4::params::BCT_BKT_ARM_LEN>) _param_bucket_arm_length,
        (ParamFloat<px4::params::BCT_BCRK_INT_ANG>) _param_bellcrank_internal_angle,
        (ParamFloat<px4::params::BCT_BKT_OFFSET>) _param_bucket_offset,
        (ParamFloat<px4::params::BCT_BOOM_LENGTH>) _param_boom_length,

        // Actuator limits
        (ParamFloat<px4::params::BCT_ACT_MIN>) _param_actuator_min,
        (ParamFloat<px4::params::BCT_ACT_MAX>) _param_actuator_max,
        (ParamFloat<px4::params::BCT_ANG_MIN>) _param_angle_min,
        (ParamFloat<px4::params::BCT_ANG_MAX>) _param_angle_max,
        (ParamFloat<px4::params::BCT_ENC_SCALE>) _param_encoder_scale,

        // Control parameters
        (ParamFloat<px4::params::BCT_PID_P>) _param_pid_p,
        (ParamFloat<px4::params::BCT_PID_I>) _param_pid_i,
        (ParamFloat<px4::params::BCT_PID_D>) _param_pid_d,
        (ParamFloat<px4::params::BCT_MAX_VEL>) _param_max_velocity,
        (ParamFloat<px4::params::BCT_MAX_ACC>) _param_max_acceleration,
        (ParamFloat<px4::params::BCT_JERK_LIM>) _param_jerk_limit,

        // Zeroing speeds
        (ParamFloat<px4::params::BCT_ZERO_FAST>) _param_zeroing_fast_speed,
        (ParamFloat<px4::params::BCT_ZERO_SLOW>) _param_zeroing_slow_speed,

        // AHRS Integration parameters
        (ParamInt<px4::params::BCT_AHRS_EN>) _param_ahrs_enabled,
        (ParamInt<px4::params::BCT_CTRL_MODE>) _param_control_mode,
        (ParamFloat<px4::params::BCT_LEVEL_P>) _param_level_p_gain,
        (ParamFloat<px4::params::BCT_LEVEL_D>) _param_level_d_gain,
        (ParamFloat<px4::params::BCT_SLOPE_COMP>) _param_slope_compensation,
        (ParamFloat<px4::params::BCT_GRADE_ANG>) _param_grading_angle,
        (ParamFloat<px4::params::BCT_TRANS_ANG>) _param_transport_angle,
        (ParamFloat<px4::params::BCT_STAB_THR>) _param_stability_threshold,
        (ParamFloat<px4::params::BCT_SPILL_THR>) _param_spill_threshold
    )
};
