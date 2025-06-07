#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <lib/pid/pid.h>
#include <lib/trajectory/trajectory.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/bucket_command.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/limit_sensor.h>
#include <uORB/topics/parameter_update.h>

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
    float generateSCurveSetpoint(float current, float target, float &velocity, float &acceleration);

    // Hardware interface through existing drivers
    void setMotorCommand(float command);
    void readEncoderFeedback();
    bool checkLimitSwitches();

    // Control variables
    float _current_actuator_length{0.0f};    // mm
    float _target_actuator_length{0.0f};     // mm
    float _current_velocity{0.0f};           // mm/s
    float _target_velocity{0.0f};            // mm/s
    float _control_output{0.0f};             // -1 to 1

    // Current angles for status reporting
    float _current_bucket_angle{0.0f};       // rad (relative to boom)
    float _current_ground_angle{0.0f};       // rad (relative to ground)

    int64_t _encoder_count{0};
    int64_t _last_encoder_count{0};
    hrt_abstime _last_encoder_time{0};
    bool _limit_switch_coarse{false};        // Down position (coarse)
    bool _limit_switch_fine{false};          // Up position (fine)

    // PID controller
    PID _position_pid;

    // S-curve planner parameters
    float _max_velocity{100.0f};             // mm/s
    float _max_acceleration{200.0f};         // mm/s²
    float _jerk_limit{1000.0f};              // mm/s³

    // uORB subscriptions
    uORB::Subscription _bucket_cmd_sub{ORB_ID(bucket_command)};
    uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
    uORB::Subscription _wheel_encoders_sub{ORB_ID(wheel_encoders)};
    uORB::Subscription _limit_sensor_sub{ORB_ID(limit_sensor)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

    // uORB publications
    uORB::Publication<bucket_status_s> _bucket_status_pub{ORB_ID(bucket_status)};
    uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

    // Motor and sensor indices
    uint8_t _motor_index{0};

    // Module parameters - Hardware mapping
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::BCT_MOT_IDX>) _param_motor_index,
        (ParamInt<px4::params::BCT_ENC_IDX>) _param_encoder_index,
        (ParamInt<px4::params::BCT_LIM_COARSE>) _param_limit_coarse_idx,
        (ParamInt<px4::params::BCT_LIM_FINE>) _param_limit_fine_idx,

        // Actuator attachment point (relative to chassis/boom base)
        (ParamFloat<px4::params::BCT_ACT_BASE_X>) _param_actuator_base_x,
        (ParamFloat<px4::params::BCT_ACT_BASE_Y>) _param_actuator_base_y,

        // Drive linkage attachment to boom
        (ParamFloat<px4::params::BCT_BELLCRANK_BOOM_X>) _param_bellcrank_boom_x,
        (ParamFloat<px4::params::BCT_BELLCRANK_BOOM_Y>) _param_bellcrank_boom_y,

        // Bucket pivot on boom
        (ParamFloat<px4::params::BCT_BKT_BOOM_X>) _param_bucket_boom_pivot_x,
        (ParamFloat<px4::params::BCT_BKT_BOOM_Y>) _param_bucket_boom_pivot_y,

        // Linkage dimensions
        (ParamFloat<px4::params::BCT_BELLCRANK_LENGTH>) _param_bellcrank_length,
        (ParamFloat<px4::params::BCT_COUPLER_LENGTH>) _param_coupler_length,
        (ParamFloat<px4::params::BCT_ACT_OFFSET>) _param_actuator_offset,
        (ParamFloat<px4::params::BCT_BKT_ARM_LEN>) _param_bucket_arm_length,
        (ParamFloat<px4::params::BCT_BELLCRANK_INT_ANG>) _param_bellcrank_internal_angle,
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
        (ParamFloat<px4::params::BCT_ZERO_SLOW>) _param_zeroing_slow_speed
    )
};
