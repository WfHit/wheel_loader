/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

// PX4 platform includes
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// PX4 library includes
#include <lib/pid/PID.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/motion_planning/TrajectoryConstraints.hpp>
#include <matrix/matrix/Euler.hpp>
#include <matrix/matrix/Vector3.hpp>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_mag_encoder.h>
#include <uORB/topics/bucket_command.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/hbridge_command.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/limit_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/quad_encoder_reset.h>
#include <uORB/topics/sensor_quad_encoder.h>

using namespace time_literals;

class BucketControl : public ModuleBase<BucketControl>, public ModuleParams, public px4::ScheduledWorkItem
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
        MOVE_TO_LOAD_LIMIT,      // Move to load limit (bucket down)
        SETTLE_AT_LOAD,          // Settle at load limit
        FAST_MOVE_TO_DUMP,       // Fast move toward dump limit
        SLOW_APPROACH_DUMP,      // Slow approach to dump limit (bucket up)
        COMPLETE
    };

    // State machine
    State _state{State::UNINITIALIZED};
    ZeroingState _zeroing_state{ZeroingState::MOVE_TO_LOAD_LIMIT};
    void updateStateMachine();

    // Zeroing procedure
    void performZeroing();
    bool _zeroing_complete{false};
    hrt_abstime _zeroing_start_time{0};
    hrt_abstime _zeroing_state_start_time{0};
    float _encoder_zero_offset{0.0f};
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
    void updateKinematicParameters();

    // Four-bar linkage solver for bellcrank -> coupler -> bucket system
    bool solveBucketLinkage(float actuator_length, float boom_angle,
                           float &bucket_angle, float &bellcrank_angle, float &coupler_angle);

    // Motion control
    void updateMotionControl();

    // Clean control architecture functions
    void monitorCommandTarget();
    void monitorBoomChanges();
    void updateActuatorTarget();

    // Hardware interface through existing drivers
    void setMotorCommand(float command);
    void readEncoderFeedback();
    bool checkLimitSwitches();
    bool checkHBridgeStatus();  // Check hbridge status for our motor instance
    void updateHBridgeStatus(); // EKF2-style instance discovery and selection
    void updateEncoderData();   // EKF2-style encoder instance discovery and selection

    // Boom angle monitoring and compensation
    void updateBoomAngleMonitoring();
    float _previous_boom_angle{0.0f};       // Previous boom angle for change detection
    float _target_absolute_bucket_angle{0.0f};  // Target absolute bucket angle (ground-relative)
    bool _boom_angle_changed{false};        // Flag indicating boom has moved
    float _boom_angle_threshold{0.005f};    // Threshold for detecting boom movement (rad)

    // Simplified status publishing
    void publishStatus();

    // Control variables
    float _current_actuator_length{0.0f};    // mm
    float _target_actuator_length{0.0f};     // mm
    float _current_velocity{0.0f};           // mm/s
    float _target_velocity{0.0f};            // mm/s
    float _control_output{0.0f};             // -1 to 1

    // Current angles for status reporting
    float _current_bucket_angle{0.0f};       // rad (relative to boom)

    float _current_boom_angle{0.0f};         // Current boom angle from AS5600 sensor

    hrt_abstime _last_encoder_time{0};
    bool _limit_switch_load{false};          // Load limit (bucket down position)
    bool _limit_switch_dump{false};          // Dump limit (bucket up position)

    // PID controllers
    PID _position_pid;
    PID _velocity_pid;

    // Motion planning components
    VelocitySmoothing _velocity_smoother;
    PositionSmoothing _position_smoother;

    // Motion planning parameters
    float _max_velocity{100.0f};             // mm/s
    float _max_acceleration{200.0f};         // mm/s²
    float _jerk_limit{1000.0f};              // mm/s³

        // uORB subscriptions
    uORB::Subscription _bucket_command_sub{ORB_ID(bucket_command)};
    uORB::SubscriptionMultiArray<sensor_quad_encoder_s> _sensor_quad_encoder_sub{ORB_ID::sensor_quad_encoder};
    uORB::SubscriptionMultiArray<hbridge_status_s> _hbridge_status_sub{ORB_ID::hbridge_status};
    uORB::Subscription _sensor_mag_encoder_sub{ORB_ID(sensor_mag_encoder)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
    uORB::SubscriptionMultiArray<limit_sensor_s> _limit_sensor_sub{ORB_ID::limit_sensor};

    // uORB publications
    uORB::Publication<bucket_status_s> _bucket_status_pub{ORB_ID(bucket_status)};
    uORB::PublicationMulti<hbridge_command_s> _hbridge_command_pub{ORB_ID(hbridge_command)};
    uORB::Publication<quad_encoder_reset_s> _quad_encoder_reset_pub{ORB_ID(quad_encoder_reset)};

    // Motor and sensor indices
    uint8_t _motor_index{0};

    // EKF2-style instance selection for SubscriptionMultiArray
    int _hbridge_status_selected{-1};       // Selected instance for hbridge status
    int _encoder_selected{-1};              // Selected instance for encoder
    int _limit_load_selected{-1};           // Selected instance for load limit sensor
    int _limit_dump_selected{-1};           // Selected instance for dump limit sensor
    hrt_abstime _last_hbridge_status_update{0};
    hrt_abstime _last_encoder_update{0};
    hrt_abstime _last_limit_load_update{0};
    hrt_abstime _last_limit_dump_update{0};

    // Module parameters - Hardware mapping
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::BCT_HBRIDGE_CH>) _param_motor_index,
        (ParamInt<px4::params::BCT_ENC_IDX>) _param_encoder_index,
        (ParamInt<px4::params::BCT_LIM_LOAD>) _param_limit_load_idx,
        (ParamInt<px4::params::BCT_LIM_DUMP>) _param_limit_dump_idx,
        (ParamInt<px4::params::BCT_BOOM_AS5600>) _param_boom_as5600_idx,

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
        (ParamFloat<px4::params::BCT_VEL_PID_P>) _param_vel_pid_p,
        (ParamFloat<px4::params::BCT_VEL_PID_I>) _param_vel_pid_i,
        (ParamFloat<px4::params::BCT_VEL_PID_D>) _param_vel_pid_d,
        (ParamFloat<px4::params::BCT_MAX_VEL>) _param_max_velocity,
        (ParamFloat<px4::params::BCT_MAX_ACC>) _param_max_acceleration,
        (ParamFloat<px4::params::BCT_JERK_LIM>) _param_jerk_limit,

        // Zeroing speeds
        (ParamFloat<px4::params::BCT_ZERO_FAST>) _param_zeroing_fast_speed,
        (ParamFloat<px4::params::BCT_ZERO_SLOW>) _param_zeroing_slow_speed,

        // Boom compensation parameters
        (ParamInt<px4::params::BCT_CTRL_MODE>) _param_control_mode,
        (ParamFloat<px4::params::BCT_LEVEL_P>) _param_level_p_gain,
        (ParamFloat<px4::params::BCT_LEVEL_D>) _param_level_d_gain,
        (ParamFloat<px4::params::BCT_SLOPE_COMP>) _param_slope_compensation,
        (ParamFloat<px4::params::BCT_GRADE_ANG>) _param_grading_angle,
        (ParamFloat<px4::params::BCT_TRANS_ANG>) _param_transport_angle,
        (ParamFloat<px4::params::BCT_STAB_THR>) _param_stability_threshold,
        (ParamFloat<px4::params::BCT_SPILL_THR>) _param_spill_threshold
    )

    )

};
};
