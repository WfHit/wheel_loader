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

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionData.hpp>

// Chassis control messages
#include <uORB/topics/wheel_encoder.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>

// Boom and bucket control messages
#include <uORB/topics/boom_command.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_command.h>
#include <uORB/topics/bucket_status.h>

// Safety and monitoring
#include <uORB/topics/safety.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/module_status.h>

using namespace time_literals;

class WheelLoaderController : public ModuleBase<WheelLoaderController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    WheelLoaderController();
    ~WheelLoaderController() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;

    // Main control modes
    enum class ControlMode {
        MANUAL,
        SEMI_AUTO,
        AUTO,
        EMERGENCY_STOP
    };

    // Wheel loader operational states
    enum class SystemState {
        INITIALIZING,
        STANDBY,
        OPERATION,
        MAINTENANCE,
        ERROR,
        EMERGENCY
    };

    // Work cycle states for autonomous operation
    enum class WorkCycleState {
        IDLE,
        APPROACH_PILE,
        LOAD_MATERIAL,
        TRANSPORT,
        DUMP_MATERIAL,
        RETURN
    };

    struct ChassisState {
        float front_wheel_velocity{0.0f};
        float rear_wheel_velocity{0.0f};
        float steering_angle{0.0f};
        float articulation_angle{0.0f};
        bool emergency_stop{false};
        uint64_t last_update{0};
    };

    struct BoomBucketState {
        float boom_angle{0.0f};
        float boom_velocity{0.0f};
        float boom_load{0.0f};
        float bucket_angle{0.0f};
        float bucket_velocity{0.0f};
        float bucket_relative_angle{0.0f};
        bool boom_ready{false};
        bool bucket_ready{false};
        uint64_t last_update{0};
    };

    struct WorkCycleParams {
        float approach_speed{0.5f};        // m/s
        float loading_speed{0.2f};         // m/s
        float transport_speed{2.0f};       // m/s
        float dump_height{2.5f};           // m
        float dump_angle{60.0f};           // degrees - bucket tip angle for dumping
        float load_angle{-10.0f};          // degrees - boom angle for loading
        float transport_angle{30.0f};      // degrees - boom angle for transport
        float carry_angle{30.0f};          // degrees - boom angle for carry position
        float load_detection_threshold{0.8f}; // normalized load
    };

    // Subscription handlers
    void updateVehicleCommands();
    void updateManualControls();
    void updateChassisStatus();
    void updateBoomBucketStatus();
    void updateSafetyStatus();
    void updateParameters();

    // Control functions
    void processManualControl();
    void processSemiAutoControl();
    void processAutoControl();
    void processEmergencyStop();

    // Chassis control
    void controlChassis(const manual_control_setpoint_s &manual);
    void publishChassisCommands();

    // Boom and bucket control
    void controlBoomBucket(const manual_control_setpoint_s &manual);
    void publishBoomBucketCommands();

    // Autonomous work cycle functions
    void executeWorkCycle();
    void transitionWorkCycleState(WorkCycleState new_state);
    bool isWorkCycleComplete();

    // Safety and monitoring
    void performSafetyChecks();
    void handleEmergencyStop();
    void publishSystemStatus();

    // AHRS-enhanced operational modes
    void configureAHRSBucketControl(uint8_t mode, float target_angle = 0.0f);
    void updateBucketControlForTerrain();
    bool isStabilityLimitRequired();
    void logAHRSStatus();

    // Utility functions
    void resetControllers();
    bool allSubsystemsReady();
    void logSystemPerformance();

    // Parameters
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::WL_WHEEL_MAX_VEL>) _param_wheel_max_vel,
        (ParamFloat<px4::params::WL_STEER_MAX_ANGLE>) _param_steer_max_angle,
        (ParamFloat<px4::params::WL_BOOM_MAX_VEL>) _param_boom_max_vel,
        (ParamFloat<px4::params::WL_BUCKET_MAX_VEL>) _param_bucket_max_vel,
        (ParamFloat<px4::params::WL_SAFETY_TIMEOUT>) _param_safety_timeout,
        (ParamBool<px4::params::WL_AUTO_ENABLED>) _param_auto_enabled,
        (ParamFloat<px4::params::WL_WORK_CYCLE_SPD>) _param_work_cycle_speed,
        (ParamFloat<px4::params::WL_LOAD_THRESHOLD>) _param_load_threshold
    )

    // Subscriptions
    uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
    uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
    uORB::Subscription _wheel_encoder_sub{ORB_ID(wheel_encoder)};
    uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
    uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};
    uORB::Subscription _safety_sub{ORB_ID(safety)};
    uORB::Subscription _system_power_sub{ORB_ID(system_power)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

    // Publications
    uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
    uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
    uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
    uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
    uORB::Publication<boom_command_s> _boom_command_pub{ORB_ID(boom_command)};
    uORB::Publication<bucket_command_s> _bucket_command_pub{ORB_ID(bucket_command)};
    uORB::Publication<module_status_s> _module_status_pub{ORB_ID(module_status)};

    // State variables
    ControlMode _control_mode{ControlMode::MANUAL};
    SystemState _system_state{SystemState::INITIALIZING};
    WorkCycleState _work_cycle_state{WorkCycleState::IDLE};

    ChassisState _chassis_state{};
    BoomBucketState _boom_bucket_state{};
    WorkCycleParams _work_cycle_params{};

    // Control outputs
    actuator_motors_s _actuator_motors{};
    actuator_servos_s _actuator_servos{};
    boom_command_s _boom_command{};
    bucket_command_s _bucket_command{};
    vehicle_status_s _vehicle_status{};
    module_status_s _module_status{};

    // Timing and safety
    uint64_t _last_manual_control{0};
    uint64_t _last_safety_check{0};
    uint64_t _work_cycle_start_time{0};
    uint64_t _state_transition_time{0};
    bool _emergency_stop_active{false};
    bool _armed{false};

    // Performance counters
    perf_counter_t _loop_perf{nullptr};
    perf_counter_t _control_latency_perf{nullptr};

    static constexpr uint32_t MAIN_LOOP_INTERVAL_US{20000}; // 50 Hz
    static constexpr uint32_t SAFETY_CHECK_INTERVAL_US{100000}; // 10 Hz
    static constexpr uint32_t STATUS_PUBLISH_INTERVAL_US{100000}; // 10 Hz
    static constexpr float EMERGENCY_STOP_TIMEOUT_S{2.0f};
    static constexpr float CONTROL_TIMEOUT_S{0.5f};
};
