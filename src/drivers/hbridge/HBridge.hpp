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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_pwm_output.h>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/hbridge_command.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/parameter_update.h>
#include <lib/parameters/param.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>

class HBridge : public ModuleBase<HBridge>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    HBridge(int instance_id);
    ~HBridge() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    int print_status() override;

    void Run() override;
    bool init();

    static constexpr uint8_t MAX_CHANNELS = 2;
    static constexpr uint8_t MAX_INSTANCES = 2; // For multiple boards
    static constexpr uint32_t SCHEDULE_INTERVAL = 10_ms; // 100Hz

private:
    // Instance management
    static HBridge *_objects[MAX_INSTANCES];
    const int _instance_id;

    // Channel data structure
    struct ChannelData {
        uint32_t dir_gpio{0};
        unsigned pwm_channel{0};
        uint32_t pwm_mask{0};
        float current_duty_cycle{0.0f};
        bool enabled{false};
        bool initialized{false};
    };

    ChannelData _channels[MAX_CHANNELS];

    // PWM management
    static bool _pwm_initialized;
    static px4::atomic<int> _running_instances;

    // uORB
    uORB::SubscriptionMultiArray<hbridge_command_s, hbridge_command_s::ORB_QUEUE_LENGTH> _command_sub{ORB_ID(hbridge_command)};
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
    orb_advert_t _status_pub{nullptr};
    int _status_instance{ORB_PRIO_MAX};

    // Parameters
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::HBRIDGE_CH0_PWM>) _param_ch0_pwm,
        (ParamInt<px4::params::HBRIDGE_CH1_PWM>) _param_ch1_pwm,
        (ParamFloat<px4::params::HBRIDGE_PWM_FREQ>) _param_pwm_freq,
        (ParamFloat<px4::params::HBRIDGE_CH0_MIN>) _param_ch0_min,
        (ParamFloat<px4::params::HBRIDGE_CH0_MAX>) _param_ch0_max,
        (ParamFloat<px4::params::HBRIDGE_CH1_MIN>) _param_ch1_min,
        (ParamFloat<px4::params::HBRIDGE_CH1_MAX>) _param_ch1_max
    )

    // Performance counters
    perf_counter_t _loop_perf;
    perf_counter_t _command_perf;

    // State tracking
    uint32_t _command_count{0};
    uint32_t _error_count{0};
    hrt_abstime _last_command_time{0};
    bool _is_running{false};

    // Methods
    bool configure_channel(int channel);
    void set_channel_speed(int channel, float duty_cycle);
    void update_channel_direction(int channel, bool forward);
    void process_commands();
    void publish_status();
    void parameters_update();

    // Getters for channel parameters
    int get_pwm_channel(int ch) const;
    float get_pwm_min(int ch) const;
    float get_pwm_max(int ch) const;

    // Utility methods
    static HBridge *instantiate(int instance);
    void usage(const char *reason = nullptr);
};
