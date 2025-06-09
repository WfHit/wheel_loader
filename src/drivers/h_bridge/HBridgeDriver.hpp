/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include <lib/perf/perf_counter.h>

#include "EnableManager.hpp"
#include "HBridgeChannel.hpp"

class HBridgeDriver : public ModuleBase<HBridgeDriver>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    HBridgeDriver();
    ~HBridgeDriver() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    int init();
    void Run() override;

private:
    // Hardware configuration
    struct {
        uint32_t pwm_frequency{25000};     // 25kHz default
        uint32_t update_rate_hz{200};      // 200Hz control loop
    } _config;

    // Channel handlers
    HBridgeChannel<0> *_channel0{nullptr};
    HBridgeChannel<1> *_channel1{nullptr};
    EnableManager *_enable_manager{nullptr};

    // Performance counters
    perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

    // Module parameters
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::HBRDG_PWM_FREQ>) _param_pwm_freq,
        (ParamInt<px4::params::HBRDG_UPDATE_RT>) _param_update_rate,
        (ParamInt<px4::params::HBRDG_PWM_CH0>) _param_pwm_ch0,
        (ParamInt<px4::params::HBRDG_PWM_CH1>) _param_pwm_ch1,
        (ParamInt<px4::params::HBRDG_GPIO_EN>) _param_gpio_enable,
        (ParamInt<px4::params::HBRDG_GPIO_DIR0>) _param_gpio_dir0,
        (ParamInt<px4::params::HBRDG_GPIO_DIR1>) _param_gpio_dir1,
        (ParamInt<px4::params::HBRDG_GPIO_FLT0>) _param_gpio_fault0,
        (ParamInt<px4::params::HBRDG_GPIO_FLT1>) _param_gpio_fault1,
        (ParamInt<px4::params::HBRDG_CH0_LM_MIN>) _param_ch0_lim_min,
        (ParamInt<px4::params::HBRDG_CH0_LM_MAX>) _param_ch0_lim_max,
        (ParamInt<px4::params::HBRDG_CH1_LM_MIN>) _param_ch1_lim_min,
        (ParamInt<px4::params::HBRDG_CH1_LM_MAX>) _param_ch1_lim_max,
        (ParamInt<px4::params::HBRDG_CH0_AL_MIN>) _param_ch0_into_min,
        (ParamInt<px4::params::HBRDG_CH0_AL_MAX>) _param_ch0_into_max,
        (ParamInt<px4::params::HBRDG_CH1_AL_MIN>) _param_ch1_into_min,
        (ParamInt<px4::params::HBRDG_CH1_AL_MAX>) _param_ch1_into_max
    )

    void updateParams() override;
    void configure_limits();

    bool _initialized{false};
};
