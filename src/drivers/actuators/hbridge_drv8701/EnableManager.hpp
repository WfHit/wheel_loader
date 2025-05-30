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

#include <px4_platform_common/px4_config.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/hbridge_system.h>
#include <uORB/topics/vehicle_command.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

class EnableManager
{
public:
    EnableManager();
    ~EnableManager() = default;

    bool init(uint32_t gpio_enable);
    void update(bool ch0_request, bool ch1_request,
                uint8_t ch0_state, uint8_t ch1_state);

    bool is_enabled() const { return _enabled; }
    bool is_emergency_stop() const { return _emergency_stop; }

    void clear_emergency_stop() { _emergency_stop = false; }

private:
    bool _initialized{false};
    uint32_t _gpio_enable{0};
    bool _enabled{false};
    bool _emergency_stop{false};

    hrt_abstime _last_pub_time{0};

    uORB::Publication<hbridge_system_s> _system_pub{ORB_ID(hbridge_system)};
    uORB::Subscription _vehicle_cmd_sub{ORB_ID(vehicle_command)};

    void publish_status(bool ch0_req, bool ch1_req,
                       uint8_t ch0_state, uint8_t ch1_state);
    float read_board_temperature();
};
