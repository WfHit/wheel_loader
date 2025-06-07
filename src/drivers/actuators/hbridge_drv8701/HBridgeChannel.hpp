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

#include <drivers/drv_pwm_output.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/hbridge_cmd.h>
#include <uORB/topics/hbridge_status.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_arch/io_timer.h>

template<uint8_t CHANNEL_ID>
class HBridgeChannel
{
public:
    struct Config {
        uint8_t pwm_channel;
        uint32_t pwm_freq;
        uint32_t gpio_dir;
        uint32_t gpio_fault;
    };

    HBridgeChannel(const char *name);
    ~HBridgeChannel();

    bool init(const Config &config);
    void update();

    bool is_enabled() const { return _enabled && !_fault_detected; }
    bool has_fault() const { return _fault_detected; }
    uint8_t get_channel_id() const { return CHANNEL_ID; }
    uint8_t get_state() const { return static_cast<uint8_t>(_state); }
    float get_output() const { return _speed_setpoint; }

    void emergency_stop();

private:
    // State machine
    enum class State : uint8_t {
        DISABLED = 0,
        RUNNING = 1,
        FAULT = 2,
        BRAKE = 3
    };

    const char *_name;
    bool _initialized{false};
    Config _hw;

    // State
    State _state{State::DISABLED};
    bool _enabled{false};
    bool _fault_detected{false};
    uint8_t _fault_code{0};

    // Control parameters
    float _speed_setpoint{0.0f};
    uint8_t _direction{0};
    uint64_t _last_cmd_timestamp{0};
    uint32_t _timeout_ms{500};
    float _actual_duty{0.0f};

    // Timing
    hrt_abstime _init_timestamp{0};
    hrt_abstime _last_status_pub{0};
    uint32_t _runtime_ms{0};

    // uORB
    uORB::Subscription _cmd_sub{ORB_ID(hbridge_cmd)};
    uORB::Publication<hbridge_status_s> _status_pub{ORB_ID(hbridge_status)};

    void process_command(const hbridge_cmd_s &cmd);
    void check_fault();
    void control_update();
    void set_pwm_duty(float duty);
    void publish_status();
};

// Template implementation
template<uint8_t CHANNEL_ID>
HBridgeChannel<CHANNEL_ID>::HBridgeChannel(const char *name) : _name(name)
{
}

template<uint8_t CHANNEL_ID>
HBridgeChannel<CHANNEL_ID>::~HBridgeChannel()
{
    if (_initialized) {
        // Disable PWM
        up_pwm_servo_set(_hw.pwm_channel, 0);
    }
}

template<uint8_t CHANNEL_ID>
bool HBridgeChannel<CHANNEL_ID>::init(const Config &config)
{
    _hw = config;

    // Initialize PWM
    up_pwm_servo_init(1 << _hw.pwm_channel);
    up_pwm_servo_set_rate_group_update(1 << _hw.pwm_channel, _hw.pwm_freq);

    // Initialize GPIO
    px4_arch_configgpio(_hw.gpio_dir);
    px4_arch_configgpio(_hw.gpio_fault);

    _initialized = true;
    _init_timestamp = hrt_absolute_time();

    PX4_INFO("%s: initialized on PWM channel %d", _name, _hw.pwm_channel);
    return true;
}

template<uint8_t CHANNEL_ID>
void HBridgeChannel<CHANNEL_ID>::update()
{
    if (!_initialized) return;

    const hrt_abstime now = hrt_absolute_time();

    // Update runtime
    _runtime_ms = (now - _init_timestamp) / 1000;

    // Check for new commands (filter by channel)
    hbridge_cmd_s cmd{};
    if (_cmd_sub.updated()) {
        _cmd_sub.copy(&cmd);
        if (cmd.channel == CHANNEL_ID) {
            process_command(cmd);
        }
    }

    // Check command timeout
    if (_last_cmd_timestamp > 0 &&
        hrt_elapsed_time(&_last_cmd_timestamp) > (_timeout_ms * 1000)) {
        PX4_WARN("%s: command timeout", _name);
        emergency_stop();
    }

    // Poll fault pin
    check_fault();

    // Run control loop
    control_update();

    // Publish status at lower rate (20Hz)
    if (hrt_elapsed_time(&_last_status_pub) > 50000) {
        publish_status();
        _last_status_pub = now;
    }
}

template<uint8_t CHANNEL_ID>
void HBridgeChannel<CHANNEL_ID>::emergency_stop()
{
    _enabled = false;
    _state = State::DISABLED;
    set_pwm_duty(0);
    PX4_WARN("%s: emergency stop", _name);
}

template<uint8_t CHANNEL_ID>
void HBridgeChannel<CHANNEL_ID>::process_command(const hbridge_cmd_s &cmd)
{
    _last_cmd_timestamp = cmd.timestamp;
    _speed_setpoint = math::constrain(cmd.speed, -1.0f, 1.0f);
    _direction = cmd.direction;
    _timeout_ms = math::max(cmd.timeout_ms, 100ul);

    // State transitions based on command
    if (!cmd.enable_request) {
        _enabled = false;
        _state = State::DISABLED;
    } else if (_fault_detected) {
        _state = State::FAULT;
    } else {
        _enabled = true;

        switch (cmd.control_mode) {
        case 0:  // IDLE
            _state = State::DISABLED;
            break;
        case 1:  // NORMAL
            _state = State::RUNNING;
            break;
        case 2:  // BRAKE
            _state = State::BRAKE;
            break;
        case 3:  // COAST
            _state = State::DISABLED;
            break;
        default:
            _state = State::RUNNING;
        }
    }
}

template<uint8_t CHANNEL_ID>
void HBridgeChannel<CHANNEL_ID>::check_fault()
{
    // Poll fault pin (active low)
    if (!px4_arch_gpioread(_hw.gpio_fault)) {
        if (!_fault_detected) {
            _fault_detected = true;
            _state = State::FAULT;
            _fault_code = 0xFF;  // Generic fault
            set_pwm_duty(0);
            PX4_WARN("%s: fault detected", _name);
        }
    } else {
        // Clear fault if pin goes high again
        if (_fault_detected) {
            _fault_detected = false;
            _fault_code = 0;
            PX4_INFO("%s: fault cleared", _name);
        }
    }
}

template<uint8_t CHANNEL_ID>
void HBridgeChannel<CHANNEL_ID>::control_update()
{
    switch (_state) {
    case State::DISABLED:
        set_pwm_duty(0);
        px4_arch_gpiowrite(_hw.gpio_dir, 0);
        _actual_duty = 0.0f;
        break;

    case State::RUNNING:
        {
            float duty = fabsf(_speed_setpoint);

            // Apply direction
            if (_speed_setpoint >= 0) {
                px4_arch_gpiowrite(_hw.gpio_dir, _direction ? 0 : 1);
            } else {
                px4_arch_gpiowrite(_hw.gpio_dir, _direction ? 1 : 0);
            }

            set_pwm_duty(duty);
            _actual_duty = duty;
        }
        break;

    case State::BRAKE:
        // Dynamic braking - both outputs low
        set_pwm_duty(1.0f);  // PWM high
        px4_arch_gpiowrite(_hw.gpio_dir, 0);  // DIR low
        _actual_duty = 0.0f;
        break;

    case State::FAULT:
        set_pwm_duty(0);
        px4_arch_gpiowrite(_hw.gpio_dir, 0);
        _actual_duty = 0.0f;
        break;
    }
}

template<uint8_t CHANNEL_ID>
void HBridgeChannel<CHANNEL_ID>::set_pwm_duty(float duty)
{
    // Convert to microseconds (assuming 400Hz PWM)
    uint16_t pwm_value = 1000 + (uint16_t)(duty * 1000.0f);
    up_pwm_servo_set(_hw.pwm_channel, pwm_value);
}

template<uint8_t CHANNEL_ID>
void HBridgeChannel<CHANNEL_ID>::publish_status()
{
    hbridge_status_s status{};
    status.timestamp = hrt_absolute_time();
    status.channel = CHANNEL_ID;
    status.actual_duty = _actual_duty;
    status.fault = _fault_detected;
    status.fault_code = _fault_code;
    status.state = static_cast<uint8_t>(_state);
    status.runtime_ms = _runtime_ms;

    _status_pub.publish(status);
}
