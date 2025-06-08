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

#include "EnableManager.hpp"

EnableManager::EnableManager()
{
}

bool EnableManager::init(uint32_t gpio_enable)
{
    _gpio_enable = gpio_enable;

    // Configure enable pin as output, initially disabled
    px4_arch_configgpio(_gpio_enable);
    px4_arch_gpiowrite(_gpio_enable, 0);

    _initialized = true;
    return true;
}

void EnableManager::update(bool ch0_request, bool ch1_request,
                          uint8_t ch0_state, uint8_t ch1_state)
{
    if (!_initialized) return;

    // Check for emergency stop commands
    vehicle_command_s vcmd;
    if (_vehicle_cmd_sub.update(&vcmd)) {
        if (vcmd.command == vehicle_command_s::VEHICLE_CMD_DO_MOTOR_TEST) {
            if (vcmd.param1 < 0) {  // Negative value = emergency stop
                _emergency_stop = true;
                PX4_WARN("Emergency stop activated");
            }
        }
    }

    // Update enable logic
    bool should_enable = (ch0_request || ch1_request) && !_emergency_stop;

    if (should_enable != _enabled) {
        _enabled = should_enable;
        px4_arch_gpiowrite(_gpio_enable, _enabled ? 1 : 0);

        PX4_INFO("Enable pin: %s", _enabled ? "HIGH" : "LOW");
    }

    // Publish system status
    hrt_abstime now = hrt_absolute_time();
    if (hrt_elapsed_time(&_last_pub_time) > 50000) {  // 20Hz
        publish_status(ch0_request, ch1_request, ch0_state, ch1_state);
        _last_pub_time = now;
    }
}

void EnableManager::publish_status(bool ch0_req, bool ch1_req,
                                  uint8_t ch0_state, uint8_t ch1_state)
{
    hbridge_system_s sys{};
    sys.timestamp = hrt_absolute_time();
    sys.enabled = _enabled;
    sys.emergency_stop = _emergency_stop;
    sys.temperature = read_board_temperature();
    sys.safety_override = _safety_override;

    // Channel states
    sys.channel[0].enabled = ch0_req && _enabled;
    sys.channel[0].state = ch0_state;
    sys.channel[0].limit_min_active = is_limit_active(_limit_config[0].min_limit_instance);
    sys.channel[0].limit_max_active = is_limit_active(_limit_config[0].max_limit_instance);

    sys.channel[1].enabled = ch1_req && _enabled;
    sys.channel[1].state = ch1_state;
    sys.channel[1].limit_min_active = is_limit_active(_limit_config[1].min_limit_instance);
    sys.channel[1].limit_max_active = is_limit_active(_limit_config[1].max_limit_instance);

    _system_pub.publish(sys);
}

float EnableManager::read_board_temperature()
{
    // TODO: Read actual temperature sensor
    return 25.0f;  // Placeholder
}

void EnableManager::set_limit_config(uint8_t channel, const LimitConfig& config)
{
    if (channel < 2) {
        _limit_config[channel] = config;
        PX4_INFO("Channel %d limits: min=%d, max=%d",
                 channel, config.min_limit_instance, config.max_limit_instance);
    }
}

bool EnableManager::is_limit_active(uint8_t instance)
{
    if (instance >= 8 || instance == 255) {
        return false;
    }

    limit_sensor_s sensor;
    if (_limit_sensor_sub[instance].copy(&sensor)) {
        // Only trust healthy sensors
        return sensor.healthy && sensor.state;
    }

    return false;
}

void EnableManager::check_safety_override()
{
    system_safety_s safety;
    if (_system_safety_sub.copy(&safety)) {
        // Check if we're in a mode that allows limit override
        _safety_override = (safety.safety_mode == system_safety_s::SAFETY_MODE_MANUAL_OVERRIDE) ||
                          (safety.safety_mode == system_safety_s::SAFETY_MODE_ZEROING);
    }
}

bool EnableManager::check_motion_allowed(uint8_t channel, float command)
{
    if (channel >= 2 || !_enabled) {
        return false;
    }

    // Update limit states periodically (every 10ms)
    hrt_abstime now = hrt_absolute_time();
    if (now - _last_limit_check > 10000) {
        check_safety_override();
        _last_limit_check = now;
    }

    const LimitConfig& config = _limit_config[channel];

    // Check min limit (negative motion)
    if (command < -0.01f && config.min_limit_instance != 255) {
        if (is_limit_active(config.min_limit_instance)) {
            if (!config.allow_into_min && !_safety_override) {
                PX4_WARN_THROTTLE(1000, "Ch%d motion blocked by min limit %d",
                                 channel, config.min_limit_instance);
                return false;
            }
        }
    }

    // Check max limit (positive motion)
    if (command > 0.01f && config.max_limit_instance != 255) {
        if (is_limit_active(config.max_limit_instance)) {
            if (!config.allow_into_max && !_safety_override) {
                PX4_WARN_THROTTLE(1000, "Ch%d motion blocked by max limit %d",
                                 channel, config.max_limit_instance);
                return false;
            }
        }
    }

    return true;
}
