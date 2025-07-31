/**
 * @file hbridge_config.h
 *
 * H-Bridge driver board configuration structure
 */

#pragma once

#include <stdint.h>

/**
 * @brief Board-specific H-Bridge configuration
 */
struct hbridge_config_t {
    uint8_t instance_id;        ///< Instance identifier (0, 1, etc.)
    const char *name;           ///< Human readable name
    uint32_t left_dir_gpio;     ///< Left channel direction GPIO
    uint32_t right_dir_gpio;    ///< Right channel direction GPIO
    uint32_t enable_gpio;       ///< H-bridge enable GPIO (0 = not used)
    int left_pwm_channel;       ///< Left channel PWM channel number
    int right_pwm_channel;      ///< Right channel PWM channel number
};
