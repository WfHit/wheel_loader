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

/**
 * @file hw_config.h
 *
 * CUAV X7+ WL (Wheel Loader) Controller hardware configuration definitions
 */

#pragma once

#include <stdint.h>

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Hardware revision and version detection */
#define HW_REV_FMUV5X               0x00
#define HW_REV_FMUV5X_V1            0x01

/* Hardware identification */
#define CUAV_X7PLUS_WL_REV          0x10
#define CUAV_X7PLUS_WL_VERSION      0x01

/* UART Configuration for Wheel Loader */
#define UART_GPS1_DEV               "/dev/ttyS0"    /* GPS primary */
#define UART_GPS2_DEV               "/dev/ttyS1"    /* GPS secondary/RTK - NOT USED */
#define UART_TEL1_DEV               "/dev/ttyS1"    /* Telemetry 1 (MAVLink ground station) */
#define UART_TEL2_DEV               "/dev/ttyS2"    /* Telemetry 2 / NXT Controller 1 */
#define UART_TEL4_DEV               "/dev/ttyS3"    /* UART4 / NXT Controller 2 */
#define UART_NXT1_DEV               "/dev/ttyS2"    /* NXT front controller (via TELEM2) */
#define UART_NXT2_DEV               "/dev/ttyS3"    /* NXT rear controller (via UART4) */

/* SPI Bus Configuration */
#define SPI_BUS_SENSORS1            1               /* Primary sensor bus */
#define SPI_BUS_SENSORS2            2               /* Secondary sensor bus */

/* I2C Bus Configuration */
#define I2C_BUS_INTERNAL            1               /* Internal peripherals */
#define I2C_BUS_EXTERNAL            2               /* External expansion */
#define I2C_BUS_GPS                 3               /* GPS and compass */

/* ADC Configuration */
#define ADC_BATTERY1_VOLTAGE_CHANNEL    2
#define ADC_BATTERY1_CURRENT_CHANNEL    3
#define ADC_BATTERY2_VOLTAGE_CHANNEL    4
#define ADC_BATTERY2_CURRENT_CHANNEL    5
#define ADC_5V_RAIL_SENSE              10
#define ADC_SPARE_1                    11
#define ADC_SPARE_2                    12

/* PWM Configuration for Wheel Loader */
#define PWM_OUTPUT_MAIN_CHANNELS        8           /* Main PWM outputs */
#define PWM_OUTPUT_AUX_CHANNELS         6           /* Auxiliary PWM outputs */

/* CAN Bus Configuration */
#define CAN_BUS_1                       0           /* Primary CAN bus */
#define CAN_BUS_2                       1           /* Secondary CAN bus */

/* Safety and Status LEDs */
#define LED_STATUS_RED                  0           /* System error */
#define LED_STATUS_GREEN                1           /* System OK */
#define LED_STATUS_BLUE                 2           /* System armed/disarmed */

/* Wheel Loader Specific Hardware */
#define WL_MAX_HYDRAULIC_PRESSURE       250         /* Bar */
#define WL_MAX_ENGINE_RPM               2200        /* RPM */
#define WL_MAX_TRAVEL_SPEED             20          /* km/h */
#define WL_MAX_STEERING_ANGLE           40          /* Degrees */

/* Communication Protocol Definitions */
#define NXT_COMM_BAUD_RATE             921600       /* NXT communication baud rate - High-speed for STM32F765 */
#define NXT_COMM_TIMEOUT_MS            100          /* Communication timeout */
#define NXT_HEARTBEAT_INTERVAL_MS      50           /* Heartbeat interval */

/* System Status Monitoring */
#define SYSTEM_HEALTH_CHECK_INTERVAL_MS 1000        /* Health check interval */
#define BATTERY_LOW_VOLTAGE_THRESHOLD   22.0        /* 24V system low voltage */
#define BATTERY_CRITICAL_VOLTAGE        20.0        /* Critical battery voltage */

/* Sensor Configuration */
#define IMU_SAMPLE_RATE_HZ             1000         /* IMU sampling rate */
#define GPS_UPDATE_RATE_HZ             10           /* GPS update rate */
#define BARO_UPDATE_RATE_HZ            50           /* Barometer update rate */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
