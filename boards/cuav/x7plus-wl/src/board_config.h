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
 * @file board_config.h
 *
 * CUAV X7+ WL (Wheel Loader) Controller Board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* LEDs */
#define GPIO_nLED_RED        /* PI5 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN5)
#define GPIO_nLED_GREEN      /* PI6 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN6)
#define GPIO_nLED_BLUE       /* PI7 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN7)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

/* Safety Switch */
#define GPIO_BTN_SAFETY      /* PI8 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTI|GPIO_PIN8)
#define GPIO_SAFETY_SWITCH_IN              GPIO_BTN_SAFETY
#define GPIO_LED_SAFETY        GPIO_nLED_RED

/* Power */
#define GPIO_VDD_BRICK_VALID   /* PG1  */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN1)
#define GPIO_VDD_BRICK2_VALID  /* PG2  */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2)
#define GPIO_VDD_USB_VALID     /* PC0  */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN0)

#define GPIO_VDD_5V_PERIPH_EN  /* PA8  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN8)
#define GPIO_VDD_5V_HIPOWER_EN /* PE4  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_VDD_3V3_SENSORS_EN /* PE3 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN /* PE5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)

/* Tone alarm output */
#define TONE_ALARM_TIMER        8  /* timer 8 */
#define TONE_ALARM_CHANNEL      1  /* PI0 TIM8_CH1 */
#define GPIO_BUZZER_1           /* PI0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN0)

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define DIRECT_INPUT_TIMER_CHANNELS 8

/* USB OTG FS */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               3  /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL       4  /* use capture/compare channel 4 */

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER             4
#define PWMIN_TIMER_CHANNEL     2
#define GPIO_PWM_IN             /* PD13 */ (GPIO_ALT|GPIO_AF2|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)

/* GPIO pins used for SPI chip selects */
#define GPIO_SPI_CS_GYRO1       /* PF10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN10)
#define GPIO_SPI_CS_GYRO2       /* PC15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
#define GPIO_SPI_CS_ACCEL1      /* PC14 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN14)
#define GPIO_SPI_CS_BARO1       /* PD7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)
#define GPIO_SPI_CS_BARO2       /* PE15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN15)

/* ADC Configuration */
#define PX4_ADC_GPIO                    /* PA4 */ (GPIO_INPUT|GPIO_ANALOG|GPIO_PORTA|GPIO_PIN4)

/* Hardware version detection */
#define GPIO_HW_REV_DRIVE               /* PG14 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN14)
#define GPIO_HW_VER_DRIVE               /* PG13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN13)

/* CAN Bus Configuration */
#define GPIO_CAN1_TX                    /* PD1  */ (GPIO_ALT|GPIO_AF9|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN1)
#define GPIO_CAN1_RX                    /* PD0  */ (GPIO_ALT|GPIO_AF9|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN0)
#define GPIO_CAN2_TX                    /* PB6  */ (GPIO_ALT|GPIO_AF9|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN6)
#define GPIO_CAN2_RX                    /* PB12 */ (GPIO_ALT|GPIO_AF9|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN12)
#define GPIO_CAN3_TX                    /* PD10 */ (GPIO_ALT|GPIO_AF9|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN10)
#define GPIO_CAN3_RX                    /* PD11 */ (GPIO_ALT|GPIO_AF9|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN11)

/* Heater for IMU */
#define GPIO_HEATER_OUTPUT              /* PB10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)

/* Use these in place of the uint32_t enumeration to allow switch statements
 * with chip select lines
 */
#define PX4_SPIDEV_GYRO1        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS1, 1)
#define PX4_SPIDEV_GYRO2        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS1, 2)
#define PX4_SPIDEV_ACCEL1       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS1, 3)
#define PX4_SPIDEV_BARO1        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS1, 4)
#define PX4_SPIDEV_BARO2        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS1, 5)

/* I2C busses */
#define PX4_I2C_BUS_EXPANSION   1
#define PX4_I2C_BUS_ONBOARD     2
#define PX4_I2C_BUS_GPS         3

/* Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_LED       0x55
#define PX4_I2C_OBDEV_HMC5883   0x1e

/* SPI bus mappings */
#define PX4_SPI_BUS_SENSORS1    1
#define PX4_SPI_BUS_SENSORS2    2
#define PX4_SPI_BUS_BARO        PX4_SPI_BUS_SENSORS1

/* UART mappings for wheel loader specific functions
 * GPS1 port (/dev/ttyS0) -> RTK GPS
 * TELEM1 port (/dev/ttyS1) -> MAVLink ground station
 * TELEM2 port (/dev/ttyS2) -> NXT Controller 1
 * UART4 port (/dev/ttyS3) -> NXT Controller 2
 */
#define PX4_UART_GPS1           "/dev/ttyS0"  /* RTK GPS */
#define PX4_UART_TEL1           "/dev/ttyS1"  /* MAVLink ground station */
#define PX4_UART_TEL2           "/dev/ttyS2"  /* NXT Controller 1 */
#define PX4_UART_TEL4           "/dev/ttyS3"  /* NXT Controller 2 */

/* Data ready pins */
#define GPIO_GYRO1_DRDY         /* PB4  */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN4)
#define GPIO_GYRO2_DRDY         /* PB5  */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN5)
#define GPIO_ACCEL1_DRDY        /* PB15 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN15)

/* Timer allocation */
#define TIM1_CH1OUT     /* PA8  TIM1_CH1 */    (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN8)
#define TIM1_CH2OUT     /* PA9  TIM1_CH2 */    (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN9)
#define TIM1_CH3OUT     /* PA10 TIM1_CH3 */    (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN10)
#define TIM1_CH4OUT     /* PA11 TIM1_CH4 */    (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN11)

#define TIM4_CH1OUT     /* PD12 TIM4_CH1 */    (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN12)
#define TIM4_CH2OUT     /* PD13 TIM4_CH2 */    (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN13)
#define TIM4_CH3OUT     /* PD14 TIM4_CH3 */    (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN14)
#define TIM4_CH4OUT     /* PD15 TIM4_CH4 */    (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN15)

/* User GPIOs */
#define GPIO_GPIO0_INPUT        /* PE14 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO1_INPUT        /* PA0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN0)
#define GPIO_GPIO2_INPUT        /* PE11 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO3_INPUT        /* PE9  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO4_INPUT        /* PH6  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTH|GPIO_PIN6)
#define GPIO_GPIO5_INPUT        /* PH5  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTH|GPIO_PIN5)

#define GPIO_GPIO0_OUTPUT       /* PE14 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO1_OUTPUT       /* PA0  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#define GPIO_GPIO2_OUTPUT       /* PE11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO3_OUTPUT       /* PE9  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO4_OUTPUT       /* PH6  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN6)
#define GPIO_GPIO5_OUTPUT       /* PH5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN5)

/* Power supply control and monitoring GPIOs */
#define GPIO_nPOWER_IN_A                /* PG1  */ GPIO_VDD_BRICK_VALID
#define GPIO_nPOWER_IN_B                /* PG2  */ GPIO_VDD_BRICK2_VALID
#define GPIO_nPOWER_IN_C                /* PC0  */ GPIO_VDD_USB_VALID

#define GPIO_nVDD_BRICK1_VALID          GPIO_VDD_BRICK_VALID
#define GPIO_nVDD_BRICK2_VALID          GPIO_VDD_BRICK2_VALID
#define GPIO_nVDD_USB_VALID             GPIO_VDD_USB_VALID

#define GPIO_VDD_5V_PERIPH_EN           GPIO_VDD_5V_PERIPH_EN
#define GPIO_VDD_5V_HIPOWER_EN          GPIO_VDD_5V_HIPOWER_EN
#define GPIO_VDD_3V3_SENSORS_EN         GPIO_VDD_3V3_SENSORS_EN

/* Define Battery 1 Voltage Divider and A per V */
#define BOARD_BATTERY1_V_DIV            (18.0f)
#define BOARD_BATTERY1_A_PER_V          (24.0f)

/* Define Battery 2 Voltage Divider and A per V */
#define BOARD_BATTERY2_V_DIV            (18.0f)
#define BOARD_BATTERY2_A_PER_V          (24.0f)

/* Tone alarm output */
#define TONE_ALARM_TIMER        8  /* timer 8 */
#define TONE_ALARM_CHANNEL      1  /* PI0 TIM8_CH1 */

#define GPIO_BUZZER_1           /* PI0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN0)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_BUZZER_1

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define DIRECT_INPUT_TIMER_CHANNELS  8

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_REV_DRIVE,                \
		GPIO_HW_VER_DRIVE,                \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_CAN3_TX,                     \
		GPIO_CAN3_RX,                     \
		GPIO_HEATER_OUTPUT,               \
		GPIO_nPOWER_IN_A,                 \
		GPIO_nPOWER_IN_B,                 \
		GPIO_nPOWER_IN_C,                 \
		GPIO_VDD_5V_PERIPH_EN,            \
		GPIO_VDD_5V_HIPOWER_EN,           \
		GPIO_VDD_3V3_SENSORS_EN,          \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_nLED_RED,                    \
		GPIO_nLED_GREEN,                  \
		GPIO_nLED_BLUE,                   \
		GPIO_SAFETY_SWITCH_IN,            \
		GPIO_BTN_SAFETY,                  \
	}

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

#ifdef __cplusplus
}
#endif
