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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <syslog.h>
#include <errno.h>

#include "nxt_qencoder.h"
#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO pin definitions for encoders */
#define ENCODER_FL_A_GPIO    (GPIO_PORTB | GPIO_PIN0)  /* PB0 */
#define ENCODER_FL_B_GPIO    (GPIO_PORTB | GPIO_PIN1)  /* PB1 */
#define ENCODER_FL_IDX_GPIO  (GPIO_PORTB | GPIO_PIN2)  /* PB2 - Index */

#define ENCODER_FR_A_GPIO    (GPIO_PORTB | GPIO_PIN4)  /* PB4 */
#define ENCODER_FR_B_GPIO    (GPIO_PORTB | GPIO_PIN5)  /* PB5 */
#define ENCODER_FR_IDX_GPIO  (GPIO_PORTB | GPIO_PIN3)  /* PB3 - Index */

#define ENCODER_RL_A_GPIO    (GPIO_PORTC | GPIO_PIN6)  /* PC6 */
#define ENCODER_RL_B_GPIO    (GPIO_PORTC | GPIO_PIN7)  /* PC7 */
#define ENCODER_RL_IDX_GPIO  (GPIO_PORTC | GPIO_PIN8)  /* PC8 - Index */

#define ENCODER_RR_A_GPIO    (GPIO_PORTA | GPIO_PIN0)  /* PA0 */
#define ENCODER_RR_B_GPIO    (GPIO_PORTA | GPIO_PIN1)  /* PA1 */
#define ENCODER_RR_IDX_GPIO  (GPIO_PORTA | GPIO_PIN2)  /* PA2 - Index */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_BOARD_NXT_QENCODER

/* Encoder configurations for wheel loader */
static const struct nxt_qe_config_s g_wheel_encoders[] =
{
  /* Front Left Wheel - GPIO mode */
  {
    .gpio = {
      .phase_a = ENCODER_FL_A_GPIO,
      .phase_b = ENCODER_FL_B_GPIO,
      .index = ENCODER_FL_IDX_GPIO,
    },
    .resolution = 1024,    /* 1024 CPR encoder */
    .use_index = true,
    .x4_mode = true,       /* 4x counting mode */
    .invert_dir = false,
  },

  /* Front Right Wheel - GPIO mode */
  {
    .gpio = {
      .phase_a = ENCODER_FR_A_GPIO,
      .phase_b = ENCODER_FR_B_GPIO,
      .index = ENCODER_FR_IDX_GPIO,
    },
    .resolution = 1024,
    .use_index = true,
    .x4_mode = true,
    .invert_dir = true,    /* Right side inverted */
  },

  /* Rear Left Wheel - GPIO mode */
  {
    .gpio = {
      .phase_a = ENCODER_RL_A_GPIO,
      .phase_b = ENCODER_RL_B_GPIO,
      .index = ENCODER_RL_IDX_GPIO,
    },
    .resolution = 1024,
    .use_index = true,
    .x4_mode = true,
    .invert_dir = false,
  },

  /* Rear Right Wheel - GPIO mode */
  {
    .gpio = {
      .phase_a = ENCODER_RR_A_GPIO,
      .phase_b = ENCODER_RR_B_GPIO,
      .index = ENCODER_RR_IDX_GPIO,
    },
    .resolution = 1024,
    .use_index = true,
    .x4_mode = true,
    .invert_dir = true,    /* Right side inverted */
  },

  /* Additional encoder for auxiliary system - GPIO mode */
  {
    .gpio = {
      .phase_a = GPIO_PORTD | GPIO_PIN12,  /* PD12 */
      .phase_b = GPIO_PORTD | GPIO_PIN13,  /* PD13 */
      .index = GPIO_PORTD | GPIO_PIN14,    /* PD14 */
    },
    .resolution = 512,     /* 512 CPR encoder */
    .use_index = true,
    .x4_mode = true,
    .invert_dir = false,
  },
};

#define NUM_ENCODERS (sizeof(g_wheel_encoders) / sizeof(g_wheel_encoders[0]))

#endif /* CONFIG_BOARD_NXT_QENCODER */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_qencoder_gpio_config
 *
 * Description:
 *   Configure GPIO pins for index signals
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_NXT_QENCODER
static void board_qencoder_gpio_config(void)
{
  /* Configure index pins as GPIO inputs for encoders */
  for (int i = 0; i < NUM_ENCODERS; i++)
    {
      if (g_wheel_encoders[i].use_index &&
          g_wheel_encoders[i].gpio.index != 0)
        {
          /* Configure index pin as input with pull-up */
          stm32_configgpio(g_wheel_encoders[i].gpio.index |
                          GPIO_INPUT | GPIO_PULLUP);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_qencoder_initialize
 *
 * Description:
 *   Initialize all quadrature encoders for the board
 *
 ****************************************************************************/

int board_qencoder_initialize(void)
{
#ifdef CONFIG_BOARD_NXT_QENCODER
  int ret;
  char devpath[16];

  syslog(LOG_INFO, "Initializing %d quadrature encoders\n", NUM_ENCODERS);

  /* Configure GPIO pins */
  board_qencoder_gpio_config();

  /* Initialize each encoder */
  for (int i = 0; i < NUM_ENCODERS; i++)
    {
      snprintf(devpath, sizeof(devpath), "/dev/qe%d", i);

      ret = nxt_qencoder_initialize(&g_wheel_encoders[i], devpath);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize encoder %s: %d\n",
                 devpath, ret);
          return ret;
        }

      syslog(LOG_INFO, "Initialized encoder %s (mode=%s, resolution=%d)\n",
             devpath,
             "GPIO",
             g_wheel_encoders[i].resolution);
    }

  syslog(LOG_INFO, "All encoders initialized successfully\n");
  return OK;

#else
  syslog(LOG_INFO, "Quadrature encoder support not enabled\n");
  return OK;
#endif
}

/****************************************************************************
 * Name: board_qencoder_get_config
 *
 * Description:
 *   Get encoder configuration for a specific encoder
 *
 ****************************************************************************/

int board_qencoder_get_config(int encoder_id,
                             FAR struct nxt_qe_config_s *config)
{
#ifdef CONFIG_BOARD_NXT_QENCODER
  if (encoder_id < 0 || encoder_id >= NUM_ENCODERS || !config)
    {
      return -EINVAL;
    }

  memcpy(config, &g_wheel_encoders[encoder_id], sizeof(struct nxt_qe_config_s));
  return OK;
#else
  return -ENOTSUP;
#endif
}

/****************************************************************************
 * Name: board_qencoder_get_count
 *
 * Description:
 *   Get the number of available encoders
 *
 ****************************************************************************/

int board_qencoder_get_count(void)
{
#ifdef CONFIG_BOARD_NXT_QENCODER
  return NUM_ENCODERS;
#else
  return 0;
#endif
}
