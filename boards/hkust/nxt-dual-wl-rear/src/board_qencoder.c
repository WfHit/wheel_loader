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
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <errno.h>

#include "stm32_gpio.h"
#include "board_config.h"
#include "../../../../../platforms/nuttx/src/px4/stm/stm32_common/include/px4_arch/nuttx_qencoder.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_BOARD_NXT_QENCODER

/* Encoder configurations for nxt-dual-wl-rear motor encoder */
static const struct nuttx_qe_config_s g_motor_encoders[] =
{
  /* Motor Encoder - GPIO mode */
  {
    .gpio = {
      .phase_a = QENCODER_A_GPIO_RAW,
      .phase_b = QENCODER_B_GPIO_RAW,
      .index = 0,  /* No index signal */
    },
    .resolution = 1024,    /* 1024 CPR encoder */
    .use_index = false,    /* No index signal for motor encoder */
    .x4_mode = true,       /* 4x counting mode for higher resolution */
    .invert_dir = false,
  },
};

#define NUM_ENCODERS (sizeof(g_motor_encoders) / sizeof(g_motor_encoders[0]))

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
  for (unsigned int i = 0; i < NUM_ENCODERS; i++)
    {
      if (g_motor_encoders[i].use_index &&
          g_motor_encoders[i].gpio.index != 0)
        {
          /* Configure index pin as input with pull-up */
          stm32_configgpio(g_motor_encoders[i].gpio.index |
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
  for (unsigned int i = 0; i < NUM_ENCODERS; i++)
    {
      snprintf(devpath, sizeof(devpath), "/dev/qe%d", i);

      ret = nuttx_qencoder_initialize(&g_motor_encoders[i], devpath);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize encoder %s: %d\n",
                 devpath, ret);
          return ret;
        }

      syslog(LOG_INFO, "Initialized encoder %s (mode=%s, resolution=%lu)\n",
             devpath,
             "GPIO",
             (unsigned long)g_motor_encoders[i].resolution);
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
                             FAR struct nuttx_qe_config_s *config)
{
#ifdef CONFIG_BOARD_NXT_QENCODER
  if (encoder_id < 0 || encoder_id >= (int)NUM_ENCODERS || !config)
    {
      return -EINVAL;
    }

  memcpy(config, &g_motor_encoders[encoder_id], sizeof(struct nuttx_qe_config_s));
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
