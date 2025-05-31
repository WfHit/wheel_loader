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
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/qencoder.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include <sys/types.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <math.h>

#include "nxt_qencoder.h"
#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QE_DEBUG 1

#if QE_DEBUG
#  define qeinfo(format, ...)   syslog(LOG_INFO, format, ##__VA_ARGS__)
#  define qeerr(format, ...)    syslog(LOG_ERR, format, ##__VA_ARGS__)
#else
#  define qeinfo(format, ...)
#  define qeerr(format, ...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Lower half driver state structure */
struct nxt_qe_lowerhalf_s
{
  /* Must be first - standard lower half driver interface */
  struct qe_lowerhalf_s lower;

  /* Configuration */
  struct nxt_qe_config_s config;

  /* Runtime state */
  volatile int32_t position;
  volatile uint32_t index_count;
  volatile int32_t last_index_position;
  uint32_t error_count;
  uint32_t last_velocity_time;
  int32_t last_velocity_position;
  float velocity;

  /* GPIO mode specific */
  struct {
    uint8_t last_state;
    volatile bool irq_pending;
  } gpio_state;

  /* Status */
  struct qe_status_s status;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Lower half operations */
static int qe_setup(FAR struct qe_lowerhalf_s *lower);
static int qe_shutdown(FAR struct qe_lowerhalf_s *lower);
static int qe_position(FAR struct qe_lowerhalf_s *lower, FAR int32_t *pos);
static int qe_reset(FAR struct qe_lowerhalf_s *lower);
static int qe_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd, unsigned long arg);

/* GPIO mode functions */
static int qe_setup_gpio(FAR struct nxt_qe_lowerhalf_s *priv);
static int qe_gpio_isr(int irq, FAR void *context, FAR void *arg);
static int qe_index_isr(int irq, FAR void *context, FAR void *arg);

/* Velocity calculation */
static void qe_calculate_velocity(FAR struct nxt_qe_lowerhalf_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half operations */
static const struct qe_ops_s g_qe_ops =
{
  .setup    = qe_setup,
  .shutdown = qe_shutdown,
  .position = qe_position,
  .reset    = qe_reset,
  .ioctl    = qe_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qe_gpio_isr
 *
 * Description:
 *   GPIO interrupt handler for quadrature signals
 *
 ****************************************************************************/

static int qe_gpio_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct nxt_qe_lowerhalf_s *priv = (FAR struct nxt_qe_lowerhalf_s *)arg;
  uint8_t state_a, state_b;
  uint8_t current_state;
  uint8_t last_state;

  /* Read current pin states */
  state_a = stm32_gpioread(priv->config.gpio.phase_a) ? 1 : 0;
  state_b = stm32_gpioread(priv->config.gpio.phase_b) ? 1 : 0;
  current_state = (state_a << 1) | state_b;

  last_state = priv->gpio_state.last_state;

  /* Quadrature state machine for x4 mode */
  if (priv->config.x4_mode)
    {
      /* Gray code state transitions */
      static const int8_t state_table[16] = {
         0, -1,  1,  2,  /* 2 = error */
         1,  0,  2, -1,
        -1,  2,  0,  1,
         2,  1, -1,  0
      };

      uint8_t transition = (last_state << 2) | current_state;
      int8_t delta = state_table[transition];

      if (delta == 2)
        {
          /* Invalid transition - quadrature error */
          priv->error_count++;
          priv->status.error_count++;
        }
      else if (delta != 0)
        {
          priv->position += priv->config.invert_dir ? -delta : delta;
        }
    }
  else
    {
      /* x1 mode - count on A rising edge only */
      if ((last_state & 0x02) == 0 && (current_state & 0x02) != 0)
        {
          if (current_state & 0x01)
            priv->position += priv->config.invert_dir ? -1 : 1;
          else
            priv->position += priv->config.invert_dir ? 1 : -1;
        }
    }

  priv->gpio_state.last_state = current_state;
  return OK;
}

/****************************************************************************
 * Name: qe_index_isr
 *
 * Description:
 *   Index pulse interrupt handler
 *
 ****************************************************************************/

static int qe_index_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct nxt_qe_lowerhalf_s *priv = (FAR struct nxt_qe_lowerhalf_s *)arg;

  /* Record index pulse */
  priv->index_count++;
  priv->status.index_count++;
  priv->last_index_position = priv->position;
  priv->status.last_index_pos = priv->position;
  priv->status.index_found = true;

  qeinfo("Index pulse detected at position %d\n", (int)priv->position);

  return OK;
}

/****************************************************************************
 * Name: qe_setup_gpio
 *
 * Description:
 *   Setup GPIO mode encoder
 *
 ****************************************************************************/

static int qe_setup_gpio(FAR struct nxt_qe_lowerhalf_s *priv)
{
  int ret;

  qeinfo("Setting up GPIO mode encoder\n");

  /* Configure GPIO pins as inputs with interrupts */
  stm32_configgpio(priv->config.gpio.phase_a | GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI);
  stm32_configgpio(priv->config.gpio.phase_b | GPIO_INPUT | GPIO_FLOAT);

  if (priv->config.use_index && priv->config.gpio.index)
    {
      stm32_configgpio(priv->config.gpio.index | GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI);
    }

  /* Attach interrupt handler for phase A */
  ret = stm32_gpiosetevent(priv->config.gpio.phase_a, true, true, true,
                          qe_gpio_isr, priv);
  if (ret < 0)
    {
      qeerr("ERROR: Failed to set GPIO event for phase A\n");
      return ret;
    }

  /* Attach interrupt handler for index if enabled */
  if (priv->config.use_index && priv->config.gpio.index)
    {
      ret = stm32_gpiosetevent(priv->config.gpio.index, true, false, false,
                             qe_index_isr, priv);
      if (ret < 0)
        {
          qeerr("ERROR: Failed to set GPIO event for index\n");
          return ret;
        }
    }

  /* Read initial state */
  priv->gpio_state.last_state =
    (stm32_gpioread(priv->config.gpio.phase_a) ? 2 : 0) |
    (stm32_gpioread(priv->config.gpio.phase_b) ? 1 : 0);

  qeinfo("GPIO encoder setup complete, initial state: %d\n",
         priv->gpio_state.last_state);

  return OK;
}

/****************************************************************************
 * Name: qe_calculate_velocity
 *
 * Description:
 *   Calculate encoder velocity
 *
 ****************************************************************************/

static void qe_calculate_velocity(FAR struct nxt_qe_lowerhalf_s *priv)
{
  uint32_t current_time = clock_systime_ticks();
  int32_t current_position = priv->position;
  float dt;

  if (priv->last_velocity_time != 0)
    {
      dt = (current_time - priv->last_velocity_time) * USEC_PER_TICK / 1000000.0f;

      if (dt > 0)
        {
          /* Velocity in counts per second */
          priv->velocity = (current_position - priv->last_velocity_position) / dt;

          /* Convert to RPM if resolution is known */
          if (priv->config.resolution > 0)
            {
              priv->velocity = (priv->velocity * 60.0f) / priv->config.resolution;
            }
        }
    }

  priv->last_velocity_time = current_time;
  priv->last_velocity_position = current_position;
}

/****************************************************************************
 * Name: qe_setup
 *
 * Description:
 *   Setup the encoder device
 *
 ****************************************************************************/

static int qe_setup(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct nxt_qe_lowerhalf_s *priv = (FAR struct nxt_qe_lowerhalf_s *)lower;
  int ret;

  qeinfo("Setup GPIO encoder\n");

  ret = qe_setup_gpio(priv);

  return ret;
}

/****************************************************************************
 * Name: qe_shutdown
 *
 * Description:
 *   Shutdown the encoder device
 *
 ****************************************************************************/

static int qe_shutdown(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct nxt_qe_lowerhalf_s *priv = (FAR struct nxt_qe_lowerhalf_s *)lower;

  qeinfo("Shutdown encoder\n");

  /* Disable GPIO interrupts */
  stm32_gpiosetevent(priv->config.gpio.phase_a, false, false, false, NULL, NULL);
  if (priv->config.use_index && priv->config.gpio.index)
    {
      stm32_gpiosetevent(priv->config.gpio.index, false, false, false, NULL, NULL);
    }

  return OK;
}

/****************************************************************************
 * Name: qe_position
 *
 * Description:
 *   Get current encoder position
 *
 ****************************************************************************/

static int qe_position(FAR struct qe_lowerhalf_s *lower, FAR int32_t *pos)
{
  FAR struct nxt_qe_lowerhalf_s *priv = (FAR struct nxt_qe_lowerhalf_s *)lower;

  if (!pos)
    {
      return -EINVAL;
    }

  *pos = priv->position;

  /* Update velocity calculation */
  qe_calculate_velocity(priv);

  return OK;
}

/****************************************************************************
 * Name: qe_reset
 *
 * Description:
 *   Reset encoder position to zero
 *
 ****************************************************************************/

static int qe_reset(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct nxt_qe_lowerhalf_s *priv = (FAR struct nxt_qe_lowerhalf_s *)lower;
  irqstate_t flags;

  qeinfo("Reset encoder position\n");

  flags = enter_critical_section();

  priv->position = 0;
  priv->index_count = 0;

  /* Reset velocity tracking */
  priv->last_velocity_time = 0;
  priv->last_velocity_position = 0;
  priv->velocity = 0;

  /* Reset status */
  priv->status.error_count = 0;
  priv->status.index_count = 0;
  priv->status.index_found = false;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: qe_ioctl
 *
 * Description:
 *   Handle IOCTL commands
 *
 ****************************************************************************/

static int qe_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd, unsigned long arg)
{
  FAR struct nxt_qe_lowerhalf_s *priv = (FAR struct nxt_qe_lowerhalf_s *)lower;
  int ret = OK;

  switch (cmd)
    {
    case QEIOC_VELOCITY:
      {
        FAR float *velocity = (FAR float *)arg;
        if (velocity)
          {
            *velocity = priv->velocity;
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    case QEIOC_INDEX_COUNT:
      {
        FAR uint32_t *count = (FAR uint32_t *)arg;
        if (count)
          {
            *count = priv->index_count;
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    case QEIOC_GET_STATUS:
      {
        FAR struct qe_status_s *status = (FAR struct qe_status_s *)arg;
        if (status)
          {
            memcpy(status, &priv->status, sizeof(struct qe_status_s));
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    case QEIOC_CALIBRATE:
      {
        /* Perform encoder calibration using index pulse */
        if (priv->config.use_index && priv->status.index_found)
          {
            qe_reset(lower);
            qeinfo("Encoder calibrated using index pulse\n");
          }
        else
          {
            ret = -ENOTSUP;
          }
      }
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxt_qencoder_initialize
 *
 * Description:
 *   Initialize quadrature encoder and register as a character device.
 *
 ****************************************************************************/

int nxt_qencoder_initialize(FAR const struct nxt_qe_config_s *config,
                           FAR const char *devpath)
{
  FAR struct nxt_qe_lowerhalf_s *priv;
  int ret;

  qeinfo("Initializing encoder at %s\n", devpath);

  /* Allocate the lower half structure */
  priv = (FAR struct nxt_qe_lowerhalf_s *)kmm_zalloc(sizeof(struct nxt_qe_lowerhalf_s));
  if (!priv)
    {
      qeerr("ERROR: Failed to allocate encoder structure\n");
      return -ENOMEM;
    }

  /* Initialize the lower half structure */
  priv->lower.ops = &g_qe_ops;
  memcpy(&priv->config, config, sizeof(struct nxt_qe_config_s));

  /* Initialize state */
  priv->position = 0;
  priv->index_count = 0;
  priv->error_count = 0;
  priv->velocity = 0;

  /* Register the encoder device */
  ret = qe_register(devpath, (FAR struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      qeerr("ERROR: Failed to register encoder device: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  qeinfo("Encoder initialized successfully at %s\n", devpath);
  return OK;
}
