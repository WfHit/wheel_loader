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
#include <px4_platform_common/px4_config.h>
#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/qencoder.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>

#include <sys/types.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <math.h>

#include <px4_arch/nuttx_qencoder.h>
#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QE_DEBUG 1
#define CONFIG_NUTTX_QENCODER_MAX_INSTANCES 8

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
struct nuttx_qe_lowerhalf_s
{
  /* Must be first - standard lower half driver interface */
  struct qe_lowerhalf_s lower;

  /* Configuration */
  struct nuttx_qe_config_s config;

  /* Instance identification */
  uint8_t instance_id;
  char devpath[16];

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

/* Global instance management structure */
struct nuttx_qe_instances_s
{
  /* Semaphore for thread-safe access */
  sem_t lock;

  /* Instance tracking */
  uint8_t num_instances;
  struct nuttx_qe_lowerhalf_s *instances[CONFIG_NUTTX_QENCODER_MAX_INSTANCES];

  /* Device path tracking */
  char devpaths[CONFIG_NUTTX_QENCODER_MAX_INSTANCES][16];

  /* Instance status */
  bool active[CONFIG_NUTTX_QENCODER_MAX_INSTANCES];
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
static int qe_setup_gpio(FAR struct nuttx_qe_lowerhalf_s *priv);
static int qe_gpio_isr(int irq, FAR void *context, FAR void *arg);
static int qe_index_isr(int irq, FAR void *context, FAR void *arg);

/* Velocity calculation */
static void qe_calculate_velocity(FAR struct nuttx_qe_lowerhalf_s *priv);

/* Instance management functions */
static int qe_alloc_instance(void);
static int qe_free_instance(uint8_t instance_id);
static FAR struct nuttx_qe_lowerhalf_s *qe_find_instance(uint8_t instance_id);
static int qe_init_instance_manager(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Global instance manager */
static struct nuttx_qe_instances_s g_qe_instances;
static bool g_instance_manager_initialized = false;

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
  FAR struct nuttx_qe_lowerhalf_s *priv = (FAR struct nuttx_qe_lowerhalf_s *)arg;
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
  FAR struct nuttx_qe_lowerhalf_s *priv = (FAR struct nuttx_qe_lowerhalf_s *)arg;

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

static int qe_setup_gpio(FAR struct nuttx_qe_lowerhalf_s *priv)
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

static void qe_calculate_velocity(FAR struct nuttx_qe_lowerhalf_s *priv)
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
  FAR struct nuttx_qe_lowerhalf_s *priv = (FAR struct nuttx_qe_lowerhalf_s *)lower;
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
  FAR struct nuttx_qe_lowerhalf_s *priv = (FAR struct nuttx_qe_lowerhalf_s *)lower;

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
  FAR struct nuttx_qe_lowerhalf_s *priv = (FAR struct nuttx_qe_lowerhalf_s *)lower;

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
  FAR struct nuttx_qe_lowerhalf_s *priv = (FAR struct nuttx_qe_lowerhalf_s *)lower;
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
  FAR struct nuttx_qe_lowerhalf_s *priv = (FAR struct nuttx_qe_lowerhalf_s *)lower;
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
 * Name: nuttx_qencoder_initialize
 *
 * Description:
 *   Initialize quadrature encoder and register as a character device.
 *
 ****************************************************************************/

int nuttx_qencoder_initialize(FAR const struct nuttx_qe_config_s *config,
                             FAR const char *devpath)
{
  FAR struct nuttx_qe_lowerhalf_s *priv;
  int ret;

  qeinfo("Initializing encoder at %s\n", devpath);

  /* Allocate the lower half structure */
  priv = (FAR struct nuttx_qe_lowerhalf_s *)kmm_zalloc(sizeof(struct nuttx_qe_lowerhalf_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  /* Initialize instance fields */
  priv->instance_id = CONFIG_NUTTX_QENCODER_MAX_INSTANCES;
  strncpy(priv->devpath, devpath, sizeof(priv->devpath) - 1);
  priv->devpath[sizeof(priv->devpath) - 1] = '\0';

  /* Copy configuration */
  memcpy(&priv->config, config, sizeof(struct nuttx_qe_config_s));

  /* Setup lower half interface */
  priv->lower.ops = &g_qe_ops;

  /* Allocate instance */
  ret = qe_alloc_instance();
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  priv->instance_id = ret;
  g_qe_instances.instances[priv->instance_id] = priv;

  /* Register character device */
  ret = qe_register(priv->devpath, &priv->lower);
  if (ret < 0)
    {
      qe_free_instance(priv->instance_id);
      kmm_free(priv);
      return ret;
    }

  /* Store device path in instance manager */
  ret = nxsem_wait(&g_qe_instances.lock);
  if (ret >= 0)
    {
      strncpy(g_qe_instances.devpaths[priv->instance_id], priv->devpath,
              sizeof(g_qe_instances.devpaths[priv->instance_id]) - 1);
      g_qe_instances.devpaths[priv->instance_id][sizeof(g_qe_instances.devpaths[priv->instance_id]) - 1] = '\0';
      nxsem_post(&g_qe_instances.lock);
    }

  qeinfo("Encoder initialized successfully (instance ID: %d)\n", priv->instance_id);

  return OK;
}

/****************************************************************************
 * Name: nuttx_qencoder_uninitialize
 *
 * Description:
 *   Uninitialize a specific quadrature encoder instance
 *
 * Input Parameters:
 *   instance_id - Instance ID to uninitialize
 *
 * Returned Value:
 *   Zero (OK) on success; negative errno on failure
 *
 ****************************************************************************/

int nuttx_qencoder_uninitialize(uint8_t instance_id)
{
  FAR struct nuttx_qe_lowerhalf_s *priv;
  int ret;

  qeinfo("Uninitializing encoder instance %d\n", instance_id);

  /* Find instance */
  priv = qe_find_instance(instance_id);
  if (!priv)
    {
      qeerr("ERROR: Instance %d not found\n", instance_id);
      return -ENODEV;
    }

  /* Shutdown encoder operations */
  qe_shutdown((FAR struct qe_lowerhalf_s *)priv);

  /* Unregister character device */
  ret = unregister_driver(priv->devpath);
  if (ret < 0)
    {
      qeerr("ERROR: Failed to unregister encoder device: %d\n", ret);
      return ret;
    }

  /* Free instance */
  ret = qe_free_instance(instance_id);
  if (ret < 0)
    {
      qeerr("ERROR: Failed to free instance: %d\n", ret);
      return ret;
    }

  /* Free memory */
  kmm_free(priv);

  qeinfo("Encoder instance %d uninitialized successfully\n", instance_id);
  return OK;
}

/****************************************************************************
 * Name: nuttx_qencoder_uninitialize_all
 *
 * Description:
 *   Uninitialize all active quadrature encoder instances
 *
 * Returned Value:
 *   Zero (OK) on success; negative errno on failure
 *
 ****************************************************************************/

int nuttx_qencoder_uninitialize_all(void)
{
  int ret = OK;
  int total_errors = 0;

  qeinfo("Uninitializing all encoder instances\n");

  if (!g_instance_manager_initialized)
    {
      return OK; /* Nothing to cleanup */
    }

  /* Lock for thread safety */
  ret = nxsem_wait(&g_qe_instances.lock);
  if (ret < 0)
    {
      qeerr("ERROR: Failed to acquire instance lock: %d\n", ret);
      return ret;
    }

  /* Uninitialize all active instances */
  for (int i = 0; i < CONFIG_NUTTX_QENCODER_MAX_INSTANCES; i++)
    {
      if (g_qe_instances.active[i] && g_qe_instances.instances[i])
        {
          /* Unlock before calling uninitialize (it will lock again) */
          nxsem_post(&g_qe_instances.lock);

          ret = nuttx_qencoder_uninitialize(i);
          if (ret < 0)
            {
              qeerr("ERROR: Failed to uninitialize instance %d: %d\n", i, ret);
              total_errors++;
            }

          /* Relock for next iteration */
          ret = nxsem_wait(&g_qe_instances.lock);
          if (ret < 0)
            {
              qeerr("ERROR: Failed to reacquire instance lock: %d\n", ret);
              return ret;
            }
        }
    }

  nxsem_post(&g_qe_instances.lock);

  /* Destroy semaphore */
  nxsem_destroy(&g_qe_instances.lock);
  g_instance_manager_initialized = false;

  qeinfo("All encoder instances uninitialized (errors: %d)\n", total_errors);
  return (total_errors > 0) ? -EIO : OK;
}

/****************************************************************************
 * Name: nuttx_qencoder_get_instance_count
 *
 * Description:
 *   Get number of active encoder instances
 *
 * Returned Value:
 *   Number of active instances; negative errno on failure
 *
 ****************************************************************************/

int nuttx_qencoder_get_instance_count(void)
{
  int ret;
  uint8_t count;

  if (!g_instance_manager_initialized)
    {
      return 0;
    }

  ret = nxsem_wait(&g_qe_instances.lock);
  if (ret < 0)
    {
      return ret;
    }

  count = g_qe_instances.num_instances;
  nxsem_post(&g_qe_instances.lock);

  return count;
}

/****************************************************************************
 * Name: nuttx_qencoder_list_instances
 *
 * Description:
 *   List all active encoder instances
 *
 * Input Parameters:
 *   instances - Array to store instance information
 *   max_count - Maximum number of instances to return
 *
 * Returned Value:
 *   Number of instances returned; negative errno on failure
 *
 ****************************************************************************/

int nuttx_qencoder_list_instances(FAR struct nuttx_qe_instance_info_s *instances,
                                 uint8_t max_count)
{
  int ret;
  uint8_t count = 0;

  if (!instances || max_count == 0)
    {
      return -EINVAL;
    }

  if (!g_instance_manager_initialized)
    {
      return 0;
    }

  ret = nxsem_wait(&g_qe_instances.lock);
  if (ret < 0)
    {
      return ret;
    }

  for (int i = 0; i < CONFIG_NUTTX_QENCODER_MAX_INSTANCES && count < max_count; i++)
    {
      if (g_qe_instances.active[i] && g_qe_instances.instances[i])
        {
          instances[count].instance_id = i;
          instances[count].active = true;
          strncpy(instances[count].devpath, g_qe_instances.devpaths[i],
                  sizeof(instances[count].devpath) - 1);
          instances[count].devpath[sizeof(instances[count].devpath) - 1] = '\0';
          memcpy(&instances[count].config, &g_qe_instances.instances[i]->config,
                 sizeof(struct nuttx_qe_config_s));
          count++;
        }
    }

  nxsem_post(&g_qe_instances.lock);

  return count;
}

/****************************************************************************
 * Name: qe_init_instance_manager
 *
 * Description:
 *   Initialize the global instance manager
 *
 ****************************************************************************/

static int qe_init_instance_manager(void)
{
  int ret;

  if (g_instance_manager_initialized)
    {
      return OK;
    }

  /* Initialize semaphore */
  ret = nxsem_init(&g_qe_instances.lock, 0, 1);
  if (ret < 0)
    {
      qeerr("ERROR: Failed to initialize instance lock: %d\n", ret);
      return ret;
    }

  /* Initialize instance tracking */
  g_qe_instances.num_instances = 0;
  memset(g_qe_instances.instances, 0, sizeof(g_qe_instances.instances));
  memset(g_qe_instances.devpaths, 0, sizeof(g_qe_instances.devpaths));
  memset(g_qe_instances.active, 0, sizeof(g_qe_instances.active));

  g_instance_manager_initialized = true;
  qeinfo("Instance manager initialized\n");

  return OK;
}

/****************************************************************************
 * Name: qe_alloc_instance
 *
 * Description:
 *   Allocate a new encoder instance
 *
 * Returned Value:
 *   Instance ID on success; negative errno on failure
 *
 ****************************************************************************/

static int qe_alloc_instance(void)
{
  int ret;
  int instance_id = -1;

  /* Initialize instance manager if needed */
  ret = qe_init_instance_manager();
  if (ret < 0)
    {
      return ret;
    }

  /* Lock for thread safety */
  ret = nxsem_wait(&g_qe_instances.lock);
  if (ret < 0)
    {
      qeerr("ERROR: Failed to acquire instance lock: %d\n", ret);
      return ret;
    }

  /* Find available instance slot */
  for (int i = 0; i < CONFIG_NUTTX_QENCODER_MAX_INSTANCES; i++)
    {
      if (!g_qe_instances.active[i])
        {
          instance_id = i;
          g_qe_instances.active[i] = true;
          g_qe_instances.num_instances++;
          break;
        }
    }

  nxsem_post(&g_qe_instances.lock);

  if (instance_id < 0)
    {
      qeerr("ERROR: No available instance slots (max: %d)\n",
            CONFIG_NUTTX_QENCODER_MAX_INSTANCES);
      return -ENOMEM;
    }

  qeinfo("Allocated instance ID: %d\n", instance_id);
  return instance_id;
}

/****************************************************************************
 * Name: qe_free_instance
 *
 * Description:
 *   Free an encoder instance
 *
 * Input Parameters:
 *   instance_id - Instance ID to free
 *
 * Returned Value:
 *   Zero (OK) on success; negative errno on failure
 *
 ****************************************************************************/

static int qe_free_instance(uint8_t instance_id)
{
  int ret;

  if (!g_instance_manager_initialized)
    {
      return -ENOTTY;
    }

  if (instance_id >= CONFIG_NUTTX_QENCODER_MAX_INSTANCES)
    {
      return -EINVAL;
    }

  /* Lock for thread safety */
  ret = nxsem_wait(&g_qe_instances.lock);
  if (ret < 0)
    {
      qeerr("ERROR: Failed to acquire instance lock: %d\n", ret);
      return ret;
    }

  if (!g_qe_instances.active[instance_id])
    {
      nxsem_post(&g_qe_instances.lock);
      qeerr("ERROR: Instance %d is not active\n", instance_id);
      return -ENODEV;
    }

  /* Free instance slot */
  g_qe_instances.active[instance_id] = false;
  g_qe_instances.instances[instance_id] = NULL;
  memset(g_qe_instances.devpaths[instance_id], 0,
         sizeof(g_qe_instances.devpaths[instance_id]));
  g_qe_instances.num_instances--;

  nxsem_post(&g_qe_instances.lock);

  qeinfo("Freed instance ID: %d\n", instance_id);
  return OK;
}

/****************************************************************************
 * Name: qe_find_instance
 *
 * Description:
 *   Find an encoder instance by ID
 *
 * Input Parameters:
 *   instance_id - Instance ID to find
 *
 * Returned Value:
 *   Pointer to instance structure; NULL on failure
 *
 ****************************************************************************/

static FAR struct nuttx_qe_lowerhalf_s *qe_find_instance(uint8_t instance_id)
{
  FAR struct nuttx_qe_lowerhalf_s *priv = NULL;
  int ret;

  if (!g_instance_manager_initialized)
    {
      return NULL;
    }

  if (instance_id >= CONFIG_NUTTX_QENCODER_MAX_INSTANCES)
    {
      return NULL;
    }

  /* Lock for thread safety */
  ret = nxsem_wait(&g_qe_instances.lock);
  if (ret < 0)
    {
      qeerr("ERROR: Failed to acquire instance lock: %d\n", ret);
      return NULL;
    }

  if (g_qe_instances.active[instance_id])
    {
      priv = g_qe_instances.instances[instance_id];
    }

  nxsem_post(&g_qe_instances.lock);

  return priv;
}
