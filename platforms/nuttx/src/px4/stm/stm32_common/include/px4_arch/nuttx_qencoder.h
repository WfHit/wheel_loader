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

#ifndef __PX4_NUTTX_STM32_COMMON_NUTTX_QENCODER_H
#define __PX4_NUTTX_STM32_COMMON_NUTTX_QENCODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/qencoder.h>
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug Configuration
 *
 * QE_DEBUG levels:
 *   0 = No debug output (production)
 *   1 = Basic info and errors only
 *   2 = Add warnings
 *   3 = Add detailed debug information
 *
 * Can be overridden via:
 *   - Build configuration: -DQE_DEBUG=0
 *   - Board defconfig: CONFIG_QE_DEBUG_LEVEL=1
 *   - menuconfig: Enable Debug Features -> Sensor Debug -> QEncoder Debug
 */

#ifndef QE_DEBUG
#  ifdef CONFIG_QE_DEBUG_LEVEL
#    define QE_DEBUG CONFIG_QE_DEBUG_LEVEL
#  else
#    define QE_DEBUG 1
#  endif
#endif

/* Debug output macros */
#if QE_DEBUG > 0
#  include <syslog.h>

/* Basic info and errors (level 1+) */
#  define qeinfo(format, ...)   syslog(LOG_INFO, "[QE] " format, ##__VA_ARGS__)
#  define qeerr(format, ...)    syslog(LOG_ERR, "[QE_ERR] " format, ##__VA_ARGS__)

#  if QE_DEBUG >= 2
/* Add warnings (level 2+) */
#    define qewarn(format, ...)  syslog(LOG_WARNING, "[QE_WARN] " format, ##__VA_ARGS__)
#  else
#    define qewarn(format, ...)
#  endif

#  if QE_DEBUG >= 3
/* Add detailed debug (level 3+) */
#    define qedebug(format, ...) syslog(LOG_DEBUG, "[QE_DBG] " format, ##__VA_ARGS__)
#  else
#    define qedebug(format, ...)
#  endif

#else
/* No debug output */
#  define qeinfo(format, ...)
#  define qeerr(format, ...)
#  define qewarn(format, ...)
#  define qedebug(format, ...)
#endif

/* Additional specialized debug macros */
#if QE_DEBUG >= 3
#  define qe_isr_debug(format, ...)  syslog(LOG_DEBUG, "[QE_ISR] " format, ##__VA_ARGS__)
#  define qe_gpio_debug(format, ...) syslog(LOG_DEBUG, "[QE_GPIO] " format, ##__VA_ARGS__)
#  define qe_pos_debug(format, ...)  syslog(LOG_DEBUG, "[QE_POS] " format, ##__VA_ARGS__)
#else
#  define qe_isr_debug(format, ...)
#  define qe_gpio_debug(format, ...)
#  define qe_pos_debug(format, ...)
#endif

/* Performance timing macros (only in highest debug level) */
#if QE_DEBUG >= 3
#  include <nuttx/clock.h>
#  define QE_PERF_START(var) uint32_t var = clock_systime_ticks()
#  define QE_PERF_END(var, msg) do { \
     uint32_t elapsed = clock_systime_ticks() - var; \
     qedebug("PERF: %s took %lu ticks\n", msg, (unsigned long)elapsed); \
   } while(0)
#else
#  define QE_PERF_START(var)
#  define QE_PERF_END(var, msg)
#endif

/* Debug helper functions */
#if QE_DEBUG >= 2
#  define QE_ASSERT(condition, msg) do { \
     if (!(condition)) { \
       qeerr("ASSERTION FAILED: %s at %s:%d\n", msg, __FILE__, __LINE__); \
     } \
   } while(0)
#else
#  define QE_ASSERT(condition, msg)
#endif

/* Configuration */
#define CONFIG_NUTTX_QENCODER_MAX_INSTANCES 8

/****************************************************************************
 * Usage Examples
 ****************************************************************************/

/*
 * Basic usage in your encoder driver:
 *
 * #include "qencoder_debug.h"
 *
 * void my_function(void)
 * {
 *   qeinfo("Encoder initialized successfully\n");
 *   qeerr("GPIO configuration failed: %d\n", ret);
 *   qewarn("Position overflow detected at %d\n", position);
 *   qedebug("ISR: state=%d, pos=%d\n", state, position);
 *
 *   QE_PERF_START(timer);
 *   // ... some operation ...
 *   QE_PERF_END(timer, "GPIO setup");
 *
 *   QE_ASSERT(position >= 0, "Position should not be negative");
 * }
 *
 * Build-time control:
 *   make px4_nxt-dual-wl-rear_default EXTRA_DEFINES="-DQE_DEBUG=0"  # No debug
 *   make px4_nxt-dual-wl-rear_default EXTRA_DEFINES="-DQE_DEBUG=3"  # Full debug
 *
 * Runtime filtering with dmesg:
 *   nsh> dmesg | grep QE          # All encoder messages
 *   nsh> dmesg | grep QE_ERR      # Only errors
 *   nsh> dmesg | grep QE_WARN     # Only warnings
 *   nsh> dmesg | grep QE_ISR      # Only ISR debug
 *   nsh> dmesg | grep QE_GPIO     # Only GPIO debug
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Extended IOCTL commands */
#define QEIOC_VELOCITY      _QEIOC(QE_FIRST + 10)
#define QEIOC_INDEX_COUNT   _QEIOC(QE_FIRST + 11)
#define QEIOC_GET_STATUS    _QEIOC(QE_FIRST + 12)
#define QEIOC_CALIBRATE     _QEIOC(QE_FIRST + 13)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Encoder configuration */
struct nuttx_qe_config_s
{
  /* GPIO mode configuration */
  struct {
    uint32_t phase_a;    /* Phase A GPIO pin */
    uint32_t phase_b;    /* Phase B GPIO pin */
    uint32_t index;      /* Optional index/Z pin */
  } gpio;

  /* Common parameters */
  uint32_t resolution;   /* Encoder counts per revolution */
  bool use_index;        /* Enable index channel */
  bool x4_mode;          /* Enable x4 quadrature mode */
  bool invert_dir;       /* Invert counting direction */
};

/* Status information */
struct qe_status_s
{
  uint32_t error_count;      /* Quadrature errors detected */
  uint32_t index_count;      /* Number of index pulses */
  int32_t last_index_pos;    /* Position at last index */
  bool index_found;          /* Index pulse detected */
  uint32_t sample_rate;      /* Actual sampling rate */
};

/* Instance information structure */
struct nuttx_qe_instance_info_s
{
  uint8_t instance_id;
  char devpath[16];
  bool active;
  struct nuttx_qe_config_s config;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: nuttx_qencoder_initialize
 *
 * Description:
 *   Initialize quadrature encoder and register as a character device.
 *
 * Input Parameters:
 *   config  - Encoder configuration
 *   devpath - Device path (e.g., "/dev/qe0")
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

__EXPORT int nuttx_qencoder_initialize(FAR const struct nuttx_qe_config_s *config,
                                      FAR const char *devpath);

/****************************************************************************
 * Name: nuttx_qencoder_uninitialize
 *
 * Description:
 *   Uninitialize a specific quadrature encoder instance.
 *
 * Input Parameters:
 *   instance_id - Instance ID to uninitialize
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

__EXPORT int nuttx_qencoder_uninitialize(uint8_t instance_id);

/****************************************************************************
 * Name: nuttx_qencoder_uninitialize_all
 *
 * Description:
 *   Uninitialize all active quadrature encoder instances.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

__EXPORT int nuttx_qencoder_uninitialize_all(void);

/****************************************************************************
 * Name: nuttx_qencoder_get_instance_count
 *
 * Description:
 *   Get number of active encoder instances.
 *
 * Returned Value:
 *   Number of active instances; negative errno on failure.
 *
 ****************************************************************************/

__EXPORT int nuttx_qencoder_get_instance_count(void);

/****************************************************************************
 * Name: nuttx_qencoder_list_instances
 *
 * Description:
 *   List all active encoder instances.
 *
 * Input Parameters:
 *   instances - Array to store instance information
 *   max_count - Maximum number of instances to return
 *
 * Returned Value:
 *   Number of instances returned; negative errno on failure.
 *
 ****************************************************************************/

__EXPORT int nuttx_qencoder_list_instances(FAR struct nuttx_qe_instance_info_s *instances,
                                          uint8_t max_count);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __PX4_NUTTX_STM32_COMMON_NUTTX_QENCODER_H */
