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
