/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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
 * @file can.c
 *
 * CUAV X7+ WL CAN Configuration.
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/can/can.h>
#include <arch/board/board.h>
#include "chip.h"
#include "stm32_can.h"
#include "board_config.h"

#if defined(CONFIG_CAN) && (defined(CONFIG_STM32_CAN1) || defined(CONFIG_STM32_CAN2) || defined(CONFIG_STM32_CAN3))

static struct can_dev_s *g_can_devices[3] = {NULL, NULL, NULL};

int stm32_caninitialize(void)
{
	int ret = 0;

#ifdef CONFIG_STM32_CAN1
	stm32_configgpio(GPIO_CAN1_RX);
	stm32_configgpio(GPIO_CAN1_TX);
	g_can_devices[0] = stm32_caninitialize(1);

	if (g_can_devices[0] == NULL) {
		ret = -ENODEV;
	}

#endif

#ifdef CONFIG_STM32_CAN2
	stm32_configgpio(GPIO_CAN2_RX);
	stm32_configgpio(GPIO_CAN2_TX);
	g_can_devices[1] = stm32_caninitialize(2);

	if (g_can_devices[1] == NULL && ret == 0) {
		ret = -ENODEV;
	}

#endif

#ifdef CONFIG_STM32_CAN3
	stm32_configgpio(GPIO_CAN3_RX);
	stm32_configgpio(GPIO_CAN3_TX);
	g_can_devices[2] = stm32_caninitialize(3);

	if (g_can_devices[2] == NULL && ret == 0) {
		ret = -ENODEV;
	}

#endif
	return ret;
}

#endif // CONFIG_CAN
