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
 * @file spi.cpp
 *
 * Board-specific SPI functions for CUAV X7+ WL (Wheel Loader) Controller
 */

#include <px4_platform_common/px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32_gpio.h>
#include "board_config.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************/

__EXPORT void stm32_spiinitialize(void)
{
	board_control_spi_sensors_power_configgpio();
	board_control_spi_sensors_power(true, 0xffff);

#ifdef CONFIG_STM32F7_SPI1
	stm32_configgpio(GPIO_SPI_CS_GYRO1);
	stm32_configgpio(GPIO_SPI_CS_GYRO2);
	stm32_configgpio(GPIO_SPI_CS_ACCEL1);
	stm32_configgpio(GPIO_SPI_CS_BARO1);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	stm32_gpiowrite(GPIO_SPI_CS_GYRO1, 1);
	stm32_gpiowrite(GPIO_SPI_CS_GYRO2, 1);
	stm32_gpiowrite(GPIO_SPI_CS_ACCEL1, 1);
	stm32_gpiowrite(GPIO_SPI_CS_BARO1, 1);

	stm32_configgpio(GPIO_GYRO1_DRDY);
	stm32_configgpio(GPIO_GYRO2_DRDY);
	stm32_configgpio(GPIO_ACCEL1_DRDY);
#endif

#ifdef CONFIG_STM32F7_SPI2
	stm32_configgpio(GPIO_SPI_CS_BARO2);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	stm32_gpiowrite(GPIO_SPI_CS_BARO2, 1);
#endif
}

/****************************************************************************
 * Name: stm32_spi1select and stm32_spi1status
 *
 * Description:
 *   Called by stm32 spi driver on bus 1.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_SPI1
__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case PX4_SPIDEV_GYRO1:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO2, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_GYRO1, !selected);
		break;

	case PX4_SPIDEV_GYRO2:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_GYRO2, !selected);
		break;

	case PX4_SPIDEV_ACCEL1:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_GYRO2, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL1, !selected);
		break;

	case PX4_SPIDEV_BARO1:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_GYRO2, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL1, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO1, !selected);
		break;

	default:
		break;
	}
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

/****************************************************************************
 * Name: stm32_spi2select and stm32_spi2status
 *
 * Description:
 *   Called by stm32 spi driver on bus 2.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_SPI2
__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	switch (devid) {
	case PX4_SPIDEV_BARO2:
		stm32_gpiowrite(GPIO_SPI_CS_BARO2, !selected);
		break;

	default:
		break;
	}
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif
