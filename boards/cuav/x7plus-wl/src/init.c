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
 * @file init.c
 *
 * CUAV X7+ WL (Wheel Loader) Controller Board-specific early startup code.
 * This file implements the board_app_initialize() function that is called
 * early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

#include "board_config.h"

#include <syslog.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <arch/board/board.h>
#include "arm_internal.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_dma_alloc.h>

#include <mpu.h>

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *   Reset peripheral power rails for wheel loader controller
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	/* set the peripheral rails off */
	VDD_5V_PERIPH_EN(false);
	board_control_spi_sensors_power(false, 0xffff);

	bool last = READ_SPEKTRUM_POWER();
	/* Keep Spektum on to discharge rail*/
	SPEKTRUM_POWER(false);

	/* wait for the peripheral rail to reach GND */
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "wheel loader board reset done, %d ms\n", ms);

	/* re-enable power */

	/* switch the peripheral rail back on */
	SPEKTRUM_POWER(last);
	board_control_spi_sensors_power(true, 0xffff);
	VDD_5V_PERIPH_EN(true);
}

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 *   Optionally provided function called on entry to board_system_reset
 *   It should perform any house keeping prior to the rest.
 *
 * Input Parameters:
 *   status_register - The value of the reset status register
 *
 * Returned Value:
 *   Status register value is passed through
 *
 ************************************************************************************/

__EXPORT uint32_t board_on_reset(uint32_t status)
{
	/* configure the GPIO pins to outputs and keep them low */
	const uint32_t gpio[] = {
		GPIO_GPIO0_OUTPUT,
		GPIO_GPIO1_OUTPUT,
		GPIO_GPIO2_OUTPUT,
		GPIO_GPIO3_OUTPUT,
		GPIO_GPIO4_OUTPUT,
		GPIO_GPIO5_OUTPUT,
	};

	px4_gpio_init(gpio, arraySize(gpio));

	/* On resets invoked from system (not boot) it is millons of times faster to set
	 * the GPIO to the inactive state and let the pullup or pulldown pull it to the
	 * final state. Just change the pins from the active low to inactive high.
	 */
	px4_gpio_init(PX4_GPIO_INIT_LIST);

	return status;
}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry
 *   point is called early in the initialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{
	/* Reset PWM first thing */
	board_on_reset(0);

	/* configure LEDs */
	board_autoled_initialize();

	/* configure pins */
	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	/* configure SPI interfaces */
	stm32_spiinitialize();

	/* configure USB interfaces */
	stm32_usbinitialize();

}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

__EXPORT int board_app_initialize(uintptr_t arg)
{

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
	/* board_configure() */

	board_peripheral_reset(10);

	/* configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

	/* set up the serial DMA polling */
	static struct hrt_call serial_dma_call;
	struct timespec ts;

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;

	hrt_call_every(&serial_dma_call,
		       ts_to_abstime(&ts),
		       ts_to_abstime(&ts),
		       (hrt_callout)stm32_serial_dma_poll,
		       NULL);

	/* initial LED state */
	drv_led_start();
	led_off(LED_RED);
	led_off(LED_GREEN);
	led_off(LED_BLUE);

	int ret = px4_platform_init();

	if (ret < 0) {
		syslog(LOG_ERR, "[boot] px4_platform_init() failed: %d\n", ret);
	}

#ifdef CONFIG_MMCSD
	ret = stm32_sdio_initialize();

	if (ret != OK) {
		board_autoled_on(LED_AMBER);
		syslog(LOG_ERR, "[boot] Failed to initialize MMC/SD: %d\n", ret);

	} else {
		syslog(LOG_INFO, "[boot] Successfully initialized MMC/SD\n");
	}

#endif

	/* Configure the HW based on the manifest */

	px4_platform_configure();

#endif

	return OK;
}
