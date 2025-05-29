#include <nuttx/config.h>
#include <nuttx/timers/timer.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include <errno.h>
#include <debug.h>
#include <string.h>

#include "quadencoder_common.h"
#include <lib/quad_encoder/quadencoder_types.h>

/* Timer-based encoder implementation */
struct timer_encoder_s {
	struct quadencoder_dev_s common;
	int timer_id;
	int channel_a;
	int channel_b;
};

/****************************************************************************
 * Name: quadencoder_timer_initialize
 *
 * Description:
 *   Initialize hardware timer-based encoder (more efficient than GPIO)
 *
 ****************************************************************************/
int quadencoder_timer_initialize(FAR struct quadencoder_config_s *config,
				 FAR struct qe_lowerhalf_s **lower)
{
	struct timer_encoder_s *priv;
	uint32_t timclk;
	uint32_t prescaler;
	uint32_t period;

	/* Allocate private data */
	priv = (struct timer_encoder_s *)kmm_zalloc(sizeof(struct timer_encoder_s));

	if (!priv) {
		return -ENOMEM;
	}

	/* Store configuration */
	priv->timer_id = config->timer_id;
	priv->channel_a = config->channel_a;
	priv->channel_b = config->channel_b;

	/* Configure timer for encoder mode */
	/* This is MCU-specific, example for STM32 */
#ifdef CONFIG_ARCH_CHIP_STM32
	/* Enable timer clock */
	modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIMxEN(priv->timer_id));

	/* Configure encoder interface mode */
	putreg32(TIM_SMCR_SMS_ENCODER3, STM32_TIMx_SMCR(priv->timer_id));

	/* Configure input pins through AF */
	stm32_configgpio(config->pins.pin_a | GPIO_AF(TIMER_AF));
	stm32_configgpio(config->pins.pin_b | GPIO_AF(TIMER_AF));

	/* Set counter period for overflow handling */
	putreg32(0xFFFF, STM32_TIMx_ARR(priv->timer_id));

	/* Enable timer */
	modifyreg32(STM32_TIMx_CR1(priv->timer_id), 0, TIM_CR1_CEN);
#endif

	/* Initialize common structure */
	quadencoder_common_initialize(&priv->common, 0, 0);

	*lower = &priv->common.lower;
	return OK;
}
