#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/gpio.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <debug.h>
#include <errno.h>
#include <string.h>

#include <lib/quad_encoder/quadencoder_types.h>

#include "quadencoder_common.h"

/* GPIO-based encoder state */
struct gpio_encoder_s {
	struct quadencoder_dev_s common;    /* Common encoder data */
	struct quadencoder_pins_s pins;     /* Pin configuration */

	/* Pin states */
	bool last_a;
	bool last_b;

	/* Gray code lookup table for X4 decoding */
	int8_t decode_table[16];
};

/* GPIO interrupt handler */
static int gpio_encoder_interrupt(int irq, void *context, void *arg)
{
	struct gpio_encoder_s *priv = (struct gpio_encoder_s *)arg;
	bool current_a, current_b;
	uint8_t state;
	int8_t delta;

	/* Read current pin states */
	current_a = gpio_read(priv->pins.pin_a);
	current_b = gpio_read(priv->pins.pin_b);

	/* Apply inversion if configured */
	if (priv->pins.invert_a) {
		current_a = !current_a;
	}

	if (priv->pins.invert_b) {
		current_b = !current_b;
	}

	/* Calculate state transition */
	state = (priv->last_a << 3) | (priv->last_b << 2) |
		(current_a << 1) | current_b;

	/* Decode using lookup table */
	delta = priv->decode_table[state];

	/* Update position */
	if (delta != 0) {
		priv->common.position += delta;

		/* Velocity calculation */
		uint32_t current_time = up_systime_ticks();
		uint32_t dt = current_time - priv->common.timestamp;

		if (dt > 0) {
			priv->common.velocity = (priv->common.position - priv->common.last_position)
						* USEC_PER_SEC / dt;
			priv->common.last_position = priv->common.position;
			priv->common.timestamp = current_time;
		}
	}

	/* Update last states */
	priv->last_a = current_a;
	priv->last_b = current_b;

	/* Notify callback if registered */
	if (priv->common.callback) {
		priv->common.callback(priv->common.arg);
	}

	return OK;
}

/* Initialize GPIO-based encoder */
int quadencoder_gpio_initialize(FAR struct quadencoder_config_s *config,
				FAR struct qe_lowerhalf_s **lower)
{
	struct gpio_encoder_s *priv;
	int ret;

	/* Allocate private data */
	priv = (struct gpio_encoder_s *)kmm_zalloc(sizeof(struct gpio_encoder_s));

	if (!priv) {
		return -ENOMEM;
	}

	/* Copy pin configuration */
	memcpy(&priv->pins, &config->pins, sizeof(struct quadencoder_pins_s));

	/* Initialize decode table for X4 mode */
	/* This table maps state transitions to count changes */
	memset(priv->decode_table, 0, sizeof(priv->decode_table));
	priv->decode_table[0b0001] =  1;  /* 00->01 */
	priv->decode_table[0b0111] =  1;  /* 01->11 */
	priv->decode_table[0b1110] =  1;  /* 11->10 */
	priv->decode_table[0b1000] =  1;  /* 10->00 */
	priv->decode_table[0b0010] = -1;  /* 00->10 */
	priv->decode_table[0b1011] = -1;  /* 10->11 */
	priv->decode_table[0b1101] = -1;  /* 11->01 */
	priv->decode_table[0b0100] = -1;  /* 01->00 */

	/* Configure GPIO pins */
	ret = gpio_config(priv->pins.pin_a,
			  GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI);

	if (ret < 0) {
		goto errout;
	}

	ret = gpio_config(priv->pins.pin_b,
			  GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI);

	if (ret < 0) {
		goto errout;
	}

	/* Configure index pin if used */
	if (priv->pins.pin_z != 0) {
		ret = gpio_config(priv->pins.pin_z,
				  GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI);

		if (ret < 0) {
			goto errout;
		}
	}

	/* Attach interrupts */
	ret = gpio_attach(priv->pins.pin_a, gpio_encoder_interrupt, priv);

	if (ret < 0) {
		goto errout;
	}

	ret = gpio_attach(priv->pins.pin_b, gpio_encoder_interrupt, priv);

	if (ret < 0) {
		goto errout;
	}

	/* Read initial states */
	priv->last_a = gpio_read(priv->pins.pin_a);
	priv->last_b = gpio_read(priv->pins.pin_b);

	/* Initialize common structure */
	quadencoder_common_initialize(&priv->common, 0, 0);

	*lower = &priv->common.lower;
	return OK;

errout:
	kmm_free(priv);
	return ret;
}
