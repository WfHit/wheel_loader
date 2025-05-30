#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Pin configuration structure */
struct quad_encoder_pins_s {
	uint32_t pin_a;         /* Channel A GPIO pin */
	uint32_t pin_b;         /* Channel B GPIO pin */
	uint32_t pin_z;         /* Index/Z GPIO pin (optional) */

	/* Pin configuration flags */
	uint32_t pin_mode;      /* GPIO mode (input, pull-up, etc.) */
	bool invert_a;          /* Invert channel A signal */
	bool invert_b;          /* Invert channel B signal */
	bool invert_z;          /* Invert index signal */
};

/* Hardware configuration */
struct quad_encoder_config_s {
	/* Pin configuration */
	struct quad_encoder_pins_s pins;

	/* Hardware interface type */
	enum {
		QUAD_ENCODER_HW_GPIO,     /* GPIO-based (software) */
		QUAD_ENCODER_HW_TIMER,    /* Hardware timer capture */
		QUAD_ENCODER_HW_DEDICATED /* Dedicated encoder peripheral */
	} hw_type;

	/* For timer-based encoders */
	int timer_id;               /* Timer peripheral ID */
	int channel_a;              /* Timer channel for A */
	int channel_b;              /* Timer channel for B */

	/* For dedicated encoder peripherals */
	uint32_t periph_base;       /* Peripheral base address */
	int periph_id;              /* Peripheral ID */
};

/* Encoder types */
enum quad_encoder_type_e {
	QUAD_ENCODER_TYPE_INCREMENTAL = 0,
	QUAD_ENCODER_TYPE_ABSOLUTE,
	QUAD_ENCODER_TYPE_DIFFERENTIAL
};

/* Encoder modes */
enum quad_encoder_mode_e {
	QUAD_ENCODER_MODE_X1 = 1,    /* Count on A edge only */
	QUAD_ENCODER_MODE_X2 = 2,    /* Count on A and B edges */
	QUAD_ENCODER_MODE_X4 = 4     /* Count on all edges */
};
