#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Pin configuration structure */
struct quadencoder_pins_s {
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
struct quadencoder_config_s {
	/* Pin configuration */
	struct quadencoder_pins_s pins;

	/* Hardware interface type */
	enum {
		QUADENCODER_HW_GPIO,     /* GPIO-based (software) */
		QUADENCODER_HW_TIMER,    /* Hardware timer capture */
		QUADENCODER_HW_DEDICATED /* Dedicated encoder peripheral */
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
enum quadencoder_type_e {
	QUADENCODER_TYPE_INCREMENTAL = 0,
	QUADENCODER_TYPE_ABSOLUTE,
	QUADENCODER_TYPE_DIFFERENTIAL
};

/* Encoder modes */
enum quadencoder_mode_e {
	QUADENCODER_MODE_X1 = 1,    /* Count on A edge only */
	QUADENCODER_MODE_X2 = 2,    /* Count on A and B edges */
	QUADENCODER_MODE_X4 = 4     /* Count on all edges */
};
