#pragma once

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <lib/quad_encoder/quadencoder_types.h>

/* Configuration */
#define QUADENCODER_MAX_INSTANCES   4
#define QUADENCODER_FILTER_SAMPLES  4

/* Callback function type */
typedef void (*qe_callback_t)(void *arg, int32_t position, int32_t velocity);

/* Hardware register offsets (example for generic implementation) */
#define QUADENCODER_CTRL_OFFSET     0x00
#define QUADENCODER_STATUS_OFFSET   0x04
#define QUADENCODER_COUNT_OFFSET    0x08
#define QUADENCODER_CLEAR_OFFSET    0x0C

/* Control register bits */
#define QUADENCODER_CTRL_ENABLE     (1 << 0)
#define QUADENCODER_CTRL_RESET      (1 << 1)
#define QUADENCODER_CTRL_INDEX_EN   (1 << 2)
#define QUADENCODER_CTRL_FILTER_EN  (1 << 3)
#define QUADENCODER_CTRL_MODE_SHIFT 4
#define QUADENCODER_CTRL_MODE_MASK  (0x3 << 4)
#define QUADENCODER_CTRL_FILTER_SHIFT 8

/* Status register bits */
#define QUADENCODER_STATUS_A_EDGE   (1 << 0)
#define QUADENCODER_STATUS_B_EDGE   (1 << 1)
#define QUADENCODER_STATUS_INDEX    (1 << 2)
#define QUADENCODER_STATUS_DIR      (1 << 3)

/* Private data structure */
struct quadencoder_dev_s {
	/* Configuration */
	uint8_t id;                     /* Encoder instance ID */
	enum quadencoder_type_e type;   /* Encoder type */
	enum quadencoder_mode_e mode;   /* Counting mode */
	uint32_t resolution;            /* Counts per revolution */
	bool index_enabled;             /* Index pulse enabled */

	/* State */
	int32_t position;               /* Current position */
	int32_t last_position;          /* Previous position */
	int32_t velocity;               /* Calculated velocity */
	uint32_t timestamp;             /* Last update timestamp */

	/* Hardware specific */
	uint32_t base_addr;             /* Base register address */
	int irq_a;                      /* IRQ for channel A */
	int irq_b;                      /* IRQ for channel B */
	int irq_z;                      /* IRQ for index */

	/* Filtering */
	uint8_t filter_level;           /* Digital filter level */
	uint16_t filter_buffer[QUADENCODER_FILTER_SAMPLES];

	/* Synchronization */
	sem_t exclsem;                  /* Exclusive access */

	/* Callbacks */
	qe_callback_t callback;         /* User callback */
	void *arg;                      /* Callback argument */
};

/* Public function prototypes */
int quadencoder_common_initialize(struct quadencoder_dev_s *dev,
				  uint8_t id,
				  uint32_t base_addr);
int quadencoder_common_setup(struct quadencoder_dev_s *dev);
int quadencoder_common_shutdown(struct quadencoder_dev_s *dev);
int32_t quadencoder_common_position(struct quadencoder_dev_s *dev);
int quadencoder_common_reset(struct quadencoder_dev_s *dev);
int quadencoder_common_setmode(struct quadencoder_dev_s *dev,
			       enum quadencoder_mode_e mode);

/* GPIO-based encoder functions */
int quadencoder_gpio_initialize(FAR struct quadencoder_config_s *config,
				FAR struct quadencoder_dev_s **dev);

/* Timer-based encoder functions */
int quadencoder_timer_initialize(FAR struct quadencoder_config_s *config,
				 FAR struct quadencoder_dev_s **dev);
