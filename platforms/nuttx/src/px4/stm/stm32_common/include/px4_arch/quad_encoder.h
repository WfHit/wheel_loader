#pragma once

#include <stdint.h>
#include <stdbool.h>

// Maximum number of encoder instances supported
#define ENCODER_MAX_INSTANCES 4

// Simplified raw data structure from platform
typedef struct {
	uint64_t timestamp;         // High-resolution timestamp
	int64_t counter;           // Raw encoder counter value
	bool direction_forward;    // Current rotation direction
	bool counter_reset;        // True if counter was reset by resolution boundary
	bool reset_direction_forward; // Direction when reset occurred (true=forward, false=reverse)
} encoder_raw_data_t;

// Encoder hardware configuration
typedef struct {
	uint32_t gpio_a;                // GPIO pin for channel A
	uint32_t gpio_b;                // GPIO pin for channel B
	uint16_t pulses_per_revolution; // Encoder resolution (counter resets at this value)
} quad_encoder_config_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize all quad encoder instances
 *
 * This function initializes all encoder instances according to their configurations.
 * Must be called once during board initialization before using any other functions.
 *
 * @return 0 on success, negative error code on failure
 */
int quad_encoder_init(void);

/**
 * @brief Start a quad encoder instance (enable interrupts)
 *
 * Enables interrupts and starts data collection for the specified encoder.
 * The encoder must have been initialized with quad_encoder_init() first.
 *
 * @param encoder_id Encoder instance ID
 * @return 0 on success, negative error code on failure
 */
int quad_encoder_start(uint8_t encoder_id);

/**
 * @brief Stop a quad encoder instance (disable interrupts)
 *
 * Disables interrupts and stops data collection for the specified encoder.
 *
 * @param encoder_id Encoder instance ID
 * @return 0 on success, negative error code on failure
 */
int quad_encoder_stop(uint8_t encoder_id);

/**
 * @brief Get raw encoder data from platform
 *
 * Gets the latest raw encoder data (timestamp, counter, direction) from the platform.
 * This is a simplified interface where all processing is done in the module.
 *
 * @param encoder_id Encoder instance ID
 * @param raw_data Pointer to store raw encoder data
 * @return true if data was retrieved successfully, false otherwise
 */
bool quad_encoder_get_raw_data(uint8_t encoder_id, encoder_raw_data_t *raw_data);

#ifdef __cplusplus
}
#endif
