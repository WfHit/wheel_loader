#pragma once

#include <stdint.h>
#include <stdbool.h>

// Maximum number of encoder instances supported
#define ENCODER_MAX_INSTANCES 4

// Encoder operating modes
typedef enum {
	ENCODER_MODE_RELATIVE = 0,      // Relative positioning mode
	ENCODER_MODE_ABSOLUTE = 1       // Absolute positioning mode
} encoder_mode_t;

// Pin event types for minimal ISR
typedef enum {
	PIN_EVENT_RISING = 0,           // Rising edge detected
	PIN_EVENT_FALLING = 1           // Falling edge detected
} pin_event_type_t;

// Raw pin event structure (minimal ISR data)
typedef struct {
	uint64_t timestamp;             // High-resolution timestamp
	uint8_t encoder_id;             // Encoder instance ID
	uint8_t pin_id;                 // Pin ID (0=A, 1=B, 2=INDEX)
	uint8_t event_type;             // pin_event_type_t
	uint8_t pin_state;              // Current pin state (0/1)
} pin_event_t;

// Processed encoder data structure (from event processor thread)
typedef struct {
	uint64_t timestamp;
	uint8_t encoder_id;

	// Position data
	int64_t position_raw;
	float position_rad;
	float position_deg;

	// Velocity data
	float velocity_rad_s;
	float velocity_rpm;

	// Direction and status
	bool direction_forward;
	bool index_detected;

	// Counters
	uint64_t pulse_count;
	uint32_t direction_changes;
	uint32_t error_count;

	// Validity flags
	bool data_valid;
	bool position_updated;
	bool velocity_updated;
} encoder_processed_data_t;

// Event processor thread statistics
typedef struct {
	uint64_t events_processed;
	uint64_t events_dropped;
	uint32_t processing_time_us;
	uint32_t max_processing_time_us;
	uint32_t queue_utilization_percent;
} encoder_event_processor_stats_t;

// Encoder hardware configuration
typedef struct {
	uint32_t gpio_a;                // GPIO pin for channel A
	uint32_t gpio_b;                // GPIO pin for channel B
	uint32_t gpio_index;            // GPIO pin for index (0 if unused)
	uint8_t irq_priority;           // Interrupt priority
	uint16_t pulses_per_revolution; // Encoder resolution
	encoder_mode_t mode;            // Operating mode
	bool swap_channels;             // Swap A/B channels for direction
	bool enable_index;              // Enable index channel
} quadrature_encoder_config_t;

// Encoder instance state
typedef struct {
	uint8_t encoder_id;             // Instance ID
	bool is_initialized;            // Initialization status
	bool is_active;                 // Active status

	// Pin mapping for fast ISR lookup
	uint32_t gpio_a_pin;            // GPIO A pin number
	uint32_t gpio_b_pin;            // GPIO B pin number
	uint32_t gpio_index_pin;        // GPIO index pin number

	// Configuration
	quadrature_encoder_config_t config;     // Hardware configuration

	// Statistics
	uint32_t interrupt_count;       // Total interrupts processed
	uint32_t error_count;           // Error counter
} encoder_instance_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the quadrature encoder driver
 *
 * This function must be called before any other quadrature encoder functions.
 * It initializes internal data structures and prepares the system for encoder operations.
 *
 * @return 0 on success, negative error code on failure
 * @retval 0 Success
 * @retval -1 Semaphore initialization failed
 */
int qencoder_init(void);

/**
 * @brief Create a new quadrature encoder instance
 *
 * @param config Pointer to encoder configuration structure
 * @return encoder_id on success, negative error code on failure
 * @retval >=0 Encoder instance ID
 * @retval -1 Driver not initialized
 * @retval -2 Invalid configuration pointer
 * @retval -3 No available instances
 */
int qencoder_create_instance(const quadrature_encoder_config_t *config);

/**
 * @brief Start an encoder instance
 *
 * Configures GPIO pins and enables interrupts for the specified encoder.
 *
 * @param encoder_id Encoder instance ID
 * @return 0 on success, negative error code on failure
 * @retval 0 Success or already active
 * @retval -1 Invalid encoder ID
 * @retval -2 Instance not initialized
 * @retval <-2 GPIO configuration error
 */
int qencoder_start(uint8_t encoder_id);

/**
 * @brief Stop an encoder instance
 *
 * Disables interrupts and unconfigures GPIO pins for the specified encoder.
 *
 * @param encoder_id Encoder instance ID
 * @return 0 on success, negative error code on failure
 * @retval 0 Success or already stopped
 * @retval -1 Invalid encoder ID
 */
int qencoder_stop(uint8_t encoder_id);

/**
 * @brief Destroy an encoder instance
 *
 * Stops the encoder if active and marks the instance as uninitialized.
 *
 * @param encoder_id Encoder instance ID
 * @return 0 on success, negative error code on failure
 * @retval 0 Success
 * @retval -1 Invalid encoder ID
 * @retval -2 Instance not initialized
 */
int qencoder_destroy(uint8_t encoder_id);

/**
 * @brief Get encoder instance information
 *
 * @param encoder_id Encoder instance ID
 * @param instance Pointer to store instance information
 * @return 0 on success, negative error code on failure
 * @retval 0 Success
 * @retval -1 Invalid parameters
 * @retval -2 Instance not initialized
 */
int qencoder_get_instance(uint8_t encoder_id, encoder_instance_t *instance);

/**
 * @brief Get the next pin event from the event queue
 *
 * This function retrieves pin events generated by the ISR for processing
 * by the event processor thread.
 *
 * @param event Pointer to store the pin event
 * @return true if event was retrieved, false if queue is empty
 */
bool qencoder_get_pin_event(pin_event_t *event);

/**
 * @brief Get the number of active encoder instances
 *
 * @return Number of active encoder instances
 */
uint8_t qencoder_get_active_count(void);

/**
 * @brief Start the event processor thread
 *
 * The event processor thread handles all pin events from ISRs and updates
 * encoder positions and velocities.
 *
 * @return 0 on success, negative error code on failure
 * @retval 0 Success or already running
 * @retval -1 Driver not initialized
 * @retval -2 Thread creation failed
 * @retval -3 Semaphore initialization failed
 */
int qencoder_start_event_processor(void);

/**
 * @brief Stop the event processor thread
 *
 * @return 0 on success, negative error code on failure
 * @retval 0 Success or already stopped
 */
int qencoder_stop_event_processor(void);

/**
 * @brief Get processed encoder data
 *
 * Retrieves the latest processed position, velocity, and status data for
 * the specified encoder in a thread-safe manner.
 *
 * @param encoder_id Encoder instance ID
 * @param data Pointer to store processed data
 * @return true if data was retrieved successfully, false otherwise
 */
bool qencoder_get_processed_data(uint8_t encoder_id, encoder_processed_data_t *data);

/**
 * @brief Get event processor thread statistics
 *
 * @param stats Pointer to store statistics
 * @return 0 on success, negative error code on failure
 * @retval 0 Success
 * @retval -1 Invalid stats pointer
 */
int qencoder_get_processor_stats(encoder_event_processor_stats_t *stats);

/**
 * @brief Check if event processor thread is running
 *
 * @return true if processor thread is running, false otherwise
 */
bool qencoder_is_processor_running(void);

#ifdef __cplusplus
}
#endif
