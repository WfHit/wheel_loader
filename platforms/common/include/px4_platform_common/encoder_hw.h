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
} encoder_hw_config_t;

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
	encoder_hw_config_t config;     // Hardware configuration

	// Statistics
	uint32_t interrupt_count;       // Total interrupts processed
	uint32_t error_count;           // Error counter
} encoder_instance_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize encoder hardware layer
 *
 * @return 0 on success, negative on error
 */
int encoder_hw_init(void);

/**
 * @brief Create encoder instance
 *
 * @param config Encoder hardware configuration
 * @return Instance ID (0-15) on success, negative on error
 */
int encoder_hw_create_instance(const encoder_hw_config_t *config);

/**
 * @brief Start encoder instance
 *
 * @param encoder_id Instance ID
 * @return 0 on success, negative on error
 */
int encoder_hw_start(uint8_t encoder_id);

/**
 * @brief Stop encoder instance
 *
 * @param encoder_id Instance ID
 * @return 0 on success, negative on error
 */
int encoder_hw_stop(uint8_t encoder_id);

/**
 * @brief Destroy encoder instance
 *
 * @param encoder_id Instance ID
 * @return 0 on success, negative on error
 */
int encoder_hw_destroy(uint8_t encoder_id);

/**
 * @brief Get encoder instance information
 *
 * @param encoder_id Instance ID
 * @param instance Pointer to instance structure
 * @return 0 on success, negative on error
 */
int encoder_hw_get_instance(uint8_t encoder_id, encoder_instance_t *instance);

/**
 * @brief Get next pin event from ISR queue
 *
 * @param event Pointer to pin event structure
 * @return true if event available, false if queue empty
 */
bool encoder_hw_get_pin_event(pin_event_t *event);

/**
 * @brief Get total number of active encoder instances
 *
 * @return Number of active instances
 */
uint8_t encoder_hw_get_active_count(void);

/**
 * @brief Start the dedicated event processor thread
 *
 * This thread processes all pin events for all encoders in a centralized manner.
 *
 * @return 0 on success, negative on error
 */
int encoder_hw_start_event_processor(void);

/**
 * @brief Stop the dedicated event processor thread
 *
 * @return 0 on success, negative on error
 */
int encoder_hw_stop_event_processor(void);

/**
 * @brief Get processed encoder data asynchronously
 *
 * This function allows apps to get pre-processed encoder data without
 * needing to process raw pin events themselves.
 *
 * THREAD-SAFE: Uses semaphore-based protection to ensure apps always get
 * consistent data. Multiple app threads can safely call this function
 * simultaneously. The function will block briefly if the event processor
 * thread is currently updating the data.
 *
 * @param encoder_id Encoder ID to get data for
 * @param data Pointer to store processed data
 * @return true if new data is available, false if no new data
 */
bool encoder_hw_get_processed_data(uint8_t encoder_id, encoder_processed_data_t *data);

/**
 * @brief Get event processor thread statistics
 *
 * @param stats Pointer to store statistics
 * @return 0 on success, negative on error
 */
int encoder_hw_get_processor_stats(encoder_event_processor_stats_t *stats);

/**
 * @brief Check if event processor thread is running
 *
 * @return true if running, false if stopped
 */
bool encoder_hw_is_processor_running(void);

#ifdef __cplusplus
}
#endif
