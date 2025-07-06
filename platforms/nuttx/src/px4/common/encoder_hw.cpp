#include <px4_platform_common/encoder_hw.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>
#include <chip.h>
#include <stm32_gpio.h>

// Pin event queue size (power of 2 for efficient circular buffer)
#define PIN_EVENT_QUEUE_SIZE 32
#define PIN_EVENT_QUEUE_MASK (PIN_EVENT_QUEUE_SIZE - 1)

// Velocity calculation constants
#define VELOCITY_BUFFER_SIZE 10
#define VELOCITY_WINDOW_S 0.1f
#define MIN_VELOCITY_UPDATE_S 0.001f

// Velocity sample structure
typedef struct {
	uint64_t timestamp;
	int64_t position;
} velocity_sample_t;

// Per-encoder processing state
typedef struct {
	// Quadrature state
	uint8_t last_ab_state;
	int64_t position_raw;
	int64_t position_offset;

	// Velocity calculation
	velocity_sample_t velocity_buffer[VELOCITY_BUFFER_SIZE];
	uint8_t velocity_buffer_head;
	uint64_t last_velocity_update_time;
	float velocity_rad_s;
	float velocity_rpm;

	// Direction and status
	bool direction_forward;
	bool index_detected_this_revolution;
	uint64_t last_index_time;

	// Counters
	uint64_t pulse_count;
	uint32_t direction_changes;
	uint32_t error_count;

	// Timing
	uint64_t last_update_time;

	// Thread-safe processed data access
	encoder_processed_data_t processed_data;  // Single buffer with semaphore protection
	px4_sem_t data_sem;                       // Semaphore for thread-safe access
	volatile bool data_updated;
} encoder_processing_state_t;

// GPIO to encoder mapping for O(1) ISR lookup
static uint8_t g_gpio_to_encoder_map[STM32H7_NGPIO];
static uint8_t g_gpio_to_pin_map[STM32H7_NGPIO];

// Encoder instance storage
static encoder_instance_t g_encoder_instances[ENCODER_MAX_INSTANCES];
static uint8_t g_active_encoder_count = 0;

// Per-encoder processing state
static encoder_processing_state_t g_processing_state[ENCODER_MAX_INSTANCES];

// Pin event circular buffer for ISR
static pin_event_t g_pin_event_queue[PIN_EVENT_QUEUE_SIZE];
static volatile uint32_t g_queue_head = 0;
static volatile uint32_t g_queue_tail = 0;
static px4_sem_t g_queue_sem;

// Event processor thread
static pthread_t g_processor_thread;
static volatile bool g_processor_running = false;
static volatile bool g_processor_should_exit = false;

// Event processor statistics
static encoder_event_processor_stats_t g_processor_stats;

// Work queue for deferred processing
static struct work_s g_encoder_work;

// Initialization flag
static bool g_encoder_hw_initialized = false;

// Forward declarations
static void *encoder_event_processor_thread(void *arg);
static void process_quadrature_event(uint8_t encoder_id, uint8_t pin_id, uint8_t pin_state, uint64_t timestamp);
static void update_encoder_velocity(uint8_t encoder_id, uint64_t timestamp);
static void update_processed_data(uint8_t encoder_id);
static void encoder_work_handler(void *arg);

/**
 * @brief Ultra-minimal universal ISR for all encoder pins
 *
 * This ISR does the absolute minimum:
 * 1. Capture timestamp
 * 2. Identify encoder and pin
 * 3. Read pin state
 * 4. Queue event
 * 5. Schedule work queue
 */
static int encoder_universal_isr(int irq, void *context, void *arg)
{
	uint32_t gpio_pin = (uint32_t)arg;
	uint8_t gpio_index = gpio_pin & 0xFF;

	// Fast O(1) lookup
	uint8_t encoder_id = g_gpio_to_encoder_map[gpio_index];
	uint8_t pin_id = g_gpio_to_pin_map[gpio_index];

	if (encoder_id == 0xFF) {
		return 0; // Invalid GPIO
	}

	// Capture timestamp as early as possible
	uint64_t timestamp = hrt_absolute_time();

	// Read current pin state
	bool pin_state = stm32_gpioread(gpio_pin);

	// Determine event type based on pin state
	pin_event_type_t event_type = pin_state ? PIN_EVENT_RISING : PIN_EVENT_FALLING;

	// Queue event (lockless circular buffer)
	uint32_t head = g_queue_head;
	uint32_t next_head = (head + 1) & PIN_EVENT_QUEUE_MASK;

	if (next_head != g_queue_tail) {
		g_pin_event_queue[head] = {
			.timestamp = timestamp,
			.encoder_id = encoder_id,
			.pin_id = pin_id,
			.event_type = (uint8_t)event_type,
			.pin_state = (uint8_t)pin_state
		};

		g_queue_head = next_head;
		g_encoder_instances[encoder_id].interrupt_count++;
	} else {
		// Queue full - increment error count
		g_encoder_instances[encoder_id].error_count++;
	}

	// Schedule work queue for deferred processing
	work_queue(LPWORK, &g_encoder_work, encoder_work_handler, nullptr, 0);

	return 0;
}

// Forward declaration
static void encoder_work_handler(void *arg);

/**
 * @brief Work queue handler for deferred encoder processing
 */
static void encoder_work_handler(void *arg)
{
	// Signal that pin events are available
	px4_sem_post(&g_queue_sem);
}

/**
 * @brief Configure GPIO for encoder pin
 */
static int configure_encoder_gpio(uint32_t gpio_pin, uint8_t encoder_id, uint8_t pin_id)
{
	// Configure GPIO as input with pull-up
	stm32_configgpio(gpio_pin | GPIO_PULLUP);

	// Set up interrupt on both edges
	int ret = stm32_gpiosetevent(gpio_pin, true, true, true, encoder_universal_isr, (void *)gpio_pin);
	if (ret < 0) {
		return ret;
	}

	// Set up fast lookup mapping
	uint8_t gpio_index = gpio_pin & 0xFF;
	g_gpio_to_encoder_map[gpio_index] = encoder_id;
	g_gpio_to_pin_map[gpio_index] = pin_id;

	return 0;
}

/**
 * @brief Remove GPIO configuration for encoder pin
 */
static void unconfigure_encoder_gpio(uint32_t gpio_pin)
{
	// Remove interrupt
	stm32_gpiosetevent(gpio_pin, false, false, false, nullptr, nullptr);

	// Clear lookup mapping
	uint8_t gpio_index = gpio_pin & 0xFF;
	g_gpio_to_encoder_map[gpio_index] = 0xFF;
	g_gpio_to_pin_map[gpio_index] = 0xFF;
}

// Public API implementation
int encoder_hw_init(void)
{
	if (g_encoder_hw_initialized) {
		return 0; // Already initialized
	}

	// Initialize GPIO lookup tables
	for (int i = 0; i < STM32H7_NGPIO; i++) {
		g_gpio_to_encoder_map[i] = 0xFF;
		g_gpio_to_pin_map[i] = 0xFF;
	}

	// Initialize encoder instances
	for (int i = 0; i < ENCODER_MAX_INSTANCES; i++) {
		g_encoder_instances[i].encoder_id = i;
		g_encoder_instances[i].is_initialized = false;
		g_encoder_instances[i].is_active = false;
		g_encoder_instances[i].interrupt_count = 0;
		g_encoder_instances[i].error_count = 0;
	}

	// Initialize queue
	g_queue_head = 0;
	g_queue_tail = 0;
	px4_sem_init(&g_queue_sem, 0, 0);

	g_active_encoder_count = 0;
	g_encoder_hw_initialized = true;

	return 0;
}

int encoder_hw_create_instance(const encoder_hw_config_t *config)
{
	if (!g_encoder_hw_initialized) {
		return -1;
	}

	if (!config) {
		return -2;
	}

	// Find available instance
	int encoder_id = -1;
	for (int i = 0; i < ENCODER_MAX_INSTANCES; i++) {
		if (!g_encoder_instances[i].is_initialized) {
			encoder_id = i;
			break;
		}
	}

	if (encoder_id < 0) {
		return -3; // No available instances
	}

	encoder_instance_t *instance = &g_encoder_instances[encoder_id];

	// Store configuration
	instance->config = *config;
	instance->gpio_a_pin = config->gpio_a;
	instance->gpio_b_pin = config->gpio_b;
	instance->gpio_index_pin = config->gpio_index;
	instance->is_initialized = true;
	instance->is_active = false;
	instance->interrupt_count = 0;
	instance->error_count = 0;

	return encoder_id;
}

int encoder_hw_start(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return -1;
	}

	encoder_instance_t *instance = &g_encoder_instances[encoder_id];

	if (!instance->is_initialized) {
		return -2;
	}

	if (instance->is_active) {
		return 0; // Already active
	}

	// Configure GPIO pins
	int ret = configure_encoder_gpio(instance->gpio_a_pin, encoder_id, 0);
	if (ret < 0) {
		return ret;
	}

	ret = configure_encoder_gpio(instance->gpio_b_pin, encoder_id, 1);
	if (ret < 0) {
		unconfigure_encoder_gpio(instance->gpio_a_pin);
		return ret;
	}

	// Configure index pin if enabled
	if (instance->config.enable_index && instance->gpio_index_pin != 0) {
		ret = configure_encoder_gpio(instance->gpio_index_pin, encoder_id, 2);
		if (ret < 0) {
			unconfigure_encoder_gpio(instance->gpio_a_pin);
			unconfigure_encoder_gpio(instance->gpio_b_pin);
			return ret;
		}
	}

	instance->is_active = true;
	g_active_encoder_count++;

	return 0;
}

int encoder_hw_stop(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return -1;
	}

	encoder_instance_t *instance = &g_encoder_instances[encoder_id];

	if (!instance->is_active) {
		return 0; // Already stopped
	}

	// Remove GPIO configurations
	unconfigure_encoder_gpio(instance->gpio_a_pin);
	unconfigure_encoder_gpio(instance->gpio_b_pin);

	if (instance->config.enable_index && instance->gpio_index_pin != 0) {
		unconfigure_encoder_gpio(instance->gpio_index_pin);
	}

	instance->is_active = false;
	g_active_encoder_count--;

	return 0;
}

int encoder_hw_destroy(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return -1;
	}

	encoder_instance_t *instance = &g_encoder_instances[encoder_id];

	if (!instance->is_initialized) {
		return -2;
	}

	// Stop if active
	if (instance->is_active) {
		encoder_hw_stop(encoder_id);
	}

	// Clear instance
	instance->is_initialized = false;
	instance->is_active = false;

	return 0;
}

int encoder_hw_get_instance(uint8_t encoder_id, encoder_instance_t *instance)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES || !instance) {
		return -1;
	}

	if (!g_encoder_instances[encoder_id].is_initialized) {
		return -2;
	}

	*instance = g_encoder_instances[encoder_id];
	return 0;
}

bool encoder_hw_get_pin_event(pin_event_t *event)
{
	if (!event) {
		return false;
	}

	// Check if events are available
	if (g_queue_tail == g_queue_head) {
		return false;
	}

	// Get event from tail
	*event = g_pin_event_queue[g_queue_tail];
	g_queue_tail = (g_queue_tail + 1) & PIN_EVENT_QUEUE_MASK;

	return true;
}

uint8_t encoder_hw_get_active_count(void)
{
	return g_active_encoder_count;
}

/**
 * @brief Dedicated event processor thread
 *
 * This thread processes all pin events for all encoders in a centralized manner.
 * It waits on the semaphore for new events and processes them immediately.
 */
static void *encoder_event_processor_thread(void *arg)
{
	g_processor_running = true;

	while (!g_processor_should_exit) {
		// Wait for pin events to be available
		if (px4_sem_wait(&g_queue_sem) != 0) {
			continue;
		}

		if (g_processor_should_exit) {
			break;
		}

		uint64_t processing_start = hrt_absolute_time();
		uint32_t events_processed = 0;

		// Process all available pin events
		pin_event_t event;
		while (encoder_hw_get_pin_event(&event)) {
			process_quadrature_event(event.encoder_id, event.pin_id, event.pin_state, event.timestamp);
			events_processed++;
		}

		// Update velocities and processed data for all active encoders
		uint64_t current_time = hrt_absolute_time();
		for (uint8_t i = 0; i < ENCODER_MAX_INSTANCES; i++) {
			if (g_encoder_instances[i].is_active) {
				update_encoder_velocity(i, current_time);
				update_processed_data(i);
			}
		}

		// Update statistics
		uint64_t processing_end = hrt_absolute_time();
		uint32_t processing_time = (uint32_t)(processing_end - processing_start);

		g_processor_stats.events_processed += events_processed;
		g_processor_stats.processing_time_us = processing_time;
		if (processing_time > g_processor_stats.max_processing_time_us) {
			g_processor_stats.max_processing_time_us = processing_time;
		}

		// Calculate queue utilization
		uint32_t queue_used = (g_queue_head - g_queue_tail) & PIN_EVENT_QUEUE_MASK;
		g_processor_stats.queue_utilization_percent = (queue_used * 100) / PIN_EVENT_QUEUE_SIZE;
	}

	g_processor_running = false;
	return nullptr;
}

/**
 * @brief Process a quadrature encoder event
 */
static void process_quadrature_event(uint8_t encoder_id, uint8_t pin_id, uint8_t pin_state, uint64_t timestamp)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return;
	}

	encoder_processing_state_t *state = &g_processing_state[encoder_id];
	encoder_instance_t *instance = &g_encoder_instances[encoder_id];

	// Handle index pulse separately
	if (pin_id == 2) {
		if (pin_state == 1) { // Rising edge of index
			if (instance->config.mode == ENCODER_MODE_ABSOLUTE) {
				state->position_raw = 0;
				state->position_offset = 0;
			}
			state->index_detected_this_revolution = true;
			state->last_index_time = timestamp;
		}
		return;
	}

	// Build current AB state
	uint8_t current_ab_state = state->last_ab_state;

	if (pin_id == 0) { // Channel A
		current_ab_state = (current_ab_state & 0x02) | (pin_state & 0x01);
	} else if (pin_id == 1) { // Channel B
		current_ab_state = (current_ab_state & 0x01) | ((pin_state & 0x01) << 1);
	}

	// Apply channel swapping if configured
	if (instance->config.swap_channels) {
		current_ab_state = ((current_ab_state & 0x01) << 1) | ((current_ab_state & 0x02) >> 1);
	}

	// Quadrature decoding state machine
	int8_t direction = 0;

	switch ((state->last_ab_state << 2) | current_ab_state) {
		case 0x01: case 0x07: case 0x08: case 0x0E: // Forward
			direction = 1;
			break;
		case 0x02: case 0x04: case 0x0B: case 0x0D: // Reverse
			direction = -1;
			break;
		case 0x03: case 0x06: case 0x09: case 0x0C: // Invalid transition
			state->error_count++;
			instance->error_count++;
			break;
		default: // No change
			break;
	}

	// Update position
	if (direction != 0) {
		state->position_raw += direction;
		state->pulse_count++;

		// Check for direction change
		bool new_direction = (direction > 0);
		if (new_direction != state->direction_forward) {
			state->direction_forward = new_direction;
			state->direction_changes++;
		}

		// Add velocity sample
		uint8_t next_head = (state->velocity_buffer_head + 1) % VELOCITY_BUFFER_SIZE;
		state->velocity_buffer[state->velocity_buffer_head].timestamp = timestamp;
		state->velocity_buffer[state->velocity_buffer_head].position = state->position_raw;
		state->velocity_buffer_head = next_head;

		state->last_update_time = timestamp;
		state->data_updated = true;
	}

	state->last_ab_state = current_ab_state;
}

/**
 * @brief Update velocity calculation for an encoder
 */
static void update_encoder_velocity(uint8_t encoder_id, uint64_t timestamp)
{
	encoder_processing_state_t *state = &g_processing_state[encoder_id];
	encoder_instance_t *instance = &g_encoder_instances[encoder_id];

	// Don't update velocity too frequently
	if (timestamp - state->last_velocity_update_time < (uint64_t)(MIN_VELOCITY_UPDATE_S * 1e6f)) {
		return;
	}

	// Find the oldest sample within the velocity window
	uint64_t window_start = timestamp - (uint64_t)(VELOCITY_WINDOW_S * 1e6f);

	int oldest_valid_index = -1;
	uint8_t current_index = state->velocity_buffer_head;

	for (int i = 0; i < VELOCITY_BUFFER_SIZE; i++) {
		current_index = (current_index + VELOCITY_BUFFER_SIZE - 1) % VELOCITY_BUFFER_SIZE;

		if (state->velocity_buffer[current_index].timestamp >= window_start) {
			oldest_valid_index = current_index;
		} else {
			break;
		}
	}

	// Calculate velocity if we have valid samples
	if (oldest_valid_index >= 0) {
		uint8_t newest_index = (state->velocity_buffer_head + VELOCITY_BUFFER_SIZE - 1) % VELOCITY_BUFFER_SIZE;

		int64_t delta_position = state->velocity_buffer[newest_index].position - state->velocity_buffer[oldest_valid_index].position;
		uint64_t delta_time = state->velocity_buffer[newest_index].timestamp - state->velocity_buffer[oldest_valid_index].timestamp;

				if (delta_time > 0) {
			// Calculate angular velocity in rad/s
			float position_per_pulse = 2.0f * M_PI_F / (float)instance->config.pulses_per_revolution;
			float velocity_rad_s = (float)delta_position * position_per_pulse / ((float)delta_time * 1e-6f);

			// Store velocity values (will be copied to processed data in update_processed_data)
			state->velocity_rad_s = velocity_rad_s;
			state->velocity_rpm = velocity_rad_s * 60.0f / (2.0f * M_PI_F);
		}
	}

	state->last_velocity_update_time = timestamp;
}

/**
 * @brief Update processed data structure for an encoder
 */
static void update_processed_data(uint8_t encoder_id)
{
	encoder_processing_state_t *state = &g_processing_state[encoder_id];
	encoder_instance_t *instance = &g_encoder_instances[encoder_id];

	// Wait for semaphore to be available
	if (px4_sem_wait(&state->data_sem) != 0) {
		return; // Semaphore not available, try again later
	}

	// Update processed data structure
	state->processed_data.timestamp = hrt_absolute_time();
	state->processed_data.encoder_id = encoder_id;

	// Position data
	int64_t effective_position = state->position_raw + state->position_offset;
	state->processed_data.position_raw = state->position_raw;
	state->processed_data.position_rad = (float)effective_position * 2.0f * M_PI_F / (float)instance->config.pulses_per_revolution;
	state->processed_data.position_deg = state->processed_data.position_rad * 180.0f / M_PI_F;

	// Velocity data
	state->processed_data.velocity_rad_s = state->velocity_rad_s;
	state->processed_data.velocity_rpm = state->velocity_rpm;
	state->processed_data.velocity_updated = true;

	// Direction and status
	state->processed_data.direction_forward = state->direction_forward;
	state->processed_data.index_detected = state->index_detected_this_revolution;

	// Counters
	state->processed_data.pulse_count = state->pulse_count;
	state->processed_data.direction_changes = state->direction_changes;
	state->processed_data.error_count = state->error_count;

	// Validity flags
	state->processed_data.data_valid = true;
	state->processed_data.position_updated = state->data_updated;

	// Signal that data is ready for reading
	px4_sem_post(&state->data_sem);

	// Reset update flags
	state->index_detected_this_revolution = false;
	state->data_updated = false;
}

// Public API implementation for event processor thread
int encoder_hw_start_event_processor(void)
{
	if (g_processor_running) {
		return 0; // Already running
	}

	if (!g_encoder_hw_initialized) {
		return -1; // Hardware not initialized
	}

	// Initialize processing state for all encoders
	for (int i = 0; i < ENCODER_MAX_INSTANCES; i++) {
		encoder_processing_state_t *state = &g_processing_state[i];
		memset(state, 0, sizeof(*state));
		state->direction_forward = true;
		state->last_ab_state = 0;
		state->velocity_rad_s = 0.0f;
		state->velocity_rpm = 0.0f;
		state->processed_data.encoder_id = i;
		px4_sem_init(&state->data_sem, 0, 1); // Initialize semaphore (binary semaphore)
		state->data_updated = false;
	}

	// Initialize statistics
	memset(&g_processor_stats, 0, sizeof(g_processor_stats));

	// Create the processor thread
	g_processor_should_exit = false;

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 2048);

	int ret = pthread_create(&g_processor_thread, &attr, encoder_event_processor_thread, nullptr);
	pthread_attr_destroy(&attr);

	if (ret != 0) {
		return -2;
	}

	// Wait for thread to start
	usleep(1000); // 1ms

	return 0;
}

int encoder_hw_stop_event_processor(void)
{
	if (!g_processor_running) {
		return 0; // Already stopped
	}

	// Signal thread to exit
	g_processor_should_exit = true;

	// Wake up the thread
	px4_sem_post(&g_queue_sem);

	// Wait for thread to exit
	pthread_join(g_processor_thread, nullptr);

	// Clean up semaphores for all encoders
	for (int i = 0; i < ENCODER_MAX_INSTANCES; i++) {
		px4_sem_destroy(&g_processing_state[i].data_sem);
	}

	return 0;
}

bool encoder_hw_get_processed_data(uint8_t encoder_id, encoder_processed_data_t *data)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES || !data) {
		return false;
	}

	if (!g_encoder_instances[encoder_id].is_active) {
		return false;
	}

	encoder_processing_state_t *state = &g_processing_state[encoder_id];

	// Wait for semaphore to be available
	if (px4_sem_wait(&state->data_sem) != 0) {
		return false; // Semaphore not available, try again later
	}

	// Copy the stable buffer data
	*data = state->processed_data;

	// Signal that data is no longer needed
	px4_sem_post(&state->data_sem);

	return data->data_valid;
}

int encoder_hw_get_processor_stats(encoder_event_processor_stats_t *stats)
{
	if (!stats) {
		return -1;
	}

	*stats = g_processor_stats;
	return 0;
}

bool encoder_hw_is_processor_running(void)
{
	return g_processor_running;
}
