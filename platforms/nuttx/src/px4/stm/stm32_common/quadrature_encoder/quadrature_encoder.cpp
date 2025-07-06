/****************************************************************************
 *
 * Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file quadrature_encoder.cpp
 *
 * STM32 Quadrature Encoder Driver for NuttX/STM32 platforms.
 *
 * This driver provides a robust, multi-instance quadrature encoder interface with:
 * - Ultra-minimal ISR for real-time performance
 * - Work queue-based GPIO-to-encoder mapping resolution
 * - Dedicated event processor thread for encoder state management
 * - Thread-safe data access with semaphore protection
 * - Comprehensive velocity calculation and statistics
 *
 * @author PX4 Development Team
 */

#include <px4_arch/quadrature_encoder.h>

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

#include <cstring>
#include <cmath>
#include <cerrno>

/**
 * @defgroup qencoder_hal Quadrature Encoder Constants and Types
 * @{
 */

/** Pin event queue size (power of 2 for efficient circular buffer) */
static constexpr size_t PIN_EVENT_QUEUE_SIZE = 32;
static constexpr size_t PIN_EVENT_QUEUE_MASK = (PIN_EVENT_QUEUE_SIZE - 1);

/** Velocity calculation constants */
static constexpr size_t VELOCITY_BUFFER_SIZE = 10;
static constexpr float VELOCITY_WINDOW_S = 0.1f;
static constexpr float MIN_VELOCITY_UPDATE_S = 0.001f;

/** GPIO pin type identifiers */
static constexpr uint8_t PIN_TYPE_PHASE_A = 0;
static constexpr uint8_t PIN_TYPE_PHASE_B = 1;
static constexpr uint8_t PIN_TYPE_INDEX = 2;

/** @} */

/**
 * @defgroup qencoder_hal_types Quadrature Encoder Data Types
 * @{
 */

/** Raw GPIO event structure (minimal data captured in ISR) */
struct raw_gpio_event_t {
	uint64_t timestamp;
	uint32_t gpio_pin;
	uint8_t pin_state;
};

/** Processed encoder event structure (handled in work queue) */
struct encoder_event_t {
	uint64_t timestamp;
	uint8_t encoder_id;
	uint8_t pin_type;  ///< PIN_TYPE_PHASE_A, PIN_TYPE_PHASE_B, or PIN_TYPE_INDEX
	uint8_t pin_state;
};

/** Velocity sample structure for time-based velocity calculation */
struct velocity_sample_t {
	uint64_t timestamp;
	int64_t position;
};

/** @} */

/**
 * @brief Per-encoder processing state
 *
 * This structure holds all the state information for processing one encoder
 * instance, including quadrature decoding, velocity calculation, and statistics.
 */
struct encoder_processing_state_t {
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
	encoder_processed_data_t processed_data;  ///< Single buffer with semaphore protection
	px4_sem_t data_sem;                       ///< Semaphore for thread-safe access
	volatile bool data_updated;
};

/**
 * @brief GPIO-to-encoder mapping structure for work queue processing
 *
 * This structure maps GPIO pins to their corresponding encoder instances
 * and pin types. Used by the work queue to resolve raw GPIO events.
 */
struct gpio_encoder_mapping_t {
	uint32_t gpio_pin;      ///< Full GPIO pin definition
	uint8_t encoder_id;     ///< Encoder instance ID
	uint8_t pin_type;       ///< PIN_TYPE_PHASE_A, PIN_TYPE_PHASE_B, or PIN_TYPE_INDEX
	bool is_active;         ///< Whether this mapping is active
};

/**
 * @defgroup qencoder_hal_globals Quadrature Encoder Global State
 * @{
 */

/** Active GPIO mappings (only stores active pins, memory efficient) */
static gpio_encoder_mapping_t g_gpio_mappings[ENCODER_MAX_INSTANCES * 3]; ///< Max 3 pins per encoder
static uint8_t g_active_mapping_count = 0;

/** Encoder instance storage */
static encoder_instance_t g_encoder_instances[ENCODER_MAX_INSTANCES];
static uint8_t g_active_encoder_count = 0;

/** Per-encoder processing state */
static encoder_processing_state_t g_processing_state[ENCODER_MAX_INSTANCES];

/** Pin event circular buffer for ISR (raw GPIO events) */
static raw_gpio_event_t g_raw_pin_event_queue[PIN_EVENT_QUEUE_SIZE];
static volatile uint32_t g_raw_queue_head = 0;
static volatile uint32_t g_raw_queue_tail = 0;

/** Processed pin event circular buffer for event processor thread */
static encoder_event_t g_processed_pin_event_queue[PIN_EVENT_QUEUE_SIZE];
static volatile uint32_t g_processed_queue_head = 0;
static volatile uint32_t g_processed_queue_tail = 0;
static px4_sem_t g_queue_sem;

/** Event processor thread */
static pthread_t g_processor_thread;
static volatile bool g_processor_running = false;
static volatile bool g_processor_should_exit = false;

/** Event processor statistics */
static encoder_event_processor_stats_t g_processor_stats;

/** Work queue for deferred processing */
static struct work_s g_encoder_work;

/** Initialization flag */
static bool g_qencoder_initialized = false;

/** @} */

/**
 * @defgroup qencoder_hal_internal Internal Function Declarations
 * @{
 */

static void *encoder_event_processor_thread(void *arg);
static void process_quadrature_event(uint8_t encoder_id, uint8_t pin_id, uint8_t pin_state, uint64_t timestamp);
static void update_encoder_velocity(uint8_t encoder_id, uint64_t timestamp);
static void update_processed_data(uint8_t encoder_id);
static void encoder_work_handler(void *arg);
static int add_gpio_mapping(uint32_t gpio_pin, uint8_t encoder_id, uint8_t pin_type);
static void remove_gpio_mapping(uint32_t gpio_pin);
static int configure_encoder_gpio(uint32_t gpio_pin, uint8_t encoder_id, uint8_t pin_type);
static void unconfigure_encoder_gpio(uint32_t gpio_pin);

/** @} */

/**
 * @defgroup qencoder_hal_isr Interrupt Service Routine
 * @{
 */

/**
 * @brief Ultra-minimal universal ISR for all encoder pins
 *
 * This ISR performs the absolute minimum work to maintain real-time performance:
 * 1. Capture high-resolution timestamp
 * 2. Read GPIO pin state
 * 3. Queue raw event in lockless circular buffer
 * 4. Schedule work queue for deferred processing
 *
 * @param irq Interrupt request number
 * @param context Interrupt context (unused)
 * @param arg GPIO pin identifier passed as argument
 * @return 0 on success
 */
static int encoder_universal_isr(int irq, void *context, void *arg)
{
	const uint32_t gpio_pin = reinterpret_cast<uint32_t>(arg);

	// Capture timestamp as early as possible for accuracy
	const uint64_t timestamp = hrt_absolute_time();

	// Read current pin state
	const bool pin_state = stm32_gpioread(gpio_pin);

	// Queue raw event using lockless circular buffer
	const uint32_t head = g_raw_queue_head;
	const uint32_t next_head = (head + 1) & PIN_EVENT_QUEUE_MASK;

	if (next_head != g_raw_queue_tail) {
		g_raw_pin_event_queue[head] = {
			.timestamp = timestamp,
			.gpio_pin = gpio_pin,
			.pin_state = static_cast<uint8_t>(pin_state)
		};

		g_raw_queue_head = next_head;
	}

	// Schedule work queue for deferred processing
	work_queue(LPWORK, &g_encoder_work, encoder_work_handler, nullptr, 0);

	return 0;
}

/** @} */

/**
 * @defgroup qencoder_hal_work_queue Work Queue Handler
 * @{
 */

/**
 * @brief Work queue handler for deferred encoder processing
 *
 * This handler runs outside the ISR context and performs the GPIO-to-encoder
 * mapping lookups, then queues the resolved events for the processor thread.
 * This separation keeps the ISR ultra-minimal while allowing more complex
 * mapping resolution to occur in a less time-critical context.
 *
 * @param arg Unused work queue argument
 */
static void encoder_work_handler(void *arg)
{
	bool has_valid_events = false;

	// Process all raw events from ISR and resolve GPIO mappings
	while (g_raw_queue_tail != g_raw_queue_head) {
		// Get raw event from ISR queue
		const raw_gpio_event_t raw_event = g_raw_pin_event_queue[g_raw_queue_tail];
		g_raw_queue_tail = (g_raw_queue_tail + 1) & PIN_EVENT_QUEUE_MASK;

		// Find the encoder mapping for this GPIO pin
		gpio_encoder_mapping_t *mapping = nullptr;

		for (uint8_t i = 0; i < g_active_mapping_count; i++) {
			if (g_gpio_mappings[i].is_active && g_gpio_mappings[i].gpio_pin == raw_event.gpio_pin) {
				mapping = &g_gpio_mappings[i];
				break;
			}
		}

		// Skip invalid GPIO events
		if (mapping == nullptr) {
			continue;
		}

		// Create resolved event for processor thread
		const encoder_event_t resolved_event = {
			.timestamp = raw_event.timestamp,
			.encoder_id = mapping->encoder_id,
			.pin_type = mapping->pin_type,
			.pin_state = raw_event.pin_state
		};

		// Queue resolved event for processor thread
		const uint32_t head = g_processed_queue_head;
		const uint32_t next_head = (head + 1) & PIN_EVENT_QUEUE_MASK;

		if (next_head != g_processed_queue_tail) {
			g_processed_pin_event_queue[head] = resolved_event;
			g_processed_queue_head = next_head;
			g_encoder_instances[mapping->encoder_id].interrupt_count++;
			has_valid_events = true;
		} else {
			// Processed queue full - increment error count
			g_encoder_instances[mapping->encoder_id].error_count++;
		}
	}

	// Signal processor thread only if we have valid events
	if (has_valid_events) {
		px4_sem_post(&g_queue_sem);
	}
}

/** @} */

/**
 * @defgroup qencoder_hal_gpio_mgmt GPIO Management Functions
 * @{
 */

/**
 * @brief Add a GPIO-to-encoder mapping
 *
 * @param gpio_pin GPIO pin definition
 * @param encoder_id Encoder instance ID
 * @param pin_type Pin type (PIN_TYPE_PHASE_A, PIN_TYPE_PHASE_B, or PIN_TYPE_INDEX)
 * @return 0 on success, negative error code on failure
 */
static int add_gpio_mapping(uint32_t gpio_pin, uint8_t encoder_id, uint8_t pin_type)
{
	// Check if we have space for another mapping
	if (g_active_mapping_count >= (ENCODER_MAX_INSTANCES * 3)) {
		return -ENOSPC; // No space available
	}

	// Find an available slot
	for (uint8_t i = 0; i < (ENCODER_MAX_INSTANCES * 3); i++) {
		if (!g_gpio_mappings[i].is_active) {
			g_gpio_mappings[i].gpio_pin = gpio_pin;
			g_gpio_mappings[i].encoder_id = encoder_id;
			g_gpio_mappings[i].pin_type = pin_type;
			g_gpio_mappings[i].is_active = true;
			g_active_mapping_count++;
			return 0;
		}
	}

	return -ENOENT; // No available slot found
}

/**
 * @brief Remove a GPIO-to-encoder mapping
 *
 * @param gpio_pin GPIO pin definition to remove
 */
static void remove_gpio_mapping(uint32_t gpio_pin)
{
	for (uint8_t i = 0; i < (ENCODER_MAX_INSTANCES * 3); i++) {
		if (g_gpio_mappings[i].is_active && g_gpio_mappings[i].gpio_pin == gpio_pin) {
			g_gpio_mappings[i].is_active = false;
			g_active_mapping_count--;
			break;
		}
	}
}

/**
 * @brief Configure GPIO for encoder pin
 *
 * @param gpio_pin GPIO pin definition
 * @param encoder_id Encoder instance ID
 * @param pin_type Pin type (PIN_TYPE_PHASE_A, PIN_TYPE_PHASE_B, or PIN_TYPE_INDEX)
 * @return 0 on success, negative error code on failure
 */
static int configure_encoder_gpio(uint32_t gpio_pin, uint8_t encoder_id, uint8_t pin_type)
{
	// Configure GPIO as input with pull-up
	stm32_configgpio(gpio_pin | GPIO_PULLUP);

	// Set up interrupt on both edges
	int ret = stm32_gpiosetevent(gpio_pin, true, true, true, encoder_universal_isr, reinterpret_cast<void *>(gpio_pin));

	if (ret < 0) {
		return ret;
	}

	// Add GPIO mapping
	ret = add_gpio_mapping(gpio_pin, encoder_id, pin_type);

	if (ret < 0) {
		// Clean up GPIO configuration on mapping failure
		stm32_gpiosetevent(gpio_pin, false, false, false, nullptr, nullptr);
		return ret;
	}

	return 0;
}

/**
 * @brief Remove GPIO configuration for encoder pin
 *
 * @param gpio_pin GPIO pin definition
 */
static void unconfigure_encoder_gpio(uint32_t gpio_pin)
{
	// Remove interrupt
	stm32_gpiosetevent(gpio_pin, false, false, false, nullptr, nullptr);

	// Remove GPIO mapping
	remove_gpio_mapping(gpio_pin);
}

/** @} */

/**
 * @defgroup qencoder_hal_public_api Public API Implementation
 * @{
 */

/**
 * @brief Initialize the quadrature encoder driver
 *
 * This function initializes all global state, data structures, and semaphores
 * required for the encoder driver to function. Must be called before any other
 * driver functions.
 *
 * @return 0 on success, negative error code on failure
 */
int qencoder_init(void)
{
	if (g_qencoder_initialized) {
		return 0; // Already initialized
	}

	// Initialize GPIO mapping array
	for (uint8_t i = 0; i < (ENCODER_MAX_INSTANCES * 3); i++) {
		g_gpio_mappings[i] = {
			.gpio_pin = 0,
			.encoder_id = 0xFF,
			.pin_type = 0xFF,
			.is_active = false
		};
	}

	g_active_mapping_count = 0;

	// Initialize encoder instances
	for (uint8_t i = 0; i < ENCODER_MAX_INSTANCES; i++) {
		encoder_instance_t &instance = g_encoder_instances[i];
		instance.encoder_id = i;
		instance.is_initialized = false;
		instance.is_active = false;
		instance.interrupt_count = 0;
		instance.error_count = 0;
	}

	// Initialize queues
	g_raw_queue_head = 0;
	g_raw_queue_tail = 0;
	g_processed_queue_head = 0;
	g_processed_queue_tail = 0;

	const int sem_ret = px4_sem_init(&g_queue_sem, 0, 0);

	if (sem_ret != 0) {
		return -EIO; // Semaphore initialization failed
	}

	g_active_encoder_count = 0;
	g_qencoder_initialized = true;

	return 0;
}

/**
 * @brief Create a new encoder instance
 *
 * @param config Encoder configuration structure
 * @return Encoder ID on success, negative error code on failure
 */
int qencoder_create_instance(const quadrature_encoder_config_t *config)
{
	if (!g_qencoder_initialized) {
		return -ENOTCONN; // Driver not initialized
	}

	if (config == nullptr) {
		return -EINVAL; // Invalid configuration
	}

	// Find available instance
	int encoder_id = -1;

	for (uint8_t i = 0; i < ENCODER_MAX_INSTANCES; i++) {
		if (!g_encoder_instances[i].is_initialized) {
			encoder_id = i;
			break;
		}
	}

	if (encoder_id < 0) {
		return -ENOSPC; // No available instances
	}

	encoder_instance_t &instance = g_encoder_instances[encoder_id];

	// Store configuration
	instance.config = *config;
	instance.gpio_a_pin = config->gpio_a;
	instance.gpio_b_pin = config->gpio_b;
	instance.gpio_index_pin = config->gpio_index;
	instance.is_initialized = true;
	instance.is_active = false;
	instance.interrupt_count = 0;
	instance.error_count = 0;

	return encoder_id;
}

/**
 * @brief Start an encoder instance
 *
 * @param encoder_id Encoder instance ID
 * @return 0 on success, negative error code on failure
 */
int qencoder_start(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return -EINVAL; // Invalid encoder ID
	}

	encoder_instance_t &instance = g_encoder_instances[encoder_id];

	if (!instance.is_initialized) {
		return -ENOTCONN; // Instance not initialized
	}

	if (instance.is_active) {
		return 0; // Already active
	}

	// Configure GPIO pins
	int ret = configure_encoder_gpio(instance.gpio_a_pin, encoder_id, PIN_TYPE_PHASE_A);

	if (ret < 0) {
		return ret;
	}

	ret = configure_encoder_gpio(instance.gpio_b_pin, encoder_id, PIN_TYPE_PHASE_B);

	if (ret < 0) {
		unconfigure_encoder_gpio(instance.gpio_a_pin);
		return ret;
	}

	// Configure index pin if enabled
	if (instance.config.enable_index && instance.gpio_index_pin != 0) {
		ret = configure_encoder_gpio(instance.gpio_index_pin, encoder_id, PIN_TYPE_INDEX);

		if (ret < 0) {
			unconfigure_encoder_gpio(instance.gpio_a_pin);
			unconfigure_encoder_gpio(instance.gpio_b_pin);
			return ret;
		}
	}

	instance.is_active = true;
	g_active_encoder_count++;

	return 0;
}

/**
 * @brief Stop an encoder instance
 *
 * @param encoder_id Encoder instance ID
 * @return 0 on success, negative error code on failure
 */
int qencoder_stop(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return -EINVAL; // Invalid encoder ID
	}

	encoder_instance_t &instance = g_encoder_instances[encoder_id];

	if (!instance.is_active) {
		return 0; // Already stopped
	}

	// Remove GPIO configurations
	unconfigure_encoder_gpio(instance.gpio_a_pin);
	unconfigure_encoder_gpio(instance.gpio_b_pin);

	if (instance.config.enable_index && instance.gpio_index_pin != 0) {
		unconfigure_encoder_gpio(instance.gpio_index_pin);
	}

	instance.is_active = false;
	g_active_encoder_count--;

	return 0;
}

/**
 * @brief Destroy an encoder instance
 *
 * @param encoder_id Encoder instance ID
 * @return 0 on success, negative error code on failure
 */
int qencoder_destroy(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return -EINVAL; // Invalid encoder ID
	}

	encoder_instance_t &instance = g_encoder_instances[encoder_id];

	if (!instance.is_initialized) {
		return -ENOTCONN; // Instance not initialized
	}

	// Stop if active
	if (instance.is_active) {
		qencoder_stop(encoder_id);
	}

	// Clear instance
	instance.is_initialized = false;
	instance.is_active = false;

	return 0;
}

/**
 * @brief Get encoder instance information
 *
 * @param encoder_id Encoder instance ID
 * @param instance Pointer to store instance information
 * @return 0 on success, negative error code on failure
 */
int qencoder_get_instance(uint8_t encoder_id, encoder_instance_t *instance)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES || instance == nullptr) {
		return -EINVAL; // Invalid parameters
	}

	if (!g_encoder_instances[encoder_id].is_initialized) {
		return -ENOTCONN; // Instance not initialized
	}

	*instance = g_encoder_instances[encoder_id];
	return 0;
}

/**
 * @brief Get processed pin event from queue
 *
 * @param event Pointer to store event information
 * @return true if event was available, false if queue is empty
 */
bool qencoder_get_pin_event(pin_event_t *event)
{
	if (event == nullptr) {
		return false;
	}

	// Check if processed events are available
	if (g_processed_queue_tail == g_processed_queue_head) {
		return false;
	}

	// Get processed event from tail
	const encoder_event_t processed_event = g_processed_pin_event_queue[g_processed_queue_tail];
	g_processed_queue_tail = (g_processed_queue_tail + 1) & PIN_EVENT_QUEUE_MASK;

	// Convert to pin_event_t format for compatibility
	event->timestamp = processed_event.timestamp;
	event->encoder_id = processed_event.encoder_id;
	event->pin_id = processed_event.pin_type;
	event->event_type = processed_event.pin_state ? PIN_EVENT_RISING : PIN_EVENT_FALLING;
	event->pin_state = processed_event.pin_state;

	return true;
}

/**
 * @brief Get the number of active encoder instances
 *
 * @return Number of active encoder instances
 */
uint8_t qencoder_get_active_count(void)
{
	return g_active_encoder_count;
}

/** @} */

/**
 * @defgroup qencoder_hal_event_processor Event Processor Thread
 * @{
 */

/**
 * @brief Dedicated event processor thread
 *
 * This thread processes all pin events for all encoders in a centralized manner.
 * It waits on the semaphore for new events and processes them immediately for
 * real-time performance. The thread performs:
 * 1. Event processing and quadrature decoding
 * 2. Velocity calculation
 * 3. Statistics tracking
 * 4. Data structure updates
 *
 * @param arg Thread argument (unused)
 * @return nullptr on thread exit
 */
static void *encoder_event_processor_thread(void *arg)
{
	(void)arg; // Unused parameter

	g_processor_running = true;

	while (!g_processor_should_exit) {
		// Wait for pin events to be available
		if (px4_sem_wait(&g_queue_sem) != 0) {
			continue;
		}

		if (g_processor_should_exit) {
			break;
		}

		const uint64_t processing_start = hrt_absolute_time();
		uint32_t events_processed = 0;

		// Process all available pin events
		pin_event_t event;

		while (qencoder_get_pin_event(&event)) {
			process_quadrature_event(event.encoder_id, event.pin_id, event.pin_state, event.timestamp);
			events_processed++;
		}

		// Update velocities and processed data for all active encoders
		const uint64_t current_time = hrt_absolute_time();

		for (uint8_t i = 0; i < ENCODER_MAX_INSTANCES; i++) {
			if (g_encoder_instances[i].is_active) {
				update_encoder_velocity(i, current_time);
				update_processed_data(i);
			}
		}

		// Update statistics
		const uint64_t processing_end = hrt_absolute_time();
		const uint32_t processing_time = static_cast<uint32_t>(processing_end - processing_start);

		g_processor_stats.events_processed += events_processed;
		g_processor_stats.processing_time_us = processing_time;

		if (processing_time > g_processor_stats.max_processing_time_us) {
			g_processor_stats.max_processing_time_us = processing_time;
		}

		// Calculate queue utilization (processed queue)
		const uint32_t queue_used = (g_processed_queue_head - g_processed_queue_tail) & PIN_EVENT_QUEUE_MASK;
		g_processor_stats.queue_utilization_percent = (queue_used * 100) / PIN_EVENT_QUEUE_SIZE;
	}

	g_processor_running = false;
	return nullptr;
}

/**
 * @brief Process a quadrature encoder event
 *
 * This function implements the quadrature decoding state machine and updates
 * encoder position, velocity samples, and statistics.
 *
 * @param encoder_id Encoder instance ID
 * @param pin_id Pin type (PIN_TYPE_PHASE_A, PIN_TYPE_PHASE_B, or PIN_TYPE_INDEX)
 * @param pin_state Current pin state (0 or 1)
 * @param timestamp Event timestamp
 */
static void process_quadrature_event(uint8_t encoder_id, uint8_t pin_id, uint8_t pin_state, uint64_t timestamp)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return;
	}

	encoder_processing_state_t &state = g_processing_state[encoder_id];
	const encoder_instance_t &instance = g_encoder_instances[encoder_id];

	// Handle index pulse separately
	if (pin_id == PIN_TYPE_INDEX) {
		if (pin_state == 1) { // Rising edge of index
			if (instance.config.mode == ENCODER_MODE_ABSOLUTE) {
				state.position_raw = 0;
				state.position_offset = 0;
			}

			state.index_detected_this_revolution = true;
			state.last_index_time = timestamp;
		}

		return;
	}

	// Build current AB state
	uint8_t current_ab_state = state.last_ab_state;

	if (pin_id == PIN_TYPE_PHASE_A) {
		current_ab_state = (current_ab_state & 0x02) | (pin_state & 0x01);
	} else if (pin_id == PIN_TYPE_PHASE_B) {
		current_ab_state = (current_ab_state & 0x01) | ((pin_state & 0x01) << 1);
	}

	// Apply channel swapping if configured
	if (instance.config.swap_channels) {
		current_ab_state = ((current_ab_state & 0x01) << 1) | ((current_ab_state & 0x02) >> 1);
	}

	// Quadrature decoding state machine
	int8_t direction = 0;

	switch ((state.last_ab_state << 2) | current_ab_state) {
	case 0x01: case 0x07: case 0x08: case 0x0E: // Forward
		direction = 1;
		break;

	case 0x02: case 0x04: case 0x0B: case 0x0D: // Reverse
		direction = -1;
		break;

	case 0x03: case 0x06: case 0x09: case 0x0C: // Invalid transition
		state.error_count++;
		g_encoder_instances[encoder_id].error_count++;
		break;

	default: // No change
		break;
	}

	// Update position
	if (direction != 0) {
		state.position_raw += direction;
		state.pulse_count++;

		// Check for direction change
		const bool new_direction = (direction > 0);

		if (new_direction != state.direction_forward) {
			state.direction_forward = new_direction;
			state.direction_changes++;
		}

		// Add velocity sample
		const uint8_t next_head = (state.velocity_buffer_head + 1) % VELOCITY_BUFFER_SIZE;
		state.velocity_buffer[state.velocity_buffer_head].timestamp = timestamp;
		state.velocity_buffer[state.velocity_buffer_head].position = state.position_raw;
		state.velocity_buffer_head = next_head;

		state.last_update_time = timestamp;
		state.data_updated = true;
	}

	state.last_ab_state = current_ab_state;
}

/**
 * @brief Update velocity calculation for an encoder
 *
 * Calculates velocity using a time-windowed approach for smooth, accurate
 * velocity estimation even at low speeds.
 *
 * @param encoder_id Encoder instance ID
 * @param timestamp Current timestamp
 */
static void update_encoder_velocity(uint8_t encoder_id, uint64_t timestamp)
{
	encoder_processing_state_t &state = g_processing_state[encoder_id];
	const encoder_instance_t &instance = g_encoder_instances[encoder_id];

	// Don't update velocity too frequently to reduce computational load
	if (timestamp - state.last_velocity_update_time < static_cast<uint64_t>(MIN_VELOCITY_UPDATE_S * 1e6f)) {
		return;
	}

	// Find the oldest sample within the velocity window
	const uint64_t window_start = timestamp - static_cast<uint64_t>(VELOCITY_WINDOW_S * 1e6f);

	int oldest_valid_index = -1;
	uint8_t current_index = state.velocity_buffer_head;

	for (int i = 0; i < VELOCITY_BUFFER_SIZE; i++) {
		current_index = (current_index + VELOCITY_BUFFER_SIZE - 1) % VELOCITY_BUFFER_SIZE;

		if (state.velocity_buffer[current_index].timestamp >= window_start) {
			oldest_valid_index = current_index;
		} else {
			break;
		}
	}

	// Calculate velocity if we have valid samples
	if (oldest_valid_index >= 0) {
		const uint8_t newest_index = (state.velocity_buffer_head + VELOCITY_BUFFER_SIZE - 1) % VELOCITY_BUFFER_SIZE;

		const int64_t delta_position = state.velocity_buffer[newest_index].position -
					       state.velocity_buffer[oldest_valid_index].position;
		const uint64_t delta_time = state.velocity_buffer[newest_index].timestamp -
					    state.velocity_buffer[oldest_valid_index].timestamp;

		if (delta_time > 0) {
			// Calculate angular velocity in rad/s
			const float position_per_pulse = 2.0f * M_PI_F / static_cast<float>(instance.config.pulses_per_revolution);
			const float velocity_rad_s = static_cast<float>(delta_position) * position_per_pulse /
						      (static_cast<float>(delta_time) * 1e-6f);

			// Store velocity values (will be copied to processed data in update_processed_data)
			state.velocity_rad_s = velocity_rad_s;
			state.velocity_rpm = velocity_rad_s * 60.0f / (2.0f * M_PI_F);
		}
	}

	state.last_velocity_update_time = timestamp;
}

/**
 * @brief Update processed data structure for an encoder
 *
 * This function updates the thread-safe processed data structure with the
 * latest encoder state, position, velocity, and statistics.
 *
 * @param encoder_id Encoder instance ID
 */
static void update_processed_data(uint8_t encoder_id)
{
	encoder_processing_state_t &state = g_processing_state[encoder_id];
	const encoder_instance_t &instance = g_encoder_instances[encoder_id];

	// Acquire semaphore for thread-safe access
	if (px4_sem_wait(&state.data_sem) != 0) {
		return; // Semaphore not available, try again later
	}

	// Update processed data structure
	state.processed_data.timestamp = hrt_absolute_time();
	state.processed_data.encoder_id = encoder_id;

	// Position data
	const int64_t effective_position = state.position_raw + state.position_offset;
	state.processed_data.position_raw = state.position_raw;
	state.processed_data.position_rad = static_cast<float>(effective_position) * 2.0f * M_PI_F /
					    static_cast<float>(instance.config.pulses_per_revolution);
	state.processed_data.position_deg = state.processed_data.position_rad * 180.0f / M_PI_F;

	// Velocity data
	state.processed_data.velocity_rad_s = state.velocity_rad_s;
	state.processed_data.velocity_rpm = state.velocity_rpm;
	state.processed_data.velocity_updated = true;

	// Direction and status
	state.processed_data.direction_forward = state.direction_forward;
	state.processed_data.index_detected = state.index_detected_this_revolution;

	// Counters
	state.processed_data.pulse_count = state.pulse_count;
	state.processed_data.direction_changes = state.direction_changes;
	state.processed_data.error_count = state.error_count;

	// Validity flags
	state.processed_data.data_valid = true;
	state.processed_data.position_updated = state.data_updated;

	// Release semaphore
	px4_sem_post(&state.data_sem);

	// Reset update flags
	state.index_detected_this_revolution = false;
	state.data_updated = false;
}

/** @} */

/**
 * @defgroup qencoder_hal_thread_mgmt Thread Management Functions
 * @{
 */

/**
 * @brief Start the event processor thread
 *
 * This function initializes the processing state for all encoders and starts
 * the dedicated event processor thread.
 *
 * @return 0 on success, negative error code on failure
 */
int qencoder_start_event_processor(void)
{
	if (g_processor_running) {
		return 0; // Already running
	}

	if (!g_qencoder_initialized) {
		return -ENOTCONN; // Hardware not initialized
	}

	// Initialize processing state for all encoders
	for (uint8_t i = 0; i < ENCODER_MAX_INSTANCES; i++) {
		encoder_processing_state_t &state = g_processing_state[i];
		memset(&state, 0, sizeof(state));
		state.direction_forward = true;
		state.last_ab_state = 0;
		state.velocity_rad_s = 0.0f;
		state.velocity_rpm = 0.0f;
		state.processed_data.encoder_id = i;

		const int sem_ret = px4_sem_init(&state.data_sem, 0, 1); // Binary semaphore

		if (sem_ret != 0) {
			// Clean up previously initialized semaphores
			for (uint8_t j = 0; j < i; j++) {
				px4_sem_destroy(&g_processing_state[j].data_sem);
			}

			return -EIO; // Semaphore initialization failed
		}

		state.data_updated = false;
	}

	// Initialize statistics
	memset(&g_processor_stats, 0, sizeof(g_processor_stats));

	// Create the processor thread
	g_processor_should_exit = false;

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 2048);

	const int ret = pthread_create(&g_processor_thread, &attr, encoder_event_processor_thread, nullptr);
	pthread_attr_destroy(&attr);

	if (ret != 0) {
		// Clean up semaphores on thread creation failure
		for (uint8_t i = 0; i < ENCODER_MAX_INSTANCES; i++) {
			px4_sem_destroy(&g_processing_state[i].data_sem);
		}

		return -EAGAIN; // Thread creation failed
	}

	// Wait for thread to start
	usleep(1000); // 1ms

	return 0;
}

/**
 * @brief Stop the event processor thread
 *
 * This function signals the event processor thread to exit and waits for
 * it to complete, then cleans up all resources.
 *
 * @return 0 on success, negative error code on failure
 */
int qencoder_stop_event_processor(void)
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
	for (uint8_t i = 0; i < ENCODER_MAX_INSTANCES; i++) {
		px4_sem_destroy(&g_processing_state[i].data_sem);
	}

	return 0;
}

/**
 * @brief Get processed encoder data
 *
 * This function retrieves the latest processed encoder data in a thread-safe
 * manner using semaphore protection.
 *
 * @param encoder_id Encoder instance ID
 * @param data Pointer to store processed data
 * @return true if data is valid, false on error or invalid data
 */
bool qencoder_get_processed_data(uint8_t encoder_id, encoder_processed_data_t *data)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES || data == nullptr) {
		return false;
	}

	if (!g_encoder_instances[encoder_id].is_active) {
		return false;
	}

	encoder_processing_state_t &state = g_processing_state[encoder_id];

	// Acquire semaphore for thread-safe access
	if (px4_sem_wait(&state.data_sem) != 0) {
		return false; // Semaphore not available, try again later
	}

	// Copy the stable buffer data
	*data = state.processed_data;

	// Release semaphore
	px4_sem_post(&state.data_sem);

	return data->data_valid;
}

/**
 * @brief Get event processor statistics
 *
 * @param stats Pointer to store statistics
 * @return 0 on success, negative error code on failure
 */
int qencoder_get_processor_stats(encoder_event_processor_stats_t *stats)
{
	if (stats == nullptr) {
		return -EINVAL;
	}

	*stats = g_processor_stats;
	return 0;
}

/**
 * @brief Check if event processor is running
 *
 * @return true if running, false if stopped
 */
bool qencoder_is_processor_running(void)
{
	return g_processor_running;
}

/** @} */
