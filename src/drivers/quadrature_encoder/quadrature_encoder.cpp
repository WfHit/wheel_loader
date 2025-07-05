/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @brief Implementation of high-performance quadrature encoder driver
 */

#include "quadrature_encoder.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <stm32_gpio.h>
#include <chip.h>
#include <hardware/stm32_exti.h>

// External board configuration (defined in board-specific files)
#ifdef BOARD_HAS_QUADRATURE_ENCODER_CONFIG
extern const struct QuadratureEncoderConfig g_quadrature_encoder_config[];
extern const unsigned int g_quadrature_encoder_count;
#else
#warning "BOARD_HAS_QUADRATURE_ENCODER_CONFIG is not defined - quadrature encoder will not work"
#endif

// Static instance storage
QuadratureEncoder *QuadratureEncoder::_instances[MAX_INSTANCES] = {};

// Static work queue for processing encoder events
struct work_s QuadratureEncoder::_work_process_events;

// Quadrature state transition table
// Index: (old_state << 2) | new_state
// Value: -1 (reverse), 0 (no change), 1 (forward), 2 (error)
const int8_t QuadratureEncoder::_state_table[16] = {
	 0,  1, -1,  2,  // old state = 00
	-1,  0,  2,  1,  // old state = 01
	 1,  2,  0, -1,  // old state = 10
	 2, -1,  1,  0   // old state = 11
};

// ============================================================================
// QuadratureEncoder Implementation
// ============================================================================

QuadratureEncoder::QuadratureEncoder(uint8_t instance) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_instance(instance),
	_filter(3)
{
	PX4_INFO("QuadratureEncoder constructor called for instance %u", (unsigned)instance);

	// Initialize work queue structure
	memset(&_work_process_events, 0, sizeof(_work_process_events));

	// Initialize event structure
	_pending_event.timestamp = 0;
	_pending_event.gpio_state = 0;
	_event_pending.store(false);

	PX4_INFO("Initializing performance counters for instance %u", (unsigned)instance);

	// Initialize performance counters
	char perf_name[32];
	snprintf(perf_name, sizeof(perf_name), "qenc_%u_cycle", _instance);
	_cycle_perf = perf_alloc(PC_ELAPSED, perf_name);

	snprintf(perf_name, sizeof(perf_name), "qenc_%u_int", _instance);
	_interrupt_perf = perf_alloc(PC_COUNT, perf_name);

	snprintf(perf_name, sizeof(perf_name), "qenc_%u_err", _instance);
	_error_perf = perf_alloc(PC_COUNT, perf_name);

	PX4_INFO("QuadratureEncoder constructor completed for instance %u", (unsigned)instance);
}

QuadratureEncoder::~QuadratureEncoder()
{
	// Stop operation
	ScheduleClear();

	// Cancel any pending work
	work_cancel(HPWORK, &_work_process_events);

	// Cleanup hardware
	cleanup_hardware();

	// Unadvertise topics
	if (_encoder_pub != nullptr) {
		orb_unadvertise(_encoder_pub);
	}

	// Free performance counters
	perf_free(_cycle_perf);
	perf_free(_interrupt_perf);
	perf_free(_error_perf);
}

bool QuadratureEncoder::init()
{
	PX4_INFO("QuadratureEncoder::init() - Starting initialization for instance %u", (unsigned)_instance);

	// Check if board configuration exists
#ifdef BOARD_HAS_QUADRATURE_ENCODER_CONFIG
	PX4_INFO("BOARD_HAS_QUADRATURE_ENCODER_CONFIG is defined");
	PX4_INFO("g_quadrature_encoder_count = %u", (unsigned)g_quadrature_encoder_count);

	if (g_quadrature_encoder_count == 0) {
		PX4_ERR("No board configuration available for quadrature encoder");
		return false;
	}
#else
	PX4_ERR("Board does not support quadrature encoder configuration - BOARD_HAS_QUADRATURE_ENCODER_CONFIG not defined");
	return false;
#endif

	// Load board configuration
	if (_instance >= g_quadrature_encoder_count) {
		PX4_ERR("Instance %u not supported (max %u)", (unsigned)_instance, (unsigned)(g_quadrature_encoder_count - 1));
		return false;
	}

	PX4_INFO("Loading configuration for instance %u", (unsigned)_instance);
	_config = g_quadrature_encoder_config[_instance];

	PX4_INFO("Configuration loaded - GPIO A: 0x%08lx, GPIO B: 0x%08lx",
		(unsigned long)_config.gpio_a, (unsigned long)_config.gpio_b);

	// Validate configuration
	if (_config.gpio_a == 0 || _config.gpio_b == 0 ||
	    _config.gpio_a == _config.gpio_b) {
		PX4_ERR("Invalid GPIO configuration for instance %u - GPIO A: 0x%08lx, GPIO B: 0x%08lx",
			(unsigned)_instance, (unsigned long)_config.gpio_a, (unsigned long)_config.gpio_b);
		return false;
	}

	PX4_INFO("GPIO configuration valid for instance %u", (unsigned)_instance);

	// Update parameters
	PX4_INFO("Updating parameters for instance %u", (unsigned)_instance);
	update_parameters();

	// Initialize hardware
	PX4_INFO("Initializing hardware for instance %u", (unsigned)_instance);
	if (!initialize_hardware()) {
		PX4_ERR("Hardware initialization failed for instance %u", (unsigned)_instance);
		return false;
	}

	// Attach interrupts
	PX4_INFO("Attaching interrupts for instance %u", (unsigned)_instance);
	if (!attach_interrupts()) {
		PX4_ERR("Interrupt attachment failed for instance %u", (unsigned)_instance);
		cleanup_hardware();
		return false;
	}

	// Initialize publication
	sensor_quad_encoder_s msg{};
	msg.timestamp = hrt_absolute_time();

	_encoder_pub = orb_advertise_multi(ORB_ID(sensor_quad_encoder), &msg, &_pub_instance);

	if (_encoder_pub == nullptr) {
		PX4_ERR("Failed to advertise sensor_quad_encoder");
		cleanup_hardware();
		return false;
	}

	// Reset state
	reset_position();

	// Start periodic updates
	uint32_t interval_us = 1_s / math::max(static_cast<int>(_param_update_rate.get()), 1);
	ScheduleOnInterval(interval_us);

	_initialized.store(true);

	PX4_INFO("Encoder %u initialized on pins %lu/%lu",
		(unsigned)_instance, (unsigned long)_config.gpio_a, (unsigned long)_config.gpio_b);

	return true;
}

bool QuadratureEncoder::initialize_hardware()
{
	PX4_INFO("Configuring GPIO A (0x%08lx) as input with pull-up and interrupt", (unsigned long)_config.gpio_a);

	// Configure GPIO A as input with pull-up and interrupt
	if (stm32_configgpio(_config.gpio_a | GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI) != 0) {
		PX4_ERR("Failed to configure GPIO A (0x%08lx)", (unsigned long)_config.gpio_a);
		return false;
	}

	PX4_INFO("GPIO A configured successfully");
	PX4_INFO("Configuring GPIO B (0x%08lx) as input with pull-up", (unsigned long)_config.gpio_b);

	// Configure GPIO B as input with pull-up (no interrupt)
	if (stm32_configgpio(_config.gpio_b | GPIO_INPUT | GPIO_PULLUP) != 0) {
		PX4_ERR("Failed to configure GPIO B (0x%08lx)", (unsigned long)_config.gpio_b);
		stm32_configgpio(_config.gpio_a | GPIO_INPUT | GPIO_FLOAT);
		return false;
	}

	PX4_INFO("GPIO B configured successfully");

	// Read initial state
	bool a_state = stm32_gpioread(_config.gpio_a);
	bool b_state = stm32_gpioread(_config.gpio_b);
	_state.quadrature_state = (b_state << 1) | a_state;

	PX4_INFO("Initial GPIO states - A: %d, B: %d, combined: %u",
		(int)a_state, (int)b_state, (unsigned)_state.quadrature_state);

	return true;
}

void QuadratureEncoder::cleanup_hardware()
{
	detach_interrupts();

	// Reset GPIO pins to safe state
	if (_config.gpio_a != 0) {
		stm32_configgpio(_config.gpio_a | GPIO_INPUT | GPIO_FLOAT);
	}

	if (_config.gpio_b != 0) {
		stm32_configgpio(_config.gpio_b | GPIO_INPUT | GPIO_FLOAT);
	}
}

bool QuadratureEncoder::attach_interrupts()
{
	PX4_INFO("Attaching interrupt handler to GPIO A (0x%08lx)", (unsigned long)_config.gpio_a);

	// Use stm32_gpiosetevent to set up GPIO interrupt
	// rising=true, falling=true, event=true (both edges), function=interrupt_handler, arg=this
	if (stm32_gpiosetevent(_config.gpio_a, true, true, true, interrupt_handler, this) != 0) {
		PX4_ERR("Failed to attach interrupt handler to GPIO A");
		return false;
	}

	_interrupts_attached.store(true);
	PX4_INFO("Interrupt handler attached successfully");

	return true;
}

void QuadratureEncoder::detach_interrupts()
{
	if (!_interrupts_attached.load()) {
		return;
	}

	// Disable GPIO interrupt by setting the handler to NULL
	stm32_gpiosetevent(_config.gpio_a, false, false, false, NULL, NULL);

	_interrupts_attached.store(false);
}

int QuadratureEncoder::interrupt_handler(int irq, void *context, void *arg)
{
	auto *instance = static_cast<QuadratureEncoder *>(arg);

	if (instance == nullptr) {
		PX4_ERR("Interrupt handler: instance is null");
		return 0;
	}

	if (!instance->_initialized.load()) {
		PX4_DEBUG("Interrupt handler: instance %u not initialized", (unsigned)instance->_instance);
		return 0;
	}

	perf_count(instance->_interrupt_perf);

	// Read GPIO states immediately
	bool a_state = stm32_gpioread(instance->_config.gpio_a);
	bool b_state = stm32_gpioread(instance->_config.gpio_b);
	uint8_t gpio_state = (b_state << 1) | a_state;

	// Store event data for work queue processing
	instance->_pending_event.timestamp = hrt_absolute_time();
	instance->_pending_event.gpio_state = gpio_state;

	// Schedule work queue if not already pending
	if (!instance->_event_pending.load()) {
		instance->_event_pending.store(true);
		work_queue(HPWORK, &_work_process_events,
			(worker_t)&QuadratureEncoder::process_events_trampoline, instance, 0);
	}

	return 0;
}

void QuadratureEncoder::process_events_trampoline(void *arg)
{
	auto *instance = static_cast<QuadratureEncoder *>(arg);
	instance->process_events();
}

void QuadratureEncoder::process_events()
{
	// Clear the pending flag
	_event_pending.store(false);

	// Process the pending event
	process_state_change(_pending_event.gpio_state, _pending_event.timestamp);
}

void QuadratureEncoder::process_state_change(uint8_t gpio_state, hrt_abstime timestamp)
{
	// Extract individual GPIO states
	bool a_state = gpio_state & 0x01;
	bool b_state = (gpio_state >> 1) & 0x01;

	// Apply filtering if enabled
	if (_config.enable_filtering) {
		if (!_filter.update(a_state, b_state)) {
			return;  // Signal not stable
		}
		a_state = _filter.get_filtered_a();
		b_state = _filter.get_filtered_b();
	}

	// Calculate new quadrature state
	uint8_t new_state = (b_state << 1) | a_state;
	uint8_t old_state = _state.quadrature_state;

	// Look up transition in state table
	uint8_t transition = (old_state << 2) | new_state;
	int8_t delta = _state_table[transition];

	if (delta == 2) {
		// Invalid transition - error
		_state.errors++;
		perf_count(_error_perf);
	} else if (delta != 0) {
		// Valid transition
		if (_config.invert_direction) {
			delta = -delta;
		}

		_state.position += delta;
		_state.transitions++;
		_state.last_update = timestamp;
	}

	// Update quadrature state
	_state.quadrature_state = new_state;
}

void QuadratureEncoder::Run()
{
	perf_begin(_cycle_perf);

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		update_parameters();
	}

	// Calculate velocity
	calculate_velocity();

	// Check health
	check_health();

	// Publish data
	publish_data();

	perf_end(_cycle_perf);
}

void QuadratureEncoder::update_parameters()
{
	PX4_INFO("Updating parameters for instance %u", (unsigned)_instance);

	// Update instance-specific parameters
	_config.pulses_per_revolution = get_instance_ppr();
	_config.invert_direction = get_instance_invert();
	_config.enable_filtering = _param_filter_enabled.get();

	PX4_INFO("Parameter values - PPR: %u, Invert: %s, Filtering: %s",
		(unsigned)_config.pulses_per_revolution,
		_config.invert_direction ? "true" : "false",
		_config.enable_filtering ? "enabled" : "disabled");

	// Update filter configuration
	_filter = QuadratureSignalFilter(_config.filter_window);
}

int32_t QuadratureEncoder::get_instance_ppr() const
{
	switch (_instance) {
	case 0: return _param_ppr_0.get();
	case 1: return _param_ppr_1.get();
	case 2: return _param_ppr_2.get();
	case 3: return _param_ppr_3.get();
	default: return 1024;  // Default value
	}
}

bool QuadratureEncoder::get_instance_invert() const
{
	switch (_instance) {
	case 0: return _param_invert_0.get();
	case 1: return _param_invert_1.get();
	case 2: return _param_invert_2.get();
	case 3: return _param_invert_3.get();
	default: return false;  // Default value
	}
}

void QuadratureEncoder::calculate_velocity()
{
	hrt_abstime now = hrt_absolute_time();
	int32_t current_position = _state.position;

	// Calculate time delta
	float dt = (now - _state.velocity_timestamp) * 1e-6f;  // Convert to seconds

	if (dt > 0.001f && dt < 1.0f) {  // Valid time delta
		// Calculate position delta
		int32_t delta_position = current_position - _state.velocity_position;

		// Convert to radians per second
		float counts_to_rad = (2.0f * M_PI_F) / _config.pulses_per_revolution;
		float velocity = (delta_position * counts_to_rad) / dt;

		// Apply simple low-pass filter
		float alpha = 0.1f;  // Filter coefficient
		float filtered_velocity = alpha * velocity + (1.0f - alpha) * _state.velocity;

		_state.velocity = filtered_velocity;
	}

	// Update tracking variables
	_state.velocity_timestamp = now;
	_state.velocity_position = current_position;

	// Update cumulative angle
	float angle = current_position * (2.0f * M_PI_F) / _config.pulses_per_revolution;
	_state.angle = angle;
}

void QuadratureEncoder::check_health()
{
	hrt_abstime now = hrt_absolute_time();

	if (now - _last_health_check < HEALTH_CHECK_INTERVAL_US) {
		return;
	}

	_last_health_check = now;

	// Calculate error rate
	uint32_t transitions = _state.transitions;
	uint32_t errors = _state.errors;

	bool healthy = true;

	if (transitions > 0) {
		float error_rate = (100.0f * errors) / transitions;

		if (error_rate > _param_max_error_rate.get()) {
			PX4_WARN("Encoder %u: High error rate %.1f%%", (unsigned)_instance, (double)error_rate);
			healthy = false;
		}
	}

	// Check for recent updates
	uint64_t last_update = _state.last_update;
	if (now - last_update > 5_s) {
		healthy = false;  // No updates for 5 seconds
	}

	_state.healthy = healthy;
}

void QuadratureEncoder::publish_data()
{
	sensor_quad_encoder_s msg{};

	msg.timestamp = hrt_absolute_time();
	msg.count = 1; // We have one encoder

	// Fill encoder data for this instance
	msg.position[_instance] = _state.position;
	msg.velocity[_instance] = _state.velocity;
	msg.angle_or_distance[_instance] = _state.angle;
	msg.valid[_instance] = 1;
	msg.encoder_type[_instance] = sensor_quad_encoder_s::TYPE_ROTARY;
	msg.pulses_per_rev[_instance] = _config.pulses_per_revolution;
	msg.invert_direction[_instance] = _config.invert_direction;
	msg.gear_ratio[_instance] = 1.0f; // Default gear ratio

	orb_publish(ORB_ID(sensor_quad_encoder), _encoder_pub, &msg);
}

void QuadratureEncoder::reset_position()
{
	_state.position = 0;
	_state.angle = 0.0f;
	_state.velocity = 0.0f;
	_state.errors = 0;
	_state.transitions = 0;
	_state.velocity_position = 0;
	_state.velocity_timestamp = hrt_absolute_time();
	_filter.reset();

	PX4_INFO("Encoder %u position reset", (unsigned)_instance);
}

int QuadratureEncoder::print_status()
{
	PX4_INFO("Quadrature Encoder %u:", (unsigned)_instance);
	PX4_INFO("  GPIO A: 0x%08lx", (unsigned long)_config.gpio_a);
	PX4_INFO("  GPIO B: 0x%08lx", (unsigned long)_config.gpio_b);
	PX4_INFO("  PPR: %lu", (unsigned long)_config.pulses_per_revolution);
	PX4_INFO("  Inverted: %s", _config.invert_direction ? "yes" : "no");
	PX4_INFO("  Filtering: %s", _config.enable_filtering ? "enabled" : "disabled");

	PX4_INFO("  Position: %ld counts", static_cast<long>(_state.position));
	PX4_INFO("  Angle: %.2f rad", (double)_state.angle);
	PX4_INFO("  Velocity: %.2f rad/s", (double)_state.velocity);
	PX4_INFO("  Healthy: %s", _state.healthy ? "yes" : "no");

	uint32_t transitions = _state.transitions;
	uint32_t errors = _state.errors;

	PX4_INFO("  Transitions: %lu", (unsigned long)transitions);
	PX4_INFO("  Errors: %lu", (unsigned long)errors);

	if (transitions > 0) {
		float error_rate = (100.0f * errors) / transitions;
		PX4_INFO("  Error rate: %.2f%%", (double)error_rate);
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interrupt_perf);
	perf_print_counter(_error_perf);

	return 0;
}

// ============================================================================
// Module Interface
// ============================================================================

int QuadratureEncoder::task_spawn(int argc, char *argv[])
{
	PX4_INFO("QuadratureEncoder::task_spawn() called with %d arguments", argc);

	int instance = 0;
	int ch;

	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:", nullptr, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			instance = atoi(myoptarg);
			PX4_INFO("Instance parameter set to: %d", instance);
			break;
		default:
			print_usage("Unknown option");
			return -1;
		}
	}

	if (instance < 0 || instance >= MAX_INSTANCES) {
		PX4_ERR("Invalid instance: %d (valid range: 0-%d)", instance, MAX_INSTANCES-1);
		return -1;
	}

	if (_instances[instance] != nullptr) {
		PX4_ERR("Instance %d already running", instance);
		return -1;
	}

	PX4_INFO("Creating quadrature encoder instance %d", instance);

	// Create and initialize instance
	auto *encoder = new QuadratureEncoder(instance);

	if (encoder == nullptr) {
		PX4_ERR("Failed to allocate encoder instance");
		return -1;
	}

	PX4_INFO("Encoder instance %d created, initializing...", instance);

	if (!encoder->init()) {
		PX4_ERR("Failed to initialize encoder instance %d", instance);
		delete encoder;
		return -1;
	}

	_instances[instance] = encoder;
	PX4_INFO("Quadrature encoder instance %d started successfully", instance);

	return 0;
}

QuadratureEncoder *QuadratureEncoder::instantiate(int instance)
{
	if (instance < 0 || instance >= MAX_INSTANCES) {
		return nullptr;
	}

	return _instances[instance];
}

int QuadratureEncoder::custom_command(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage("Missing command");
		return -1;
	}

	if (!strcmp(argv[0], "reset")) {
		// Parse instance
		int instance = 0;
		if (argc >= 3) {
			instance = atoi(argv[2]);
		}

		auto *encoder = instantiate(instance);
		if (encoder == nullptr) {
			PX4_ERR("Instance %d not running", instance);
			return -1;
		}

		encoder->reset_position();
		return 0;
	}

	print_usage("Unknown command");
	return -1;
}

int QuadratureEncoder::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
High-performance quadrature encoder driver using GPIO interrupts.

This driver reads quadrature encoders without requiring hardware timer peripherals,
making it suitable for applications where timer resources are limited or when
maximum flexibility is needed.

Features:
- Multi-instance support (up to 4 encoders)
- Digital filtering for noise immunity
- Real-time velocity calculation
- Comprehensive error detection
- Thread-safe operation

### Examples
Start encoder on instance 0:
$ quadrature_encoder start -i 0

Reset encoder position:
$ quadrature_encoder reset -i 0

Show status:
$ quadrature_encoder status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("quadrature_encoder", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver on specific instance");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "Instance number", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset encoder position");

	return 0;
}

// Board configuration is only available if BOARD_HAS_QUADRATURE_ENCODER_CONFIG is defined
// This eliminates the need for weak symbols and provides cleaner conditional compilation

extern "C" __EXPORT int quadrature_encoder_main(int argc, char *argv[])
{
	PX4_INFO("quadrature_encoder_main() called with %d arguments", argc);

	for (int i = 0; i < argc; i++) {
		PX4_INFO("  argv[%d]: %s", i, argv[i]);
	}

	return QuadratureEncoder::main(argc, argv);
}
