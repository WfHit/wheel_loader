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

#include "quad_encoder_gpio.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <stm32_gpio.h>

// Static storage for multiple instances
QuadEncoderGPIO *_objects_gpio[QuadEncoderGPIO::MAX_INSTANCES] = {};

// Quadrature decoder lookup table for state transitions
// Index: (old_state << 2) | new_state
// Value: direction (-1, 0, 1) or error (2)
static const int8_t QUADRATURE_TABLE[16] = {
	// old_state = 00 (0)
	 0, // 00 -> 00: no change
	 1, // 00 -> 01: forward
	-1, // 00 -> 10: reverse
	 2, // 00 -> 11: error
	// old_state = 01 (1)
	-1, // 01 -> 00: reverse
	 0, // 01 -> 01: no change
	 2, // 01 -> 10: error
	 1, // 01 -> 11: forward
	// old_state = 10 (2)
	 1, // 10 -> 00: forward
	 2, // 10 -> 01: error
	 0, // 10 -> 10: no change
	-1, // 10 -> 11: reverse
	// old_state = 11 (3)
	 2, // 11 -> 00: error
	-1, // 11 -> 01: reverse
	 1, // 11 -> 10: forward
	 0, // 11 -> 11: no change
};

/*
 * GPIO-based Quadrature Encoder Driver Implementation
 *
 * This implementation provides robust encoder reading through GPIO interrupts
 * with thread-safe operation and comprehensive error handling.
 */


QuadEncoderGPIO::QuadEncoderGPIO(int instance_id) :
	ModuleParams(nullptr),
	ScheduledWorkItem("quad_encoder_gpio", px4::wq_configurations::hp_default),
	_instance_id(instance_id)
{
	// Initialize encoder state with atomic initialization
	_encoder_state = {};
	_prev_position.store(0);
	_prev_timestamp.store(0);
	_is_running.store(false);
	_interrupts_attached.store(false);
	_error_count.store(0);
	_interrupt_count.store(0);
	_last_error_time.store(0);
	_consecutive_errors.store(0);
	_error_rate.store(0.0f);

	// Initialize signal filter
	_signal_filter = {};

	// Initialize timing
	_last_health_check = 0;

	PX4_DEBUG("QuadEncoderGPIO instance %d created", _instance_id);
}

QuadEncoderGPIO::~QuadEncoderGPIO()
{
	// Ensure driver is stopped
	_is_running.store(false);

	// Unadvertise topics
	if (_sensor_quad_encoder_pub != nullptr) {
		orb_unadvertise(_sensor_quad_encoder_pub);
		_sensor_quad_encoder_pub = nullptr;
	}

	// Detach interrupts and deinit GPIO
	deinit_gpio();

	// Free performance counters
	perf_free(_loop_perf);
	perf_free(_interrupt_perf);
	perf_free(_error_perf);


	PX4_DEBUG("QuadEncoderGPIO instance %d destroyed", _instance_id);

}

bool QuadEncoderGPIO::init()
{
	// Validate configuration first
	const quad_encoder_gpio_config_s* config = get_config_for_instance();
	if (!validate_config(config)) {
		PX4_ERR("Invalid encoder configuration for instance %d", _instance_id);
		return false;
	}

	// Load initial parameters
	parameters_update();

	// Initialize GPIO pins
	if (!init_gpio()) {
		PX4_ERR("Failed to initialize GPIO pins for encoder %d", _instance_id);
		return false;
	}

	// Attach interrupt handlers
	if (!attach_interrupts()) {
		PX4_ERR("Failed to attach interrupt handlers for encoder %d", _instance_id);
		deinit_gpio();
		return false;
	}

	// Initialize multi-instance publications
	sensor_quad_encoder_s sensor_msg{};
	sensor_msg.timestamp = hrt_absolute_time();

	// Advertise sensor encoder topic with instance
	_sensor_quad_encoder_pub = orb_advertise_multi(ORB_ID(sensor_quad_encoder), &sensor_msg,
		&_sensor_encoder_instance);

	if (_sensor_quad_encoder_pub == nullptr) {
		PX4_ERR("Failed to advertise sensor_quad_encoder for instance %d", _instance_id);
		deinit_gpio();
		return false;
	}

	PX4_INFO("Advertising sensor_quad_encoder instance %d", _sensor_encoder_instance);

	// Reset encoder state
	reset_encoder();

	// Start work queue with configured interval
	uint32_t interval_ms = _param_update_rate.get() > 0 ? (1000 / _param_update_rate.get()) : DEFAULT_SCHEDULE_INTERVAL;
	ScheduleOnInterval(interval_ms * 1000); // Convert ms to us

	_is_running.store(true);

	const quad_encoder_gpio_config_s* config = get_config_for_instance();
	PX4_INFO("QuadEncoderGPIO initialized (instance %d, GPIO A: %s, GPIO B: %s)",
		_instance_id,
		config ? get_gpio_pin_name(config->gpio_a) : "N/A",
		config ? get_gpio_pin_name(config->gpio_b) : "N/A");

	return true;
}

bool QuadEncoderGPIO::init_gpio()
{
	const quad_encoder_gpio_config_s* config = get_config_for_instance();
	if (!config) {
		PX4_ERR("No board configuration found for instance %d", _instance_id);
		return false;
	}

	uint32_t gpio_a = config->gpio_a;
	uint32_t gpio_b = config->gpio_b;

	if (gpio_a == 0 || gpio_b == 0) {
		PX4_ERR("Invalid GPIO pins: A=0x%08x, B=0x%08x", gpio_a, gpio_b);
		return false;
	}

	// Validate GPIO pins are different
	if (gpio_a == gpio_b) {
		PX4_ERR("GPIO A and B pins must be different: A=0x%08x, B=0x%08x", gpio_a, gpio_b);
		return false;
	}

	// Configure GPIO A as input with pull-up and interrupt capability
	int ret_a = stm32_configgpio(gpio_a | GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI);
	if (ret_a != 0) {
		PX4_ERR("Failed to configure GPIO A (0x%08x): %d", gpio_a, ret_a);
		return false;
	}

	// Configure GPIO B as input with pull-up (no interrupt on B to avoid double counting)
	int ret_b = stm32_configgpio(gpio_b | GPIO_INPUT | GPIO_PULLUP);
	if (ret_b != 0) {
		PX4_ERR("Failed to configure GPIO B (0x%08x): %d", gpio_b, ret_b);
		// Clean up GPIO A configuration
		stm32_configgpio(gpio_a | GPIO_INPUT | GPIO_PULLUP);
		return false;
	}

	// Test GPIO functionality by reading initial states
	bool a_state = read_gpio_pin(gpio_a);
	bool b_state = read_gpio_pin(gpio_b);

	PX4_INFO("GPIO initialized successfully: A=%s (state=%d), B=%s (state=%d)",
		get_gpio_pin_name(gpio_a), a_state, get_gpio_pin_name(gpio_b), b_state);

	return true;
}

void QuadEncoderGPIO::deinit_gpio()
{
	// Detach interrupts first
	detach_interrupts();

	const quad_encoder_gpio_config_s* config = get_config_for_instance();
	if (config) {
		// Configure GPIO pins as floating inputs (safe state)
		if (config->gpio_a != 0) {
			stm32_configgpio(config->gpio_a | GPIO_INPUT | GPIO_FLOAT);
		}
		if (config->gpio_b != 0) {
			stm32_configgpio(config->gpio_b | GPIO_INPUT | GPIO_FLOAT);
		}
	}

	PX4_DEBUG("GPIO deinitialized for instance %d", _instance_id);
}

bool QuadEncoderGPIO::attach_interrupts()
{
	const quad_encoder_gpio_config_s* config = get_config_for_instance();
	if (!config || config->gpio_a == 0 || config->gpio_b == 0) {
		PX4_ERR("Cannot attach interrupts: invalid GPIO pins");
		return false;
	}

	// Get IRQ number for GPIO A only (following NuttX implementation pattern)
	int irq_a = stm32_gpioirq(config->gpio_a);
	if (irq_a < 0) {
		PX4_ERR("Failed to get IRQ number for GPIO A (0x%08x): %d", config->gpio_a, irq_a);
		return false;
	}

	// Attach interrupt handler for phase A only
	int ret = irq_attach(irq_a, gpio_interrupt_handler, this);
	if (ret != 0) {
		PX4_ERR("Failed to attach interrupt handler for GPIO A (IRQ %d): %d", irq_a, ret);
		return false;
	}

	// Enable interrupt
	up_enable_irq(irq_a);

	_interrupts_attached.store(true);

	PX4_INFO("Interrupt attached successfully: GPIO A IRQ=%d", irq_a);
	return true;
}

void QuadEncoderGPIO::detach_interrupts()
{
	if (!_interrupts_attached.load()) {
		return;
	}

	const quad_encoder_gpio_config_s* config = get_config_for_instance();
	if (config && config->gpio_a != 0) {
		int irq_a = stm32_gpioirq(config->gpio_a);
		if (irq_a >= 0) {
			up_disable_irq(irq_a);
			irq_detach(irq_a);
			PX4_DEBUG("Interrupt detached: GPIO A IRQ=%d", irq_a);
		}
	}

	_interrupts_attached.store(false);
}

int QuadEncoderGPIO::gpio_interrupt_handler(int irq, void *context, void *arg)
{
	QuadEncoderGPIO *instance = static_cast<QuadEncoderGPIO *>(arg);

	if (instance == nullptr) {
		return -1;
	}

	// Check if instance is still running
	if (!instance->_is_running.load()) {
		return 0;
	}

	// Update performance counters
	perf_count(instance->_interrupt_perf);
	instance->_interrupt_count.fetch_add(1);

	// Get board configuration for GPIO pins
	const quad_encoder_gpio_config_s* config = instance->get_config_for_instance();
	if (!config) {
		return -1;
	}

	// Read current GPIO states
	bool a_state = instance->read_gpio_pin(config->gpio_a);
	bool b_state = instance->read_gpio_pin(config->gpio_b);

	// Process the encoder state change
	instance->process_encoder_state(a_state, b_state);

	return 0;
}

void QuadEncoderGPIO::process_encoder_state(bool a_state, bool b_state)
{
	// Apply signal filtering if enabled (parameter controlled)
	if (_param_filter_en.get()) {
		if (!apply_signal_filter(a_state, b_state)) {
			// Signals not stable, ignore this transition
			return;
		}
		// Use filtered signals
		a_state = _signal_filter.last_filtered_a;
		b_state = _signal_filter.last_filtered_b;
	}

	// Update current quadrature state (2-bit value: A=LSB, B=MSB)
	uint8_t new_state = (b_state << 1) | a_state;
	uint8_t last_state = _encoder_state.last_state.load();

	// Validate state transition
	if (!validate_state_transition(last_state, new_state)) {
		update_diagnostics(EncoderError::INVALID_TRANSITION);
		return;
	}

	// Calculate state transition index for lookup table
	uint8_t transition = (last_state << 2) | new_state;

	// Get direction from lookup table
	int8_t delta = QUADRATURE_TABLE[transition];

	if (delta == 2) {
		// Invalid transition - quadrature error
		update_diagnostics(EncoderError::INVALID_TRANSITION);
		_encoder_state.error_count.fetch_add(1);
		_error_count.fetch_add(1);
		_consecutive_errors.fetch_add(1);

		// Check if we've exceeded maximum consecutive errors
		if (_consecutive_errors.load() > MAX_CONSECUTIVE_ERRORS) {
			update_diagnostics(EncoderError::HIGH_ERROR_RATE);
		}
	} else if (delta != 0) {
		// Valid transition - reset consecutive error counter
		_consecutive_errors.store(0);

		// Apply direction inversion if needed
		if (_encoder_state.invert_direction) {
			delta = -delta;
		}

		// Update position atomically
		int32_t new_position = _encoder_state.position.fetch_add(delta) + delta;
		_encoder_state.position_change.fetch_add(delta);

		// Update timestamp and validity
		_encoder_state.timestamp.store(hrt_absolute_time());
		_encoder_state.valid.store(true);

		// Update transition counter
		_encoder_state.total_transitions.fetch_add(1);

		// Debug logging for significant position changes
		if (abs(delta) > 1) {
			PX4_DEBUG("Encoder %d: large delta=%d, pos=%d, state=%d->%d",
				_instance_id, delta, new_position, last_state, new_state);
		}
	}

	// Update state tracking
	_encoder_state.last_state.store(last_state);
	_encoder_state.current_state.store(new_state);
}

bool QuadEncoderGPIO::apply_signal_filter(bool a_state, bool b_state)
{
	// Simple majority voting filter
	_signal_filter.a_samples[_signal_filter.sample_index] = a_state;
	_signal_filter.b_samples[_signal_filter.sample_index] = b_state;

	_signal_filter.sample_index = (_signal_filter.sample_index + 1) % signal_filter_s::FILTER_SIZE;

	// Count majority for each signal
	int a_count = 0, b_count = 0;
	for (int i = 0; i < signal_filter_s::FILTER_SIZE; i++) {
		if (_signal_filter.a_samples[i]) a_count++;
		if (_signal_filter.b_samples[i]) b_count++;
	}

	bool filtered_a = (a_count > signal_filter_s::FILTER_SIZE / 2);
	bool filtered_b = (b_count > signal_filter_s::FILTER_SIZE / 2);

	// Check if signals are stable
	if (filtered_a == _signal_filter.last_filtered_a &&
	    filtered_b == _signal_filter.last_filtered_b) {
		_signal_filter.stable_count++;
	} else {
		_signal_filter.stable_count = 0;
	}

	_signal_filter.last_filtered_a = filtered_a;
	_signal_filter.last_filtered_b = filtered_b;

	// Require minimum stability before accepting transition
	return (_signal_filter.stable_count >= 2);
}

bool QuadEncoderGPIO::validate_state_transition(uint8_t old_state, uint8_t new_state) const
{
	// Valid quadrature transitions follow Gray code pattern
	// Check if transition is to adjacent state or no change
	uint8_t diff = old_state ^ new_state;

	// Valid transitions: no change (0), single bit change (1, 2), or both bits change (3)
	// Invalid: reserved states or impossible transitions
	return (diff == 0 || diff == 1 || diff == 2 || diff == 3);
}

void QuadEncoderGPIO::update_diagnostics(EncoderError error_type)
{
	hrt_abstime now = hrt_absolute_time();
	_last_error_time.store(now);
	_encoder_state.last_error_time = now;
	_encoder_state.last_error_type = error_type;

	// Update performance counter
	perf_count(_error_perf);

	// Calculate error rate
	uint32_t total_transitions = _encoder_state.total_transitions.load();
	uint32_t errors = _error_count.load();

	if (total_transitions > 0) {
		float error_rate = (float)errors / total_transitions * 100.0f;
		_error_rate.store(error_rate);

		if (error_rate > _param_max_err_rate.get()) {
			PX4_WARN("Encoder %d high error rate: %.2f%% (%u errors / %u transitions)",
				_instance_id, error_rate, errors, total_transitions);
		}
	}

	switch (error_type) {
		case EncoderError::INVALID_TRANSITION:
			PX4_DEBUG("Encoder %d: Invalid transition detected", _instance_id);
			break;
		case EncoderError::HIGH_ERROR_RATE:
			PX4_DEBUG("Encoder %d: High error rate detected", _instance_id);
			break;
		case EncoderError::GPIO_FAILURE:
			PX4_DEBUG("Encoder %d: GPIO failure detected", _instance_id);
			break;
		default:
			break;
	}
}

void QuadEncoderGPIO::check_encoder_health()
{
	hrt_abstime now = hrt_absolute_time();

	// Only check health periodically
	if (now - _last_health_check < HEALTH_CHECK_INTERVAL * 1000) {
		return;
	}

	_last_health_check = now;

	// Check error rate
	float error_rate = _error_rate.load();
	if (error_rate > _param_max_err_rate.get()) {
		PX4_WARN("Encoder %d health check: High error rate %.2f%%", _instance_id, error_rate);
	}

	// Check for recent activity
	uint64_t last_timestamp = _encoder_state.timestamp.load();
	if (now - last_timestamp > 5000000) { // 5 seconds
		PX4_INFO("Encoder %d health check: No recent activity (last: %llu us ago)",
			_instance_id, now - last_timestamp);
	}

	// Check consecutive errors
	uint32_t consecutive_errors = _consecutive_errors.load();
	if (consecutive_errors > MAX_CONSECUTIVE_ERRORS / 2) {
		PX4_WARN("Encoder %d health check: %u consecutive errors", _instance_id, consecutive_errors);
	}
}

bool QuadEncoderGPIO::read_gpio_pin(uint32_t gpio_pin) const
{
	if (gpio_pin == 0) {
		return false;
	}

	return stm32_gpioread(gpio_pin);
}

const char* QuadEncoderGPIO::get_gpio_pin_name(uint32_t gpio_pin) const
{
	// Extract port and pin from GPIO definition
	uint32_t port = (gpio_pin & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
	uint32_t pin = (gpio_pin & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

	static char name_buffer[8];
	snprintf(name_buffer, sizeof(name_buffer), "P%c%d", 'A' + port, pin);
	return name_buffer;
}

void QuadEncoderGPIO::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_is_running.store(false);
		return;
	}

	perf_begin(_loop_perf);

	// Check for parameter updates
	parameter_update_s param_update;
	if (_parameter_update_sub.update(&param_update)) {
		parameters_update();
	}

	// Perform periodic health checks
	check_encoder_health();

	// Calculate velocity from position change
	hrt_abstime current_time = hrt_absolute_time();
	int32_t current_position = _encoder_state.position.load();
	calculate_velocity(current_position, current_time);

	// Publish encoder data
	sensor_quad_encoder_s sensor_msg{};
	sensor_msg.timestamp = current_time;
	sensor_msg.count = 1;

	if (_encoder_state.valid.load()) {
		sensor_msg.valid[0] = 1;
		sensor_msg.position[0] = current_position;
		sensor_msg.velocity[0] = _encoder_state.velocity_rad_s.load();
		sensor_msg.angle[0] = _encoder_state.angle_rad.load();
		sensor_msg.pulses_per_rev[0] = _encoder_state.pulses_per_rev;
		sensor_msg.invert_direction[0] = _encoder_state.invert_direction;
	} else {
		sensor_msg.valid[0] = 0;
	}

	// Publish the message
	if (_sensor_quad_encoder_pub != nullptr) {
		orb_publish(ORB_ID(sensor_quad_encoder), _sensor_quad_encoder_pub, &sensor_msg);
	}

	perf_end(_loop_perf);
}

void QuadEncoderGPIO::calculate_velocity(int32_t current_position, hrt_abstime current_time)
{
	hrt_abstime prev_time = _prev_timestamp.load();

	if (prev_time > 0) {
		// Calculate time delta
		float dt = (current_time - prev_time) / 1e6f; // Convert to seconds

		if (dt > 0.0f && dt < 1.0f) { // Reasonable time delta
			// Calculate position change
			int32_t prev_pos = _prev_position.load();
			int32_t position_change = current_position - prev_pos;

			// Calculate velocity in pulses per second
			float velocity_pulses_per_s = position_change / dt;

			// Convert to rad/s using PPR
			if (_encoder_state.pulses_per_rev > 0) {
				float velocity_rad_s = (velocity_pulses_per_s * 2.0f * M_PI_F) / _encoder_state.pulses_per_rev;
				_encoder_state.velocity_rad_s.store(velocity_rad_s);
			}

			// Calculate cumulative angle
			float angle_rad = (current_position * 2.0f * M_PI_F) / _encoder_state.pulses_per_rev;
			_encoder_state.angle_rad.store(angle_rad);
		}
	}

	// Update previous values
	_prev_position.store(current_position);
	_prev_timestamp.store(current_time);
}

void QuadEncoderGPIO::reset_encoder()
{
	// Reset all encoder state atomically
	_encoder_state.position.store(0);
	_encoder_state.velocity_rad_s.store(0.0f);
	_encoder_state.angle_rad.store(0.0f);
	_encoder_state.position_change.store(0);
	_encoder_state.error_count.store(0);
	_encoder_state.index_count.store(0);
	_encoder_state.last_index_position.store(0);
	_encoder_state.index_found.store(false);
	_encoder_state.total_transitions.store(0);
	_encoder_state.last_error_time = 0;
	_encoder_state.last_error_type = EncoderError::NONE;

	// Reset velocity calculation state
	_prev_position.store(0);
	_prev_timestamp.store(0);

	// Reset error tracking
	_error_count.store(0);
	_consecutive_errors.store(0);
	_error_rate.store(0.0f);

	// Reset signal filter
	_signal_filter = {};

	// Get board configuration for initial GPIO state reading
	const quad_encoder_gpio_config_s* config = get_config_for_instance();
	if (config) {
		// Read initial GPIO states and set initial quadrature state
		bool a_state = read_gpio_pin(config->gpio_a);
		bool b_state = read_gpio_pin(config->gpio_b);
		uint8_t initial_state = (b_state << 1) | a_state;

		_encoder_state.current_state.store(initial_state);
		_encoder_state.last_state.store(initial_state);
		_encoder_state.timestamp.store(hrt_absolute_time());
		_encoder_state.valid.store(true);

		PX4_INFO("Encoder %d reset: Initial state A=%d, B=%d, State=%d",
			_instance_id, a_state, b_state, initial_state);
	}
}

void QuadEncoderGPIO::parameters_update()
{
	// Update parameters from parameter system
	ModuleParams::updateParams();

	// Get PPR and invert settings for this instance
	const quad_encoder_gpio_config_s* config = get_config_for_instance();
	if (config != nullptr) {
		_encoder_state.pulses_per_rev = config->resolution;
		_encoder_state.invert_direction = config->invert_direction;

		PX4_DEBUG("Encoder %d using board config: PPR=%d, Invert=%d",
			_instance_id, _encoder_state.pulses_per_rev, _encoder_state.invert_direction);
	} else {
		// Fall back to parameters
		_encoder_state.pulses_per_rev = get_ppr_for_instance();
		_encoder_state.invert_direction = get_invert_for_instance();

		PX4_DEBUG("Encoder %d using parameters: PPR=%d, Invert=%d",
			_instance_id, _encoder_state.pulses_per_rev, _encoder_state.invert_direction);
	}

	// Validate PPR
	if (_encoder_state.pulses_per_rev <= 0) {
		PX4_WARN("Invalid PPR for encoder %d: %d, using default 1024",
			_instance_id, _encoder_state.pulses_per_rev);
		_encoder_state.pulses_per_rev = 1024;
	}
}

int32_t QuadEncoderGPIO::get_ppr_for_instance() const
{
	switch (_instance_id) {
	case 0: return _param_ppr_0.get();
	case 1: return _param_ppr_1.get();
	case 2: return _param_ppr_2.get();
	case 3: return _param_ppr_3.get();
	default: return 1024;
	}
}

bool QuadEncoderGPIO::get_invert_for_instance() const
{
	switch (_instance_id) {
	case 0: return _param_invert_0.get();
	case 1: return _param_invert_1.get();
	case 2: return _param_invert_2.get();
	case 3: return _param_invert_3.get();
	default: return false;
	}
}

const quad_encoder_gpio_config_s* QuadEncoderGPIO::get_config_for_instance() const
{
#ifdef CONFIG_DRIVERS_QUAD_ENCODER_GPIO
	if (_instance_id < (int)board_quad_encoder_gpio_count) {
		return &board_quad_encoder_gpio_configs[_instance_id];
	}
#endif
	return nullptr;
}

bool QuadEncoderGPIO::validate_config(const quad_encoder_gpio_config_s* config)
{
	if (config == nullptr) {
		// No board config, rely on parameters and provided GPIO pins
		return true;
	}

	// Validate GPIO pins
	if (config->gpio_a == 0 || config->gpio_b == 0) {
		PX4_ERR("Invalid GPIO pins in config: A=0x%08x, B=0x%08x",
			config->gpio_a, config->gpio_b);
		return false;
	}

	if (config->gpio_a == config->gpio_b) {
		PX4_ERR("GPIO A and B must be different: A=0x%08x, B=0x%08x",
			config->gpio_a, config->gpio_b);
		return false;
	}

	// Validate resolution
	if (config->resolution <= 0 || config->resolution > 10000) {
		PX4_ERR("Invalid resolution in config: %d", config->resolution);
		return false;
	}

	return true;
}

bool QuadEncoderGPIO::is_encoder_healthy() const
{
	// Check error rate
	float error_rate = _error_rate.load();
	if (error_rate > _param_max_err_rate.get()) {
		return false;
	}

	// Check consecutive errors
	uint32_t consecutive_errors = _consecutive_errors.load();
	if (consecutive_errors > MAX_CONSECUTIVE_ERRORS / 2) {
		return false;
	}

	// Check if encoder is providing recent data
	uint64_t last_timestamp = _encoder_state.timestamp.load();
	hrt_abstime now = hrt_absolute_time();
	if (now - last_timestamp > 5000000) { // 5 seconds
		return false;
	}

	return true;
}

int QuadEncoderGPIO::print_status()
{
	PX4_INFO("QuadEncoderGPIO Status (Instance %d):", _instance_id);
	PX4_INFO("  Configuration:");

	const quad_encoder_gpio_config_s* config = get_config_for_instance();
	if (config) {
		PX4_INFO("    GPIO A: %s (0x%08x), GPIO B: %s (0x%08x)",
			get_gpio_pin_name(config->gpio_a), config->gpio_a,
			get_gpio_pin_name(config->gpio_b), config->gpio_b);
	} else {
		PX4_INFO("    No board configuration available");
	}

	PX4_INFO("    PPR: %d, Invert: %s",
		_encoder_state.pulses_per_rev, _encoder_state.invert_direction ? "Yes" : "No");
	PX4_INFO("    Running: %s, Interrupts attached: %s",
		_is_running.load() ? "Yes" : "No", _interrupts_attached.load() ? "Yes" : "No");

	PX4_INFO("  Current State:");
	PX4_INFO("    Position: %d pulses", _encoder_state.position.load());
	PX4_INFO("    Velocity: %.3f rad/s", _encoder_state.velocity_rad_s.load());
	PX4_INFO("    Angle: %.3f rad", _encoder_state.angle_rad.load());
	PX4_INFO("    Valid: %s", _encoder_state.valid.load() ? "Yes" : "No");

	PX4_INFO("  Quadrature State:");
	PX4_INFO("    Current: %d, Last: %d",
		_encoder_state.current_state.load(), _encoder_state.last_state.load());
	PX4_INFO("    Total transitions: %u", _encoder_state.total_transitions.load());

	PX4_INFO("  Diagnostics:");
	PX4_INFO("    Interrupts: %u, Errors: %u",
		_interrupt_count.load(), _error_count.load());
	PX4_INFO("    Encoder errors: %u, Consecutive: %u",
		_encoder_state.error_count.load(), _consecutive_errors.load());
	PX4_INFO("    Error rate: %.2f%%", _error_rate.load());
	PX4_INFO("    Health: %s", is_encoder_healthy() ? "Good" : "Poor");

	if (_encoder_state.last_error_time > 0) {
		hrt_abstime now = hrt_absolute_time();
		PX4_INFO("    Last error: %llu us ago", now - _encoder_state.last_error_time);
	}

	// Print performance counters
	perf_print_counter(_loop_perf);
	perf_print_counter(_interrupt_perf);
	perf_print_counter(_error_perf);

	return 0;
}

// ... (rest of the static methods remain the same as in the original file)
// task_spawn, instantiate, usage, print_usage, custom_command

int QuadEncoderGPIO::task_spawn(int argc, char *argv[])
{
	int instance = 0;
	uint32_t gpio_a = 0;
	uint32_t gpio_b = 0;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:a:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			instance = atoi(myoptarg);
			break;
		case 'a':
			gpio_a = strtoul(myoptarg, nullptr, 16);
			break;
		case 'b':
			gpio_b = strtoul(myoptarg, nullptr, 16);
			break;
		case '?':
			error_flag = true;
			break;
		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		usage();
		return -1;
	}

	if (instance < 0 || instance >= MAX_INSTANCES) {
		PX4_ERR("Invalid instance %d (must be 0-%d)", instance, MAX_INSTANCES - 1);
		usage();
		return -1;
	}

	// Check if instance already exists
	if (_objects_gpio[instance] != nullptr) {
		PX4_ERR("Instance %d already exists", instance);
		return -1;
	}

	// Create new instance
	QuadEncoderGPIO *object = instantiate(instance);
	if (object == nullptr) {
		PX4_ERR("Failed to instantiate encoder %d", instance);
		return -1;
	}

	_objects_gpio[instance] = object;

	// Start the object
	if (!object->init()) {
		PX4_ERR("Failed to initialize encoder %d", instance);
		_objects_gpio[instance] = nullptr;
		delete object;
		return -1;
	}

	return 0;
}

QuadEncoderGPIO *QuadEncoderGPIO::instantiate(int instance)
{
	// Use board-specific configuration if available
#ifdef CONFIG_DRIVERS_QUAD_ENCODER_GPIO
	if (instance < (int)board_quad_encoder_gpio_count) {
		const struct quad_encoder_gpio_config_s *config = &board_quad_encoder_gpio_configs[instance];
		PX4_INFO("Using board config for encoder %d: GPIO A=0x%08x, B=0x%08x",
			instance, config->gpio_a, config->gpio_b);
	} else {
		PX4_WARN("No board config for encoder %d", instance);
	}
#else
	PX4_WARN("Board quadrature encoder support not enabled");
#endif

	return new QuadEncoderGPIO(instance);
}

void QuadEncoderGPIO::usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PX4_INFO("usage: quad_encoder_gpio {start|stop|status|reset} [-i <instance>] [-a <gpio_a>] [-b <gpio_b>]");
	PX4_INFO("       -i <instance>    Encoder instance (0-%d)", MAX_INSTANCES - 1);
	PX4_INFO("       -a <gpio_a>      GPIO A pin (hex, e.g., 0x20005 for PD5)");
	PX4_INFO("       -b <gpio_b>      GPIO B pin (hex, e.g., 0x20006 for PD6)");
	PX4_INFO("Commands:");
	PX4_INFO("  start                 Start encoder driver");
	PX4_INFO("  stop                  Stop encoder driver");
	PX4_INFO("  status                Show encoder status");
	PX4_INFO("  reset                 Reset encoder position to zero");
}

int QuadEncoderGPIO::print_usage(const char *reason)
{
	usage(reason);
	return 0;
}

int QuadEncoderGPIO::custom_command(int argc, char *argv[])
{
	if (argc < 2) {
		return print_usage("missing command");
	}

	if (strcmp(argv[1], "reset") == 0) {
		// Reset encoder
		reset_encoder();
		PX4_INFO("Encoder %d reset", _instance_id);
		return 0;
	} else if (strcmp(argv[1], "status") == 0) {
		// Print detailed status
		return print_status();
	} else {
		return print_usage("unrecognized command");
	}
}

extern "C" __EXPORT int quad_encoder_gpio_main(int argc, char *argv[])
{
	return QuadEncoderGPIO::main(argc, argv);
}
