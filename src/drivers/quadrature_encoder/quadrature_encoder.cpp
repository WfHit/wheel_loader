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
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>

using namespace quadrature_encoder;

// External board configuration (defined in board-specific files)
extern const struct EncoderConfig board_encoder_configs[];
extern const unsigned int board_encoder_count;

// Static instance storage
QuadratureEncoder *QuadratureEncoder::_instances[MAX_INSTANCES] = {};

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
// SignalFilter Implementation
// ============================================================================

SignalFilter::SignalFilter(uint8_t window_size) :
	_window_size(math::min(window_size, MAX_WINDOW))
{
	reset();
}

bool SignalFilter::update(bool a_state, bool b_state)
{
	// Store new samples
	_samples_a[_index] = a_state;
	_samples_b[_index] = b_state;
	_index = (_index + 1) % _window_size;

	// Calculate majority vote
	uint8_t a_count = 0, b_count = 0;
	for (uint8_t i = 0; i < _window_size; i++) {
		if (_samples_a[i]) a_count++;
		if (_samples_b[i]) b_count++;
	}

	bool new_a = (a_count > _window_size / 2);
	bool new_b = (b_count > _window_size / 2);

	// Check stability
	if (new_a == _output_a && new_b == _output_b) {
		_stable_count = math::min(_stable_count + 1, uint8_t(255));
	} else {
		_stable_count = 0;
		_output_a = new_a;
		_output_b = new_b;
	}

	// Require minimum stability
	return (_stable_count >= 2);
}

void SignalFilter::reset()
{
	_index = 0;
	_stable_count = 0;
	_output_a = false;
	_output_b = false;

	for (uint8_t i = 0; i < MAX_WINDOW; i++) {
		_samples_a[i] = false;
		_samples_b[i] = false;
	}
}

// ============================================================================
// QuadratureEncoder Implementation
// ============================================================================

QuadratureEncoder::QuadratureEncoder(uint8_t instance) :
	ModuleParams(nullptr),
	ScheduledWorkItem("quadrature_encoder", px4::wq_configurations::hp_default),
	_instance(instance),
	_filter(3)
{
	// Initialize performance counters
	char perf_name[32];
	snprintf(perf_name, sizeof(perf_name), "qenc_%u_cycle", _instance);
	_cycle_perf = perf_alloc(PC_ELAPSED, perf_name);

	snprintf(perf_name, sizeof(perf_name), "qenc_%u_int", _instance);
	_interrupt_perf = perf_alloc(PC_COUNT, perf_name);

	snprintf(perf_name, sizeof(perf_name), "qenc_%u_err", _instance);
	_error_perf = perf_alloc(PC_COUNT, perf_name);
}

QuadratureEncoder::~QuadratureEncoder()
{
	// Stop operation
	ScheduleClear();

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
	// Load board configuration
	if (_instance >= board_encoder_count) {
		PX4_ERR("Instance %u not supported (max %u)", _instance, board_encoder_count - 1);
		return false;
	}

	_config = board_encoder_configs[_instance];

	// Validate configuration
	if (_config.gpio_a == 0 || _config.gpio_b == 0 ||
	    _config.gpio_a == _config.gpio_b) {
		PX4_ERR("Invalid GPIO configuration for instance %u", _instance);
		return false;
	}

	// Update parameters
	update_parameters();

	// Initialize hardware
	if (!initialize_hardware()) {
		return false;
	}

	// Attach interrupts
	if (!attach_interrupts()) {
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
	uint32_t interval_us = 1_s / math::max(_param_update_rate.get(), 1);
	ScheduleOnInterval(interval_us);

	_initialized.store(true);

	PX4_INFO("Encoder %u initialized on pins %u/%u",
		_instance, _config.gpio_a, _config.gpio_b);

	return true;
}

bool QuadratureEncoder::initialize_hardware()
{
	// Configure GPIO A as input with pull-up and interrupt
	if (stm32_configgpio(_config.gpio_a | GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI) != 0) {
		PX4_ERR("Failed to configure GPIO A (0x%08x)", _config.gpio_a);
		return false;
	}

	// Configure GPIO B as input with pull-up (no interrupt)
	if (stm32_configgpio(_config.gpio_b | GPIO_INPUT | GPIO_PULLUP) != 0) {
		PX4_ERR("Failed to configure GPIO B (0x%08x)", _config.gpio_b);
		stm32_configgpio(_config.gpio_a | GPIO_INPUT | GPIO_FLOAT);
		return false;
	}

	// Read initial state
	bool a_state = stm32_gpioread(_config.gpio_a);
	bool b_state = stm32_gpioread(_config.gpio_b);
	_state.quadrature_state.store((b_state << 1) | a_state);

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
	int irq = stm32_gpioirq(_config.gpio_a);

	if (irq < 0) {
		PX4_ERR("Failed to get IRQ for GPIO A");
		return false;
	}

	if (irq_attach(irq, interrupt_handler, this) != 0) {
		PX4_ERR("Failed to attach interrupt handler");
		return false;
	}

	up_enable_irq(irq);
	_interrupts_attached.store(true);

	return true;
}

void QuadratureEncoder::detach_interrupts()
{
	if (!_interrupts_attached.load()) {
		return;
	}

	int irq = stm32_gpioirq(_config.gpio_a);

	if (irq >= 0) {
		up_disable_irq(irq);
		irq_detach(irq);
	}

	_interrupts_attached.store(false);
}

int QuadratureEncoder::interrupt_handler(int irq, void *context, void *arg)
{
	auto *instance = static_cast<QuadratureEncoder *>(arg);

	if (instance == nullptr || !instance->_initialized.load()) {
		return 0;
	}

	perf_count(instance->_interrupt_perf);

	// Read GPIO states
	bool a_state = stm32_gpioread(instance->_config.gpio_a);
	bool b_state = stm32_gpioread(instance->_config.gpio_b);

	// Process state change
	instance->process_state_change(a_state, b_state);

	return 0;
}

void QuadratureEncoder::process_state_change(bool a_state, bool b_state)
{
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
	uint8_t old_state = _state.quadrature_state.load();

	// Look up transition in state table
	uint8_t transition = (old_state << 2) | new_state;
	int8_t delta = _state_table[transition];

	if (delta == 2) {
		// Invalid transition - error
		_state.errors.fetch_add(1);
		perf_count(_error_perf);
	} else if (delta != 0) {
		// Valid transition
		if (_config.invert_direction) {
			delta = -delta;
		}

		_state.position.fetch_add(delta);
		_state.transitions.fetch_add(1);
		_state.last_update.store(hrt_absolute_time());
	}

	// Update quadrature state
	_state.quadrature_state.store(new_state);
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
	// Update instance-specific parameters
	_config.pulses_per_revolution = get_instance_ppr();
	_config.invert_direction = get_instance_invert();
	_config.enable_filtering = _param_filter_enabled.get();

	// Update filter configuration
	_filter = SignalFilter(_config.filter_window);
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
	int32_t current_position = _state.position.load();

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
		float filtered_velocity = alpha * velocity + (1.0f - alpha) * _state.velocity.load();

		_state.velocity.store(filtered_velocity);
	}

	// Update tracking variables
	_state.velocity_timestamp = now;
	_state.velocity_position = current_position;

	// Update cumulative angle
	float angle = current_position * (2.0f * M_PI_F) / _config.pulses_per_revolution;
	_state.angle.store(angle);
}

void QuadratureEncoder::check_health()
{
	hrt_abstime now = hrt_absolute_time();

	if (now - _last_health_check < HEALTH_CHECK_INTERVAL_US) {
		return;
	}

	_last_health_check = now;

	// Calculate error rate
	uint32_t transitions = _state.transitions.load();
	uint32_t errors = _state.errors.load();

	bool healthy = true;

	if (transitions > 0) {
		float error_rate = (100.0f * errors) / transitions;

		if (error_rate > _param_max_error_rate.get()) {
			PX4_WARN("Encoder %u: High error rate %.1f%%", _instance, (double)error_rate);
			healthy = false;
		}
	}

	// Check for recent updates
	uint64_t last_update = _state.last_update.load();
	if (now - last_update > 5_s) {
		healthy = false;  // No updates for 5 seconds
	}

	_state.healthy.store(healthy);
}

void QuadratureEncoder::publish_data()
{
	sensor_quad_encoder_s msg{};

	msg.timestamp = hrt_absolute_time();
	msg.device_id = _instance;
	msg.counts = _state.position.load();
	msg.angle_rad = _state.angle.load();
	msg.angular_velocity_rad_s = _state.velocity.load();
	msg.direction = _config.invert_direction ? -1 : 1;
	msg.resolution = _config.pulses_per_revolution;
	msg.errors = _state.errors.load();

	orb_publish(ORB_ID(sensor_quad_encoder), _encoder_pub, &msg);
}

void QuadratureEncoder::reset_position()
{
	_state.position.store(0);
	_state.angle.store(0.0f);
	_state.velocity.store(0.0f);
	_state.errors.store(0);
	_state.transitions.store(0);
	_state.velocity_position = 0;
	_state.velocity_timestamp = hrt_absolute_time();
	_filter.reset();

	PX4_INFO("Encoder %u position reset", _instance);
}

int QuadratureEncoder::print_status()
{
	PX4_INFO("Quadrature Encoder %u:", _instance);
	PX4_INFO("  GPIO A: 0x%08x", _config.gpio_a);
	PX4_INFO("  GPIO B: 0x%08x", _config.gpio_b);
	PX4_INFO("  PPR: %u", _config.pulses_per_revolution);
	PX4_INFO("  Inverted: %s", _config.invert_direction ? "yes" : "no");
	PX4_INFO("  Filtering: %s", _config.enable_filtering ? "enabled" : "disabled");

	PX4_INFO("  Position: %d counts", _state.position.load());
	PX4_INFO("  Angle: %.2f rad", (double)_state.angle.load());
	PX4_INFO("  Velocity: %.2f rad/s", (double)_state.velocity.load());
	PX4_INFO("  Healthy: %s", _state.healthy.load() ? "yes" : "no");

	uint32_t transitions = _state.transitions.load();
	uint32_t errors = _state.errors.load();

	PX4_INFO("  Transitions: %u", transitions);
	PX4_INFO("  Errors: %u", errors);

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
	int instance = 0;
	int ch;

	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:", nullptr, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			instance = atoi(myoptarg);
			break;
		default:
			print_usage("Unknown option");
			return -1;
		}
	}

	if (instance < 0 || instance >= MAX_INSTANCES) {
		PX4_ERR("Invalid instance: %d", instance);
		return -1;
	}

	if (_instances[instance] != nullptr) {
		PX4_ERR("Instance %d already running", instance);
		return -1;
	}

	// Create and initialize instance
	auto *encoder = new QuadratureEncoder(instance);

	if (encoder == nullptr) {
		PX4_ERR("Failed to allocate encoder instance");
		return -1;
	}

	if (!encoder->init()) {
		delete encoder;
		return -1;
	}

	_instances[instance] = encoder;

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

extern "C" __EXPORT int quadrature_encoder_main(int argc, char *argv[])
{
	return QuadratureEncoder::main(argc, argv);
}
