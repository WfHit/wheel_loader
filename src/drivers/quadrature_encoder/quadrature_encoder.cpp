#include "quadrature_encoder.hpp"

// System includes
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/param.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

// Standard includes
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

QuadratureEncoder::QuadratureEncoder() :
	ModuleBase<QuadratureEncoder>(),
	ModuleParams(nullptr),
	ScheduledWorkItem("quadrature_encoder", px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, "quadrature_encoder: cycle")),
	_encoder_perf(perf_alloc(PC_ELAPSED, "quadrature_encoder: encoder"))
{
	// Initialize encoder channels
	for (int i = 0; i < sensor_quad_encoder_s::MAX_ENCODERS; i++) {
		_encoders[i] = {};
	}
}

QuadratureEncoder::~QuadratureEncoder()
{
	// Stop all encoders
	for (int i = 0; i < _num_encoders; i++) {
		if (_encoders[i].initialized) {
			quad_encoder_stop(i);
		}
	}

	// Free performance counters
	perf_free(_loop_perf);
	perf_free(_encoder_perf);
}

bool QuadratureEncoder::init()
{
	PX4_INFO("QuadratureEncoder::init() starting...");

	// Load initial parameters
	parameters_update();
	PX4_INFO("Parameters updated");

	// Get number of available encoders from board configuration
	PX4_INFO("Calling board_get_max_encoders()...");
	_num_encoders = board_get_max_encoders();
	PX4_INFO("Board reports %d available encoders", _num_encoders);

	if (_num_encoders == 0) {
		PX4_ERR("No quadrature encoders available on this board");
		return false;
	}

	if (_num_encoders > sensor_quad_encoder_s::MAX_ENCODERS) {
		PX4_WARN("Board has %d encoders, limiting to %d", _num_encoders, sensor_quad_encoder_s::MAX_ENCODERS);
		_num_encoders = sensor_quad_encoder_s::MAX_ENCODERS;
	}

	// Configure each available encoder
	bool any_configured = false;
	for (int i = 0; i < _num_encoders; i++) {
		PX4_INFO("Attempting to configure encoder %d", i);
		if (configure_encoder(i)) {
			any_configured = true;
			PX4_INFO("Successfully configured encoder %d", i);
		} else {
			PX4_WARN("Failed to configure encoder %d", i);
		}
	}

	if (!any_configured) {
		PX4_ERR("Failed to configure any encoders");
		return false;
	}

	PX4_INFO("Successfully configured %d encoder(s)", any_configured ? 1 : 0);

	// Start periodic updates
	int32_t rate = _param_rate.get();
	PX4_INFO("Setting update rate to %ld Hz", (long)rate);
	if (rate > 0) {
		ScheduleOnInterval(1000000 / (uint32_t)rate, 1000);
	} else {
		ScheduleOnInterval(SCHEDULE_INTERVAL);
	}

	_is_running = true;
	PX4_INFO("Module _is_running set to true");

	PX4_INFO("QuadratureEncoder initialized with %d channels", _num_encoders);
	PX4_INFO("QuadratureEncoder::init() completed successfully");
	return true;
}

bool QuadratureEncoder::configure_encoder(int encoder_id)
{
	PX4_INFO("configure_encoder: Starting configuration for encoder %d", encoder_id);

	if (encoder_id >= sensor_quad_encoder_s::MAX_ENCODERS) {
		PX4_ERR("configure_encoder: Invalid encoder_id %d (max: %d)", encoder_id, sensor_quad_encoder_s::MAX_ENCODERS);
		return false;
	}

	EncoderChannel &enc = _encoders[encoder_id];

	// Initialize parameters for this encoder
	init_encoder_parameters(encoder_id);
	PX4_INFO("configure_encoder: Parameters initialized for encoder %d", encoder_id);

	// Check if encoder is enabled
	int enabled = get_param_int(enc.param_enabled, 1);
	PX4_INFO("configure_encoder: Encoder %d enabled parameter = %d", encoder_id, enabled);
	if (enabled == 0) {
		PX4_INFO("Encoder %d disabled by parameter", encoder_id);
		return false;
	}

	// Set mode from parameter
	enc.mode = (encoder_mode_t)get_param_int(enc.param_mode, ENCODER_MODE_RELATIVE);

	// Initialize state variables
	enc.position_raw = 0;
	enc.position_rad = 0.0f;
	enc.velocity_rad_s = 0.0f;
	enc.direction_forward = true;
	enc.pulse_count = 0;
	enc.reset_count = 0;
	enc.last_update_time = 0;

	// Start the encoder hardware
	PX4_INFO("configure_encoder: Starting hardware for encoder %d", encoder_id);
	int ret = quad_encoder_start(encoder_id);
	PX4_INFO("configure_encoder: quad_encoder_start(%d) returned %d", encoder_id, ret);
	if (ret != 0) {
		PX4_ERR("Failed to start encoder %d hardware (error: %d)", encoder_id, ret);
		return false;
	}

	enc.initialized = true;
	enc.enabled = true;

	PX4_INFO("Configured encoder %d (%s)", encoder_id, board_get_encoder_name(encoder_id));
	PX4_INFO("configure_encoder: Successfully completed for encoder %d", encoder_id);
	return true;
}

void QuadratureEncoder::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_is_running = false;
		return;
	}

	perf_begin(_loop_perf);

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_update();
	}

	// Process all encoder channels
	process_encoders();

	// Publish combined encoder data
	publish_encoder_data();

	perf_end(_loop_perf);
}

void QuadratureEncoder::process_encoders()
{
	perf_begin(_encoder_perf);

	for (int i = 0; i < _num_encoders; i++) {
		if (!_encoders[i].initialized || !_encoders[i].enabled) {
			continue;
		}

		encoder_raw_data_t raw_data;
		if (quad_encoder_get_raw_data(i, &raw_data)) {
			process_encoder_data(i, raw_data);
		}
	}

	perf_end(_encoder_perf);
}

void QuadratureEncoder::process_encoder_data(int encoder_id, const encoder_raw_data_t &raw_data)
{
	EncoderChannel &enc = _encoders[encoder_id];

	// Calculate time delta for velocity calculation
	uint64_t current_time = raw_data.timestamp;
	uint64_t time_delta = current_time - enc.last_update_time;

	if (enc.last_update_time == 0) {
		// First measurement - initialize
		enc.position_raw = raw_data.counter;
		enc.last_update_time = current_time;
		enc.direction_forward = raw_data.direction_forward;
		enc.pulse_count = 0;
		enc.reset_count = 0;
		return;
	}

	// Detect direction changes
	if (enc.direction_forward != raw_data.direction_forward) {
		enc.direction_forward = raw_data.direction_forward;
	}

	// Calculate position delta, accounting for counter resets
	int64_t position_delta;

	if (raw_data.counter_reset) {
		// Counter was reset, calculate delta considering the wrap-around
		int32_t ppr = get_param_int(enc.param_pulses_per_revolution, 1024);

		if (raw_data.reset_direction_forward) {
			// Forward reset: counter went from (resolution-1) to 0
			position_delta = (raw_data.counter + ppr) - enc.position_raw;
		} else {
			// Reverse reset: counter went from 0 to (resolution-1)
			position_delta = raw_data.counter - (enc.position_raw + ppr);
		}

		enc.reset_count++; // Track number of resets
	} else {
		// Normal case: no reset occurred
		position_delta = raw_data.counter - enc.position_raw;
	}

	enc.position_raw = raw_data.counter;
	enc.pulse_count += abs(position_delta);

	// Calculate absolute position
	int64_t adjusted_position = enc.position_raw;

	// Handle absolute mode reset
	if (enc.mode == ENCODER_MODE_ABSOLUTE) {
		// In absolute mode, position resets to 0 at each revolution
		adjusted_position = enc.position_raw;
	}

	// Convert to radians
	int32_t ppr = get_param_int(enc.param_pulses_per_revolution, 1024);
	enc.position_rad = (2.0f * (float)M_PI * adjusted_position) / ppr;

	// Calculate velocity if we have sufficient time delta
	if (time_delta > 0) {
		// Calculate velocity in rad/s
		float position_change_rad = (2.0f * (float)M_PI * position_delta) / ppr;
		float time_delta_s = time_delta / 1000000.0f; // Convert microseconds to seconds

		// Apply simple low-pass filter to velocity
		float new_velocity = position_change_rad / time_delta_s;
		float filter_alpha = get_param_float(enc.param_vel_filter, 0.1f);

		if (filter_alpha > 0.0f && filter_alpha < 1.0f) {
			enc.velocity_rad_s = (1.0f - filter_alpha) * enc.velocity_rad_s + filter_alpha * new_velocity;
		} else {
			enc.velocity_rad_s = new_velocity;
		}
	}

	// Update timing
	enc.last_update_time = current_time;
}

void QuadratureEncoder::publish_encoder_data()
{
	sensor_quad_encoder_s encoder_msg{};

	encoder_msg.timestamp = hrt_absolute_time();

	// Count enabled encoders
	int enabled_count = 0;
	for (int i = 0; i < _num_encoders && enabled_count < sensor_quad_encoder_s::MAX_ENCODERS; i++) {
		if (_encoders[i].initialized && _encoders[i].enabled) {
			encoder_msg.position[enabled_count] = _encoders[i].position_raw;
			encoder_msg.velocity[enabled_count] = _encoders[i].velocity_rad_s;
			encoder_msg.angle_or_distance[enabled_count] = _encoders[i].position_rad;
			encoder_msg.valid[enabled_count] = (_encoders[i].pulse_count > 0) ? 1 : 0;
			encoder_msg.encoder_type[enabled_count] = sensor_quad_encoder_s::TYPE_ROTARY;
			encoder_msg.pulses_per_rev[enabled_count] = get_param_int(_encoders[i].param_pulses_per_revolution, 1024);
			encoder_msg.invert_direction[enabled_count] = false; // Channel swapping handled by platform
			encoder_msg.gear_ratio[enabled_count] = 1.0f;  // Direct drive
			enabled_count++;
		}
	}

	encoder_msg.count = enabled_count;

	// Publish message
	_encoder_pub.publish(encoder_msg);
}

void QuadratureEncoder::parameters_update()
{
	updateParams();

	// Update schedule interval if rate parameter changed
	int32_t rate = _param_rate.get();
	if (rate > 0) {
		ScheduleOnInterval(1000000 / (uint32_t)rate, 1000);
	}

	// Update encoder configurations if parameters changed
	for (int i = 0; i < _num_encoders; i++) {
		if (_encoders[i].initialized) {
			// Check if encoder was enabled/disabled
			bool enabled = get_param_int(_encoders[i].param_enabled, 1) != 0;
			if (enabled != _encoders[i].enabled) {
				_encoders[i].enabled = enabled;
				if (enabled) {
					PX4_INFO("Encoder %d enabled", i);
				} else {
					PX4_INFO("Encoder %d disabled", i);
				}
			}

			// Update mode
			_encoders[i].mode = (encoder_mode_t)get_param_int(_encoders[i].param_mode, ENCODER_MODE_RELATIVE);
		}
	}
}

void QuadratureEncoder::init_encoder_parameters(int encoder_id)
{
	char param_name[32];
	EncoderChannel &enc = _encoders[encoder_id];

	// Initialize parameter handles with instance-specific names
	snprintf(param_name, sizeof(param_name), "QENC%d_PPR", encoder_id);
	enc.param_pulses_per_revolution = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "QENC%d_MODE", encoder_id);
	enc.param_mode = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "QENC%d_VEL_FILT", encoder_id);
	enc.param_vel_filter = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "QENC%d_EN", encoder_id);
	enc.param_enabled = param_find(param_name);
}

int32_t QuadratureEncoder::get_param_int(param_t param_handle, int32_t default_value) const
{
	int32_t value = default_value;

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &value);
	}

	return value;
}

float QuadratureEncoder::get_param_float(param_t param_handle, float default_value) const
{
	float value = default_value;

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &value);
	}

	return value;
}

int QuadratureEncoder::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GPIO-based quadrature encoder driver for multiple encoder channels.

Supports both relative and absolute positioning modes with configurable
resolution and velocity filtering. All encoder channels are managed by a
single driver instance.

### Configuration
Configure each encoder using the following parameters:
- QENC<N>_EN: Enable encoder N (0=disabled, 1=enabled)
- QENC<N>_PPR: Pulses per revolution for encoder N
- QENC<N>_MODE: Mode for encoder N (0=relative, 1=absolute)
- QENC<N>_VEL_FILT: Velocity filter alpha for encoder N (0.0-1.0)
- QENC_RATE: Update rate in Hz (default: 50)

### Examples
Start the driver:
$ quadrature_encoder start

Check status:
$ quadrature_encoder status

Stop the driver:
$ quadrature_encoder stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("quadrature_encoder", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int QuadratureEncoder::task_spawn(int argc, char *argv[])
{
	PX4_INFO("QuadratureEncoder::task_spawn: Starting...");

	QuadratureEncoder *instance = new QuadratureEncoder();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	PX4_INFO("QuadratureEncoder::task_spawn: Instance created, storing object...");
	_object.store(instance);
	_task_id = task_id_is_work_queue;
	PX4_INFO("QuadratureEncoder::task_spawn: _task_id set to %d (task_id_is_work_queue)", _task_id);

	PX4_INFO("QuadratureEncoder::task_spawn: Calling init()...");
	if (instance->init()) {
		PX4_INFO("QuadratureEncoder::task_spawn: init() successful, scheduling now...");
		instance->ScheduleNow();
		PX4_INFO("QuadratureEncoder::task_spawn: Successfully spawned and scheduled");
		return 0;
	}

	PX4_ERR("QuadratureEncoder::task_spawn: init() failed, cleaning up...");
	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return -1;
}

QuadratureEncoder *QuadratureEncoder::instantiate(int argc, char *argv[])
{
	return new QuadratureEncoder();
}

int QuadratureEncoder::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int QuadratureEncoder::print_status()
{
	PX4_INFO("QuadratureEncoder Status:");
	PX4_INFO("Running: %s", _is_running ? "yes" : "no");
	PX4_INFO("Update rate: %ld Hz", (long)_param_rate.get());
	PX4_INFO("Available encoders: %d", _num_encoders);

	for (int i = 0; i < _num_encoders; i++) {
		const EncoderChannel &enc = _encoders[i];
		PX4_INFO("Encoder %d (%s):", i, board_get_encoder_name(i));
		PX4_INFO("  Initialized: %s", enc.initialized ? "Yes" : "No");
		PX4_INFO("  Enabled: %s", enc.enabled ? "Yes" : "No");
		if (enc.initialized) {
			PX4_INFO("  Position: %lld counts (%.3f rad)", (long long)enc.position_raw, (double)enc.position_rad);
			PX4_INFO("  Velocity: %.3f rad/s", (double)enc.velocity_rad_s);
			PX4_INFO("  Pulse count: %llu", (unsigned long long)enc.pulse_count);
			PX4_INFO("  Reset count: %lu", (unsigned long)enc.reset_count);
			PX4_INFO("  PPR: %ld", (long)get_param_int(enc.param_pulses_per_revolution, 1024));
			PX4_INFO("  Mode: %s", enc.mode == ENCODER_MODE_RELATIVE ? "Relative" : "Absolute");
		}
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_encoder_perf);

	return 0;
}
