#include "quadrature_encoder.hpp"

// System includes
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/param.h>
#include <drivers/drv_hrt.h>

// Standard includes
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Initialize static members
QuadratureEncoder *QuadratureEncoder::_instances[ENCODER_MAX_INSTANCES] = {nullptr};
uint8_t QuadratureEncoder::_instance_count = 0;

QuadratureEncoder::QuadratureEncoder(uint8_t encoder_id) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_encoder_id(encoder_id),
	_initialized(false),
	_running(false),
	_position_raw(0),
	_position_rad(0.0f),
	_velocity_rad_s(0.0f),
	_direction_forward(true),
	_pulse_count(0),
	_reset_count(0),
	_last_update_time(0),
	_encoder_pub(ORB_ID(sensor_quad_encoder)),
	_param_pulses_per_revolution_handle(PARAM_INVALID),
	_param_mode_handle(PARAM_INVALID),
	_param_vel_filter_handle(PARAM_INVALID),
	_param_rate_handle(PARAM_INVALID)
{
	// Initialize performance counters
	_cycle_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	_interval_perf = perf_alloc(PC_INTERVAL, MODULE_NAME": interval");
	_error_perf = perf_alloc(PC_COUNT, MODULE_NAME": errors");
}

QuadratureEncoder::~QuadratureEncoder()
{
	// Stop encoder if running
	if (_running) {
		quad_encoder_stop(_encoder_id);
	}

	// Free performance counters
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	perf_free(_error_perf);
}

bool QuadratureEncoder::init()
{
	// Initialize instance-specific parameters
	init_instance_parameters();

	// Configure encoder hardware
	update_params();

	// Get board-specific configuration
	const quad_encoder_config_t *board_config = board_get_encoder_config(_encoder_id);
	if (!board_config) {
		PX4_ERR("Failed to get board configuration for encoder %d", _encoder_id);
		return false;
	}

	// Set mode from parameter
	_mode = (encoder_mode_t)get_param_int(_param_mode_handle, ENCODER_MODE_RELATIVE);

	// Initialize state variables
	_position_raw = 0;
	_position_rad = 0.0f;
	_velocity_rad_s = 0.0f;
	_direction_forward = true;
	_pulse_count = 0;
	_last_update_time = 0;

	// Start scheduled work item with default rate
	int32_t rate = get_param_int(_param_rate_handle, 50); // Default 50 Hz
	if (rate > 0) {
		ScheduleOnInterval(1000000 / (uint32_t)rate, 1000);
	}

	_initialized = true;
	return true;
}

void QuadratureEncoder::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// Get raw data from platform (timestamp, counter, direction only)
	encoder_raw_data_t raw_data;

	if (quad_encoder_get_raw_data(_encoder_id, &raw_data)) {
		process_raw_encoder_data(raw_data);
		publish_encoder_data();
	}

	perf_end(_cycle_perf);
}

void QuadratureEncoder::publish_encoder_data()
{
	sensor_quad_encoder_s encoder_msg{};

	encoder_msg.timestamp = hrt_absolute_time();
	encoder_msg.count = 1;  // Single encoder per message

	// Set data for this encoder instance (index 0 since count=1)
	encoder_msg.position[0] = _position_raw;
	encoder_msg.velocity[0] = _velocity_rad_s;
	encoder_msg.angle_or_distance[0] = _position_rad;
	encoder_msg.valid[0] = (_running && _pulse_count > 0) ? 1 : 0;
	encoder_msg.encoder_type[0] = sensor_quad_encoder_s::TYPE_ROTARY;
	encoder_msg.pulses_per_rev[0] = get_param_int(_param_pulses_per_revolution_handle, 1024);
	encoder_msg.invert_direction[0] = false; // Channel swapping handled by platform
	encoder_msg.gear_ratio[0] = 1.0f;  // Direct drive

	// Publish message
	_encoder_pub.publish(encoder_msg);
}

void QuadratureEncoder::update_params()
{
	// Update schedule interval if rate parameter changed
	int32_t rate = get_param_int(_param_rate_handle, 50); // Default 50 Hz

	if (rate > 0) {
		ScheduleOnInterval(1000000 / (uint32_t)rate, 1000);
	}
}

int QuadratureEncoder::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GPIO-based quadrature encoder driver for multiple encoder instances.

Supports both relative and absolute positioning modes with configurable
resolution and channel swapping.

### Examples
Start all available encoders:
$ quadrature_encoder start

Start encoder 0 specifically:
$ quadrature_encoder start -i 0

Stop encoder 0:
$ quadrature_encoder stop -i 0

Show status:
$ quadrature_encoder status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("quadrature_encoder", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start encoder instance(s)");
	PRINT_MODULE_USAGE_PARAM_INT('i', -1, 0, 15, "Encoder ID (omit to start all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop encoder instance");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 15, "Encoder ID", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Show driver status");

	return 0;
}

int QuadratureEncoder::task_spawn(int argc, char *argv[])
{
	// Parse encoder ID from arguments
	int encoder_id = 0;
	bool error_flag = false;
	bool encoder_id_set = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			encoder_id = atoi(myoptarg);
			encoder_id_set = true;
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
		return print_usage("invalid arguments");
	}

	// If no encoder ID specified, start all available encoders
	if (!encoder_id_set) {
		PX4_INFO("Starting all available quadrature encoders");

		// Get number of available encoders from board configuration
		int num_encoders = board_get_max_encoders();

		if (num_encoders == 0) {
			PX4_ERR("No quadrature encoders available on this board");
			return -1;
		}

		bool any_started = false;

		// Start all available encoders
		for (int i = 0; i < num_encoders; i++) {
			if (start_instance(static_cast<uint8_t>(i)) == 0) {
				any_started = true;
			}
		}

		if (!any_started) {
			PX4_ERR("Failed to start any quadrature encoders");
			return -1;
		}

		PX4_INFO("Started %d quadrature encoder(s)", num_encoders);
		return 0;
	}

	// Check if encoder ID is valid
	if (encoder_id < 0 || encoder_id >= board_get_max_encoders()) {
		PX4_ERR("Invalid encoder ID: %d (available: 0-%d)", encoder_id, board_get_max_encoders() - 1);
		return -1;
	}

	// Start specific encoder instance
	return start_instance(static_cast<uint8_t>(encoder_id));
}

int QuadratureEncoder::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	if (!strcmp(argv[0], "start")) {
		// Parse encoder ID with -i option or as direct argument
		int encoder_id = 0;
		bool encoder_id_set = false;

		// Check for -i option
		for (int i = 1; i < argc - 1; i++) {
			if (!strcmp(argv[i], "-i")) {
				encoder_id = atoi(argv[i + 1]);
				encoder_id_set = true;
				break;
			}
		}

		// If no -i option, check if encoder ID is provided as direct argument
		if (!encoder_id_set && argc >= 2) {
			encoder_id = atoi(argv[1]);
			encoder_id_set = true;
		}

		if (!encoder_id_set) {
			return print_usage("missing encoder ID");
		}

		return start_instance(encoder_id);
	}

	if (!strcmp(argv[0], "stop")) {
		// Parse encoder ID with -i option or as direct argument
		int encoder_id = 0;
		bool encoder_id_set = false;

		// Check for -i option
		for (int i = 1; i < argc - 1; i++) {
			if (!strcmp(argv[i], "-i")) {
				encoder_id = atoi(argv[i + 1]);
				encoder_id_set = true;
				break;
			}
		}

		// If no -i option, check if encoder ID is provided as direct argument
		if (!encoder_id_set && argc >= 2) {
			encoder_id = atoi(argv[1]);
			encoder_id_set = true;
		}

		if (!encoder_id_set) {
			return print_usage("missing encoder ID");
		}

		return stop_instance(encoder_id);
	}

	if (!strcmp(argv[0], "status")) {
		PX4_INFO("Quadrature Encoder Status:");
		PX4_INFO("Active instances: %d", _instance_count);

		for (int i = 0; i < ENCODER_MAX_INSTANCES; i++) {
			if (_instances[i]) {
				PX4_INFO("  Encoder %d: %s - Running", i, board_get_encoder_name(i));
			}
		}

		return 0;
	}

	return print_usage("unknown command");
}

QuadratureEncoder *QuadratureEncoder::get_instance(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return nullptr;
	}

	return _instances[encoder_id];
}

int QuadratureEncoder::start_instance(uint8_t encoder_id)
{
	if (encoder_id >= board_get_max_encoders()) {
		PX4_ERR("Invalid encoder ID: %d", encoder_id);
		return -1;
	}

	// Check if instance already exists
	if (_instances[encoder_id] != nullptr) {
		PX4_WARN("Encoder %d already running", encoder_id);
		return 0;
	}

	// Create new instance
	QuadratureEncoder *encoder = new QuadratureEncoder(encoder_id);

	if (!encoder) {
		PX4_ERR("Failed to allocate encoder %d", encoder_id);
		return -1;
	}

	// Initialize encoder
	if (!encoder->init()) {
		delete encoder;
		return -1;
	}

	// Start the encoder hardware (interrupts)
	if (quad_encoder_start(encoder_id) != 0) {
		PX4_ERR("Failed to start encoder %d", encoder_id);
		delete encoder;
		return -1;
	}

	encoder->_running = true;
	_instances[encoder_id] = encoder;
	_instance_count++;

	PX4_INFO("Started encoder %d (%s)", encoder_id, board_get_encoder_name(encoder_id));

	return 0;
}

int QuadratureEncoder::stop_instance(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return -1;
	}

	QuadratureEncoder *encoder = _instances[encoder_id];

	if (!encoder) {
		return -1;
	}

	encoder->_running = false;
	_instances[encoder_id] = nullptr;
	_instance_count--;

	delete encoder;

	PX4_INFO("Stopped encoder %d", encoder_id);

	return 0;
}

void QuadratureEncoder::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// Get raw data from platform (timestamp, counter, direction only)
	encoder_raw_data_t raw_data;

	if (quad_encoder_get_raw_data(_encoder_id, &raw_data)) {
		process_raw_encoder_data(raw_data);
		publish_encoder_data();
	}

	perf_end(_cycle_perf);
}

void QuadratureEncoder::process_raw_encoder_data(const encoder_raw_data_t &raw_data)
{
	// Calculate time delta for velocity calculation
	uint64_t current_time = raw_data.timestamp;
	uint64_t time_delta = current_time - _last_update_time;

	if (_last_update_time == 0) {
		// First measurement - initialize
		_position_raw = raw_data.counter;
		_last_update_time = current_time;
		_direction_forward = raw_data.direction_forward;
		_pulse_count = 0;
		_reset_count = 0;
		return;
	}

	// Detect direction changes
	if (_direction_forward != raw_data.direction_forward) {
		_direction_forward = raw_data.direction_forward;
	}

	// Calculate position delta, accounting for counter resets
	int64_t position_delta;

	if (raw_data.counter_reset) {
		// Counter was reset, calculate delta considering the wrap-around
		int32_t ppr = get_param_int(_param_pulses_per_revolution_handle, 1024);

		if (raw_data.reset_direction_forward) {
			// Forward reset: counter went from (resolution-1) to 0
			position_delta = (raw_data.counter + ppr) - _position_raw;
		} else {
			// Reverse reset: counter went from 0 to (resolution-1)
			position_delta = raw_data.counter - (_position_raw + ppr);
		}

		_reset_count++; // Track number of resets
	} else {
		// Normal case: no reset occurred
		position_delta = raw_data.counter - _position_raw;
	}

	_position_raw = raw_data.counter;
	_pulse_count += abs(position_delta);

	// Calculate absolute position
	int64_t adjusted_position = _position_raw;

	// Handle absolute mode reset
	if (_mode == ENCODER_MODE_ABSOLUTE) {
		// In absolute mode, position resets to 0 at each revolution
		adjusted_position = _position_raw;
	}

	// Convert to radians
	int32_t ppr = get_param_int(_param_pulses_per_revolution_handle, 1024);
	_position_rad = (2.0f * M_PI * adjusted_position) / ppr;

	// Calculate velocity if we have sufficient time delta
	if (time_delta > 0) {
		// Calculate velocity in rad/s
		float position_change_rad = (2.0f * M_PI * position_delta) / ppr;
		float time_delta_s = time_delta / 1000000.0f; // Convert microseconds to seconds

		// Apply simple low-pass filter to velocity
		float new_velocity = position_change_rad / time_delta_s;
		float filter_alpha = get_param_float(_param_vel_filter_handle, 0.1f);

		if (filter_alpha > 0.0f && filter_alpha < 1.0f) {
			_velocity_rad_s = (1.0f - filter_alpha) * _velocity_rad_s + filter_alpha * new_velocity;
		} else {
			_velocity_rad_s = new_velocity;
		}
	}

	// Update timing
	_last_update_time = current_time;
}

void QuadratureEncoder::init_instance_parameters()
{
	char param_name[32];

	// Initialize parameter handles with instance-specific names
	snprintf(param_name, sizeof(param_name), "QENC%d_PPR", _encoder_id);
	_param_pulses_per_revolution_handle = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "QENC%d_MODE", _encoder_id);
	_param_mode_handle = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "QENC%d_VEL_FILT", _encoder_id);
	_param_vel_filter_handle = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "QENC%d_RATE", _encoder_id);
	_param_rate_handle = param_find(param_name);
}

int32_t QuadratureEncoder::get_param_int(param_t param_handle, int32_t default_value)
{
	int32_t value = default_value;

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &value);
	}

	return value;
}

float QuadratureEncoder::get_param_float(param_t param_handle, float default_value)
{
	float value = default_value;

	if (param_handle != PARAM_INVALID) {
		param_get(param_handle, &value);
	}

	return value;
}
