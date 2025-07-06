#include "quadrature_encoder.hpp"

// System includes
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>

// Standard includes
#include <string.h>
#include <stdlib.h>

// Initialize static members
QuadratureEncoder *QuadratureEncoder::_instances[ENCODER_MAX_INSTANCES] = {nullptr};
uint8_t QuadratureEncoder::_instance_count = 0;

QuadratureEncoder::QuadratureEncoder(uint8_t encoder_id) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_encoder_id(encoder_id),
	_platform_encoder_id(-1),
	_initialized(false),
	_running(false),
	_position_raw(0),
	_position_offset(0),
	_position_rad(0.0f),
	_position_deg(0.0f),
	_velocity_rad_s(0.0f),
	_velocity_rpm(0.0f),
	_direction_forward(true),
	_index_detected_this_revolution(false),
	_pulse_count(0),
	_direction_changes(0),
	_error_count(0),
	_last_update_time(0),
	_encoder_pub(ORB_ID(sensor_quad_encoder))
{
	// Initialize performance counters
	_cycle_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	_interval_perf = perf_alloc(PC_INTERVAL, MODULE_NAME": interval");
	_error_perf = perf_alloc(PC_COUNT, MODULE_NAME": errors");
}

QuadratureEncoder::~QuadratureEncoder()
{
	// Stop encoder if running
	if (_running && _platform_encoder_id != 0xFF) {
		quadrature_encoder_hal_stop(_platform_encoder_id);
	}

	// Free performance counters
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	perf_free(_error_perf);
}

bool QuadratureEncoder::init()
{
	// Update parameters first
	update_params();

	// Check if encoder is enabled
	if (!_param_enable.get()) {
		PX4_INFO("Encoder %d disabled by parameter", _encoder_id);
		return false;
	}

	// Configure encoder hardware
	_config.pulses_per_revolution = _param_pulses_per_revolution.get();
	_config.mode = (encoder_mode_t)_param_mode.get();
	_config.swap_channels = (_param_swap_channels.get() != 0);
	_config.enable_index = false;  // Index channel not supported yet

	// Get board-specific configuration
	const quadrature_encoder_hal_config_t *board_config = board_get_encoder_config(_encoder_id);
	if (!board_config) {
		PX4_ERR("Failed to get board configuration for encoder %d", _encoder_id);
		return false;
	}

	// Copy board configuration and apply parameter overrides
	_config = *board_config;

	// Create platform encoder instance
	int platform_id = quadrature_encoder_hal_create_instance(&_config);
	if (platform_id < 0) {
		PX4_ERR("Failed to create platform encoder instance: %d", platform_id);
		return false;
	}
	_platform_encoder_id = (uint8_t)platform_id;

	// Start scheduled work item
	ScheduleOnInterval(1000000 / (uint32_t)_param_rate.get(), 1000);

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

	bool data_updated = false;
	uint32_t updates_processed = 0;
	const uint32_t max_updates_per_cycle = 10; // Limit to prevent blocking too long

	// Process multiple updates in a loop to handle high-frequency events
	encoder_processed_data_t processed_data;
	while ((updates_processed < max_updates_per_cycle) &&
	       quadrature_encoder_hal_get_processed_data(_platform_encoder_id, &processed_data)) {

		// Update local state from processed data
		_position_raw = processed_data.position_raw;
		_position_rad = processed_data.position_rad;
		_position_deg = processed_data.position_deg;
		_velocity_rad_s = processed_data.velocity_rad_s;
		_velocity_rpm = processed_data.velocity_rpm;
		_direction_forward = processed_data.direction_forward;
		_index_detected_this_revolution = processed_data.index_detected;
		_pulse_count = processed_data.pulse_count;
		_direction_changes = processed_data.direction_changes;
		_error_count = processed_data.error_count;

		// Update timing
		_last_update_time = processed_data.timestamp;

		data_updated = true;
		updates_processed++;
	}

	// Publish encoder data only if we received updates
	if (data_updated) {
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
	encoder_msg.pulses_per_rev[0] = _config.pulses_per_revolution;
	encoder_msg.invert_direction[0] = _config.swap_channels;
	encoder_msg.gear_ratio[0] = 1.0f;  // Direct drive

	// Reset index detection flag
	_index_detected_this_revolution = false;

	// Publish message
	_encoder_pub.publish(encoder_msg);
}



void QuadratureEncoder::update_params()
{
	ModuleParams::updateParams();

	// Update schedule interval if rate parameter changed
	if (_param_rate.get() > 0) {
		ScheduleOnInterval(1000000 / (uint32_t)_param_rate.get(), 1000);
	}
}

// Static methods
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

	// Start the event processor thread if this is the first encoder
	if (_instance_count == 0) {
		int ret = quadrature_encoder_hal_start_event_processor();
		if (ret != 0) {
			PX4_ERR("Failed to start event processor thread (error: %d)", ret);
			if (ret == -1) {
				PX4_ERR("Hardware not initialized - check quadrature_encoder_hal_init()");
			} else if (ret == -2) {
				PX4_ERR("Thread creation failed - check system resources");
			} else if (ret == -3) {
				PX4_ERR("Semaphore initialization failed");
			}
			return -1;
		}
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

	// Start the encoder hardware
	if (quadrature_encoder_hal_start(encoder->_platform_encoder_id) != 0) {
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

	// Stop the event processor thread if this was the last encoder
	if (_instance_count == 0) {
		quadrature_encoder_hal_stop_event_processor();
	}

	PX4_INFO("Stopped encoder %d", encoder_id);

	return 0;
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
