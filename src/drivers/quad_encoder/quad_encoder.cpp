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

#include "quad_encoder.hpp"

#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>

// Static storage for multiple instances
static QuadEncoder *_objects[QuadEncoder::MAX_INSTANCES] = {};
static_assert(QuadEncoder::MAX_INSTANCES == 4, "Update _objects array size");

QuadEncoder::QuadEncoder(int instance_id, const char *device_path) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_instance_id(instance_id)
{
	// Store device path or use default
	if (device_path) {
		strncpy(_device_path, device_path, sizeof(_device_path) - 1);
		_device_path[sizeof(_device_path) - 1] = '\0';
	} else {
		// Default device path based on instance
		snprintf(_device_path, sizeof(_device_path), "/dev/qe%d", instance_id);
	}

	// Initialize encoder data
	_encoder_data = {};
	_prev_position = 0;
}

QuadEncoder::~QuadEncoder()
{
	// Unadvertise topics
	if (_sensor_quad_encoder_pub != nullptr) {
		orb_unadvertise(_sensor_quad_encoder_pub);
	}

	close_encoder();

	perf_free(_loop_perf);
	perf_free(_read_perf);
}

bool QuadEncoder::init()
{
	// Load initial parameters
	parameters_update();

	// Open encoder device
	if (!open_encoder()) {
		PX4_ERR("Failed to open encoder device %s", _device_path);
		return false;
	}

	// Initialize multi-instance publications
	sensor_quad_encoder_s sensor_msg{};
	sensor_msg.timestamp = hrt_absolute_time();

	// Advertise sensor encoder topic with instance
	_sensor_quad_encoder_pub = orb_advertise_multi(ORB_ID(sensor_quad_encoder), &sensor_msg,
		&_sensor_encoder_instance);

	if (_sensor_quad_encoder_pub == nullptr) {
		PX4_ERR("Failed to advertise sensor_quad_encoder");
		return false;
	}
	PX4_INFO("Advertising sensor_quad_encoder instance %d", _sensor_encoder_instance);

	// Reset encoder
	reset_encoder();

	// Start work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	_is_running = true;

	PX4_INFO("QuadEncoder initialized (instance %d, device %s)", _instance_id, _device_path);
	return true;
}

bool QuadEncoder::open_encoder()
{
	_fd_encoder = open(_device_path, O_RDONLY);

	if (_fd_encoder < 0) {
		PX4_ERR("Failed to open %s: %d", _device_path, errno);
		return false;
	} else {
		PX4_DEBUG("Opened encoder: %s (fd=%d)", _device_path, _fd_encoder);
	}

	return true;
}

void QuadEncoder::close_encoder()
{
	if (_fd_encoder >= 0) {
		close(_fd_encoder);
		_fd_encoder = -1;
	}
}

void QuadEncoder::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_is_running = false;
		return;
	}

	perf_begin(_loop_perf);

	// Check for parameter updates
	parameter_update_s param_update;
	if (_parameter_update_sub.update(&param_update)) {
		parameters_update();
	}

	// Read encoder data
	read_encoder();

	perf_end(_loop_perf);
}

void QuadEncoder::read_encoder()
{
	perf_begin(_read_perf);

	sensor_quad_encoder_s sensor_msg{};
	sensor_msg.timestamp = hrt_absolute_time();
	sensor_msg.count = 1;

	bool any_valid = false;

	if (_fd_encoder < 0) {
		_encoder_data.valid = false;
		sensor_msg.valid[0] = 0;
		perf_end(_read_perf);
		return;
	}

	// Read position from encoder using IOCTL
	int32_t position = 0;
	int ret = ioctl(_fd_encoder, QEIOC_POSITION, &position);

	if (ret == 0) {
		// Get encoder parameters for this specific instance
		int32_t ppr = get_ppr_for_instance();
		bool invert = get_invert_for_instance();

		if (invert) {
			position = -position;
		}

		_encoder_data.position = position;
		_encoder_data.timestamp = sensor_msg.timestamp;
		_encoder_data.valid = true;
		_encoder_data.pulses_per_rev = ppr;
		_encoder_data.invert_direction = invert;
		any_valid = true;

		// Calculate velocity and angle for rotary encoders
		float velocity = 0.0f;
		float angle_rad = 0.0f;

		if (_prev_timestamp > 0) {
			float dt = (sensor_msg.timestamp - _prev_timestamp) * 1e-6f;
			if (dt > 0.0f) {
				int32_t delta_pos = position - _prev_position;

				// Rotary encoder calculations
				float angular_velocity = (2.0f * static_cast<float>(M_PI) * delta_pos) / (ppr * dt); // rad/s
				velocity = angular_velocity;
				angle_rad = (2.0f * static_cast<float>(M_PI) * position) / ppr; // cumulative angle in radians

				_encoder_data.velocity_rad_s = velocity;
				_encoder_data.angle_rad = angle_rad;
			}
		}

		// Fill message
		sensor_msg.position[0] = position;
		sensor_msg.velocity[0] = _encoder_data.velocity_rad_s;
		sensor_msg.angle_or_distance[0] = _encoder_data.angle_rad;
		sensor_msg.valid[0] = 1;
		sensor_msg.pulses_per_rev[0] = ppr;
		sensor_msg.invert_direction[0] = invert;

		_prev_position = position;

	} else {
		_encoder_data.valid = false;
		sensor_msg.valid[0] = 0;
		_error_count++;
		PX4_DEBUG("Failed to read encoder: %d", ret);
	}

	// Fill remaining slots with invalid data
	for (int i = 1; i < sensor_quad_encoder_s::MAX_ENCODERS; i++) {
		sensor_msg.position[i] = 0;
		sensor_msg.velocity[i] = 0.0f;
		sensor_msg.angle_or_distance[i] = 0.0f;
		sensor_msg.valid[i] = 0;
		sensor_msg.pulses_per_rev[i] = 0;
		sensor_msg.invert_direction[i] = false;
	}

	if (any_valid && _sensor_quad_encoder_pub != nullptr) {
		orb_publish(ORB_ID(sensor_quad_encoder), _sensor_quad_encoder_pub, &sensor_msg);
		_read_count++;
	}

	_prev_timestamp = sensor_msg.timestamp;
	perf_end(_read_perf);
}

void QuadEncoder::reset_encoder()
{
	if (_fd_encoder >= 0) {
		// Reset encoder position
		if (ioctl(_fd_encoder, QEIOC_RESET, 0) < 0) {
			PX4_WARN("Failed to reset encoder");
		}
	}
	_prev_position = 0;

	_prev_timestamp = 0;

	PX4_INFO("Encoder reset");
}

void QuadEncoder::parameters_update()
{
	updateParams();

	// Update work schedule based on update rate parameter
	uint32_t update_interval_us = 1000000 / _param_update_rate.get();
	ScheduleOnInterval(update_interval_us);
}

int32_t QuadEncoder::get_ppr_for_instance() const
{
	switch (_instance_id) {
	case 0: return _param_ppr_0.get();
	case 1: return _param_ppr_1.get();
	case 2: return _param_ppr_2.get();
	case 3: return _param_ppr_3.get();
	default: return 1024; // Default fallback
	}
}

bool QuadEncoder::get_invert_for_instance() const
{
	switch (_instance_id) {
	case 0: return _param_invert_0.get();
	case 1: return _param_invert_1.get();
	case 2: return _param_invert_2.get();
	case 3: return _param_invert_3.get();
	default: return false; // Default fallback
	}
}

int QuadEncoder::print_status()
{
	PX4_INFO("QuadEncoder (instance %d) status:", _instance_id);
	PX4_INFO("  Running: %s", _is_running ? "yes" : "no");
	PX4_INFO("  Update rate: %ld Hz", _param_update_rate.get());
	PX4_INFO("  Device: %s", _device_path);

	// Show uORB topic instances
	PX4_INFO("  Sensor encoder topic instance: %d", _sensor_encoder_instance);

	PX4_INFO("Encoder status:");
	PX4_INFO("  Enc: %s, pos=%ld, vel=%.2f rad/s, angle=%.2f rad, ppr=%ld, invert=%s",
		_encoder_data.valid ? "OK" : "FAIL",
		(long)_encoder_data.position,
		(double)_encoder_data.velocity_rad_s,
		(double)_encoder_data.angle_rad,
		(long)_encoder_data.pulses_per_rev,
		_encoder_data.invert_direction ? "yes" : "no");

	PX4_INFO("Statistics:");
	PX4_INFO("  Read count: %lu", _read_count);
	PX4_INFO("  Error count: %lu", _error_count);

	if (_last_error_time > 0) {
		PX4_INFO("  Last error: %llu us ago", hrt_absolute_time() - _last_error_time);
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_read_perf);

	return 0;
}

int QuadEncoder::task_spawn(int argc, char *argv[])
{
	int instance_id = 0;
	const char *device_path = nullptr;

	// Parse command line arguments
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			instance_id = atoi(myoptarg);
			break;
		case 'd':
			device_path = myoptarg;
			break;
		case '?':
			PX4_WARN("Unknown option");
			return -1;
		}
	}

	// Validate instance ID
	if (instance_id < 0 || instance_id >= MAX_INSTANCES) {
		PX4_ERR("Instance ID must be between 0 and %d", MAX_INSTANCES - 1);
		return -1;
	}

	// Check if instance already exists
	if (_objects[instance_id] != nullptr) {
		PX4_ERR("Instance %d already running", instance_id);
		return -1;
	}

	// Create new instance
	QuadEncoder *instance = instantiate(instance_id, device_path);

	if (instance == nullptr) {
		PX4_ERR("Allocation failed");
		return -1;
	}

	// Store instance
	_objects[instance_id] = instance;

	// Store in ModuleBase for the main commands (use instance 0 as primary)
	if (instance_id == 0) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
	}

	if (!instance->init()) {
		delete instance;
		_objects[instance_id] = nullptr;
		if (instance_id == 0) {
			_object.store(nullptr);
			_task_id = -1;
		}
		return -1;
	}

	return 0;
}

QuadEncoder *QuadEncoder::instantiate(int instance, const char *device_path)
{
	QuadEncoder *obj = new QuadEncoder(instance, device_path);
	return obj;
}

int QuadEncoder::custom_command(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	const char *command = argv[1];

	if (!strcmp(command, "stop")) {
		// Parse instance ID
		int instance_id = 0;
		int myoptind = 2;
		int ch;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'i':
				instance_id = atoi(myoptarg);
				break;
			}
		}

		if (instance_id < 0 || instance_id >= MAX_INSTANCES) {
			PX4_ERR("Invalid instance ID");
			return 1;
		}

		if (_objects[instance_id] != nullptr) {
			_objects[instance_id]->request_stop();
			delete _objects[instance_id];
			_objects[instance_id] = nullptr;

			if (instance_id == 0) {
				_object.store(nullptr);
				_task_id = -1;
			}
			return 0;
		}

		PX4_WARN("Instance %d not running", instance_id);
		return 1;
	}

	if (!strcmp(command, "status")) {
		bool any_running = false;

		for (int i = 0; i < MAX_INSTANCES; i++) {
			if (_objects[i] != nullptr) {
				PX4_INFO("=== Instance %d ===", i);
				_objects[i]->print_status();
				any_running = true;
			}
		}

		if (!any_running) {
			PX4_INFO("No instances running");
		}

		return 0;
	}

	if (!strcmp(command, "reset")) {
		// Parse instance ID
		int instance_id = -1; // -1 means all instances
		int myoptind = 2;
		int ch;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'i':
				instance_id = atoi(myoptarg);
				break;
			}
		}

		if (instance_id >= 0) {
			// Reset specific instance
			if (instance_id >= MAX_INSTANCES) {
				PX4_ERR("Invalid instance ID");
				return 1;
			}

			if (_objects[instance_id] != nullptr) {
				_objects[instance_id]->reset_encoder();
				return 0;
			}

			PX4_WARN("Instance %d not running", instance_id);
			return 1;
		} else {
			// Reset all instances
			bool any_reset = false;
			for (int i = 0; i < MAX_INSTANCES; i++) {
				if (_objects[i] != nullptr) {
					_objects[i]->reset_encoder();
					any_reset = true;
				}
			}

			if (!any_reset) {
				PX4_WARN("No instances running");
				return 1;
			}

			return 0;
		}
	}

	return print_usage("unknown command");
}

int QuadEncoder::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GPIO-based quadrature encoder driver for rotary encoders.

This driver interfaces with the NuttX GPIO quadrature encoder framework
to provide encoder data for rotary applications including:
- Wheels and rotary actuators
- Motors and position sensors

Features:
- Single encoder per instance with configurable device path
- Multi-instance support (up to 4 instances) with independent uORB topics
- Configurable pulses per revolution and direction inversion

### Configuration
Configure each encoder instance using the following parameters:
- QE_PPR_0/1/2/3: Pulses per revolution for instance 0/1/2/3
- QE_INVERT_0/1/2/3: Invert direction for instance 0/1/2/3

### Examples
Start multiple instances for wheel loader configuration:
$ param set QE_PPR_0 1024      # Front/rear wheel encoder: 1024 PPR
$ param set QE_INVERT_0 0      # Wheel encoder: normal direction
$ param set QE_PPR_1 1024      # Bucket position encoder: 1024 PPR
$ param set QE_INVERT_1 0      # Bucket encoder: normal direction

# Front board: wheel + bucket encoders
$ quad_encoder start -i 0 -d /dev/qe0     # Front wheel encoder
$ quad_encoder start -i 1 -d /dev/qe1     # Bucket position encoder

# Rear board: wheel encoder only
$ quad_encoder start -i 0 -d /dev/qe0     # Rear wheel encoder

Reset encoder position:
$ quad_encoder reset           # Reset all instances
$ quad_encoder reset -i 1      # Reset specific instance

Check status of all instances:
$ quad_encoder status

Stop specific instance:
$ quad_encoder stop -i 1       # Stop specific instance (default: instance 0)
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("quad_encoder", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "Instance ID", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/qe0", "<file:dev>", "Encoder device path", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset encoder position");
	PRINT_MODULE_USAGE_PARAM_INT('i', -1, -1, 3, "Instance ID (-1 for all instances)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop specific instance");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "Instance ID", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void QuadEncoder::usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GPIO-based quadrature encoder driver for rotary encoders.

This driver interfaces with the NuttX GPIO quadrature encoder framework
to provide encoder data for rotary applications including:
- Wheels and rotary actuators
- Motors and position sensors

Features:
- Single encoder per instance with configurable device path
- Multi-instance support (up to 4 instances) with independent uORB topics
- Configurable pulses per revolution and direction inversion

### Configuration
Configure each encoder instance using the following parameters:
- QE_PPR_0/1/2/3: Pulses per revolution for instance 0/1/2/3
- QE_INVERT_0/1/2/3: Invert direction for instance 0/1/2/3

### Examples
Start multiple instances for wheel loader configuration:
$ param set QE_PPR_0 1024      # Front/rear wheel encoder: 1024 PPR
$ param set QE_INVERT_0 0      # Wheel encoder: normal direction
$ param set QE_PPR_1 1024      # Bucket position encoder: 1024 PPR
$ param set QE_INVERT_1 0      # Bucket encoder: normal direction

# Front board: wheel + bucket encoders
$ quad_encoder start -i 0 -d /dev/qe0     # Front wheel encoder
$ quad_encoder start -i 1 -d /dev/qe1     # Bucket position encoder

# Rear board: wheel encoder only
$ quad_encoder start -i 0 -d /dev/qe0     # Rear wheel encoder

Reset encoder position:
$ quad_encoder reset           # Reset all instances
$ quad_encoder reset -i 1      # Reset specific instance

Check status:
$ quad_encoder status

Stop the driver:
$ quad_encoder stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("quad_encoder", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "Instance ID", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/qe0", "<file:dev>", "Encoder device path", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset encoder position");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int quad_encoder_main(int argc, char *argv[])
{
	return QuadEncoder::main(argc, argv);
}
