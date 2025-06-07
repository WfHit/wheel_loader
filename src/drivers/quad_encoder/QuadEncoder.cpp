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

#include "QuadEncoder.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Static storage for multiple instances
static QuadEncoder *_objects[QuadEncoder::MAX_INSTANCES] = {};
static_assert(QuadEncoder::MAX_INSTANCES == 4, "Update _objects array size");

QuadEncoder::QuadEncoder(int instance_id) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_instance_id(instance_id)
{
	// Initialize encoder data
	for (int i = 0; i < MAX_ENCODERS; i++) {
		_encoder_data[i] = {};
		_prev_position[i] = 0;
	}
}

QuadEncoder::~QuadEncoder()
{
	// Unadvertise topics
	if (_sensor_quad_encoder_pub != nullptr) {
		orb_unadvertise(_sensor_quad_encoder_pub);
	}

	close_encoders();

	perf_free(_loop_perf);
	perf_free(_read_perf);
}

bool QuadEncoder::init()
{
	// Load initial parameters
	parameters_update();

	// Determine number of active encoders
	_num_active_encoders = _param_num_encoders.get();
	if (_num_active_encoders == 0) {
		// Auto-detect available encoders
		_num_active_encoders = 0;
		for (int i = 0; i < MAX_ENCODERS; i++) {
			int fd = open(ENCODER_DEVICE_PATHS[i], O_RDONLY);
			if (fd >= 0) {
				close(fd);
				_num_active_encoders = i + 1;
			} else {
				break; // Stop at first unavailable encoder
			}
		}
		if (_num_active_encoders == 0) {
			_num_active_encoders = 4; // Default fallback
		}
		PX4_INFO("Auto-detected %d encoders", _num_active_encoders);
	}

	// Open encoder devices
	if (!open_encoders()) {
		PX4_ERR("Failed to open encoder devices");
		return false;
	}

	// Initialize multi-instance publications
	sensor_quad_encoder_s sensor_msg{};
	sensor_msg.timestamp = hrt_absolute_time();

	// Advertise sensor encoder topic with instance
	_sensor_quad_encoder_pub = orb_advertise_multi(ORB_ID(sensor_quad_encoder), &sensor_msg,
		&_sensor_encoder_instance, ORB_PRIO_DEFAULT);

	if (_sensor_quad_encoder_pub == nullptr) {
		PX4_ERR("Failed to advertise sensor_quad_encoder");
		return false;
	}
	PX4_INFO("Advertising sensor_quad_encoder instance %d", _sensor_encoder_instance);

	// Reset encoders
	reset_encoders();

	// Start work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	_is_running = true;

	PX4_INFO("QuadEncoder initialized (instance %d, %d encoders)", _instance_id, _num_active_encoders);
	return true;
}

bool QuadEncoder::open_encoders()
{
	bool success = true;

	for (int i = 0; i < _num_active_encoders; i++) {
		_fd_encoders[i] = open(ENCODER_DEVICE_PATHS[i], O_RDONLY);

		if (_fd_encoders[i] < 0) {
			PX4_ERR("Failed to open %s: %d", ENCODER_DEVICE_PATHS[i], errno);
			success = false;
		} else {
			PX4_DEBUG("Opened encoder %d: %s (fd=%d)", i, ENCODER_DEVICE_PATHS[i], _fd_encoders[i]);
		}
	}

	return success;
}

void QuadEncoder::close_encoders()
{
	for (int i = 0; i < MAX_ENCODERS; i++) {
		if (_fd_encoders[i] >= 0) {
			close(_fd_encoders[i]);
			_fd_encoders[i] = -1;
		}
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
	read_encoders();

	perf_end(_loop_perf);
}

void QuadEncoder::read_encoders()
{
	perf_begin(_read_perf);

	sensor_quad_encoder_s sensor_msg{};
	sensor_msg.timestamp = hrt_absolute_time();
	sensor_msg.count = _num_active_encoders;

	bool any_valid = false;

	for (int i = 0; i < _num_active_encoders; i++) {
		if (_fd_encoders[i] < 0) {
			_encoder_data[i].valid = false;
			sensor_msg.valid[i] = 0;
			continue;
		}

		// Read position from encoder using IOCTL
		int32_t position = 0;
		int ret = ioctl(_fd_encoders[i], QEIOC_POSITION, &position);

		if (ret == 0) {
			// Get encoder-specific parameters
			int32_t ppr = 1024; // default
			bool invert = false;

			switch (i) {
			case 0:
				ppr = _param_ppr_0.get();
				invert = _param_invert_0.get();
				break;
			case 1:
				ppr = _param_ppr_1.get();
				invert = _param_invert_1.get();
				break;
			case 2:
				ppr = _param_ppr_2.get();
				invert = _param_invert_2.get();
				break;
			case 3:
				ppr = _param_ppr_3.get();
				invert = _param_invert_3.get();
				break;
			default:
				ppr = 1024;
				invert = false;
				break;
			}

			if (invert) {
				position = -position;
			}

			_encoder_data[i].position = position;
			_encoder_data[i].timestamp = sensor_msg.timestamp;
			_encoder_data[i].valid = true;
			_encoder_data[i].pulses_per_rev = ppr;
			_encoder_data[i].invert_direction = invert;
			any_valid = true;

			// Calculate velocity and angle for rotary encoders
			float velocity = 0.0f;
			float angle_rad = 0.0f;

			if (_prev_timestamp > 0) {
				float dt = (sensor_msg.timestamp - _prev_timestamp) * 1e-6f;
				if (dt > 0.0f) {
					int32_t delta_pos = position - _prev_position[i];

					// Rotary encoder calculations
					float angular_velocity = (2.0f * M_PI * delta_pos) / (ppr * dt); // rad/s
					velocity = angular_velocity;
					angle_rad = (2.0f * M_PI * position) / ppr; // cumulative angle in radians

					_encoder_data[i].velocity_rad_s = velocity;
					_encoder_data[i].angle_rad = angle_rad;
				}
			}

			// Fill message
			sensor_msg.position[i] = position;
			sensor_msg.velocity[i] = _encoder_data[i].velocity_rad_s;
			sensor_msg.angle_or_distance[i] = _encoder_data[i].angle_rad;
			sensor_msg.valid[i] = 1;
			sensor_msg.pulses_per_rev[i] = ppr;
			sensor_msg.invert_direction[i] = invert;

			_prev_position[i] = position;

		} else {
			_encoder_data[i].valid = false;
			sensor_msg.valid[i] = 0;
			_error_count++;
			PX4_DEBUG("Failed to read encoder %d: %d", i, ret);
		}
	}

	// Fill remaining slots with invalid data
	for (int i = _num_active_encoders; i < sensor_quad_encoder_s::MAX_ENCODERS; i++) {
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

void QuadEncoder::reset_encoders()
{
	for (int i = 0; i < _num_active_encoders; i++) {
		if (_fd_encoders[i] >= 0) {
			// Reset encoder position
			if (ioctl(_fd_encoders[i], QEIOC_RESET, 0) < 0) {
				PX4_WARN("Failed to reset encoder %d", i);
			}
		}
		_prev_position[i] = 0;
	}

	_prev_timestamp = 0;

	PX4_INFO("Encoders reset");
}

void QuadEncoder::parameters_update()
{
	updateParams();

	// Update work schedule based on update rate parameter
	uint32_t update_interval_us = 1000000 / _param_update_rate.get();
	ScheduleOnInterval(update_interval_us);
}

void QuadEncoder::print_status()
{
	PX4_INFO("QuadEncoder (instance %d) status:", _instance_id);
	PX4_INFO("  Running: %s", _is_running ? "yes" : "no");
	PX4_INFO("  Update rate: %d Hz", _param_update_rate.get());
	PX4_INFO("  Active encoders: %d", _num_active_encoders);

	// Show uORB topic instances
	PX4_INFO("  Sensor encoder topic instance: %d", _sensor_encoder_instance);

	PX4_INFO("Encoder status:");
	for (int i = 0; i < _num_active_encoders; i++) {
		PX4_INFO("  Enc %d: %s, pos=%ld, vel=%.2f rad/s, angle=%.2f rad, ppr=%ld, invert=%s",
			i,
			_encoder_data[i].valid ? "OK" : "FAIL",
			(long)_encoder_data[i].position,
			(double)_encoder_data[i].velocity_rad_s,
			(double)_encoder_data[i].angle_rad,
			(long)_encoder_data[i].pulses_per_rev,
			_encoder_data[i].invert_direction ? "yes" : "no");
	}

	PX4_INFO("Statistics:");
	PX4_INFO("  Read count: %u", _read_count);
	PX4_INFO("  Error count: %u", _error_count);

	if (_last_error_time > 0) {
		PX4_INFO("  Last error: %llu us ago", hrt_absolute_time() - _last_error_time);
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_read_perf);
}
}

int QuadEncoder::task_spawn(int argc, char *argv[])
{
	int instance_id = 0;

	// Parse command line arguments
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			instance_id = atoi(myoptarg);
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
	QuadEncoder *instance = instantiate(instance_id);

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

QuadEncoder *QuadEncoder::instantiate(int instance)
{
	QuadEncoder *obj = new QuadEncoder(instance);
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
				_objects[instance_id]->reset_encoders();
				return 0;
			}

			PX4_WARN("Instance %d not running", instance_id);
			return 1;
		} else {
			// Reset all instances
			bool any_reset = false;
			for (int i = 0; i < MAX_INSTANCES; i++) {
				if (_objects[i] != nullptr) {
					_objects[i]->reset_encoders();
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
- Supports up to 4 encoders per instance with auto-detection
- Multi-instance support (up to 4 instances) with independent uORB topics
- Independent parameter configuration per encoder
- Configurable pulses per revolution and direction inversion

### Configuration
Configure each encoder using the following parameters:
- QE_PPR_X: Pulses per revolution for encoder X
- QE_INVERT_X: Invert direction for encoder X

### Examples
Start the driver (instance 0):
$ quad_encoder start

Start with specific instance:
$ quad_encoder start -i 1

Reset encoder positions:
$ quad_encoder reset

Reset specific instance:
$ quad_encoder reset -i 1

Check status of all instances:
$ quad_encoder status

Stop specific instance:
$ quad_encoder stop -i 1
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("quad_encoder", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "Instance ID", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset encoder positions");
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
- Supports up to 4 encoders per instance with auto-detection
- Multi-instance support (up to 4 instances) with independent uORB topics
- Independent parameter configuration per encoder
- Configurable pulses per revolution and direction inversion

### Configuration
Configure each encoder using the following parameters:
- QE_PPR_X: Pulses per revolution for encoder X
- QE_INVERT_X: Invert direction for encoder X

### Examples
Start the driver:
$ quad_encoder start

Start with specific instance:
$ quad_encoder start -i 1

Reset encoder positions:
$ quad_encoder reset

Check status:
$ quad_encoder status

Stop the driver:
$ quad_encoder stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("quad_encoder", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "Instance ID", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset encoder positions");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int quad_encoder_main(int argc, char *argv[])
{
	return QuadEncoder::main(argc, argv);
}
