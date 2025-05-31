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
	close_encoders();

	perf_free(_loop_perf);
	perf_free(_read_perf);
	perf_free(_odom_perf);
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

	// Calculate and publish odometry if enabled
	if (_param_enable_odom.get()) {
		calculate_odometry();
	}

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
			uint8_t encoder_type = 0; // default to rotary
			float gear_ratio = 1.0f; // default

			switch (i) {
			case 0:
				ppr = _param_ppr_0.get();
				invert = _param_invert_0.get();
				encoder_type = _param_type_0.get();
				gear_ratio = _param_gear_ratio_0.get();
				break;
			case 1:
				ppr = _param_ppr_1.get();
				invert = _param_invert_1.get();
				encoder_type = _param_type_1.get();
				gear_ratio = _param_gear_ratio_1.get();
				break;
			case 2:
				ppr = _param_ppr_2.get();
				invert = _param_invert_2.get();
				encoder_type = _param_type_2.get();
				gear_ratio = _param_gear_ratio_2.get();
				break;
			case 3:
				ppr = _param_ppr_3.get();
				invert = _param_invert_3.get();
				encoder_type = _param_type_3.get();
				gear_ratio = _param_gear_ratio_3.get();
				break;
			default:
				ppr = 1024;
				invert = false;
				encoder_type = 0;
				gear_ratio = 1.0f;
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
			_encoder_data[i].encoder_type = encoder_type;
			_encoder_data[i].gear_ratio = gear_ratio;
			any_valid = true;

			// Calculate velocity and position based on encoder type
			float velocity = 0.0f;
			float angle_or_distance = 0.0f;

			if (_prev_timestamp > 0) {
				float dt = (sensor_msg.timestamp - _prev_timestamp) * 1e-6f;
				if (dt > 0.0f) {
					int32_t delta_pos = position - _prev_position[i];

					if (encoder_type == sensor_quad_encoder_s::TYPE_LINEAR) {
						// Linear encoder calculations
						// Convert pulses to mm, then to meters
						float distance_delta = (delta_pos * gear_ratio) / 1000.0f; // gear_ratio is screw pitch in mm
						velocity = distance_delta / dt; // m/s
						angle_or_distance = (position * gear_ratio) / 1000.0f; // cumulative distance in meters
					} else {
						// Rotary encoder calculations (default)
						float angular_velocity = (2.0f * M_PI * delta_pos * gear_ratio) / (ppr * dt); // rad/s
						velocity = angular_velocity;
						angle_or_distance = (2.0f * M_PI * position * gear_ratio) / ppr; // cumulative angle in radians
					}

					_encoder_data[i].velocity_rad_s = velocity;
					_encoder_data[i].angle_rad = angle_or_distance;
				}
			}

			// Fill message
			sensor_msg.position[i] = position;
			sensor_msg.velocity[i] = _encoder_data[i].velocity_rad_s;
			sensor_msg.angle_or_distance[i] = _encoder_data[i].angle_rad;
			sensor_msg.valid[i] = 1;
			sensor_msg.encoder_type[i] = encoder_type;
			sensor_msg.pulses_per_rev[i] = ppr;
			sensor_msg.invert_direction[i] = invert;
			sensor_msg.gear_ratio[i] = gear_ratio;

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
		sensor_msg.encoder_type[i] = sensor_quad_encoder_s::TYPE_ROTARY;
		sensor_msg.pulses_per_rev[i] = 0;
		sensor_msg.invert_direction[i] = false;
		sensor_msg.gear_ratio[i] = 1.0f;
	}

	if (any_valid) {
		_sensor_quad_encoder_pub.publish(sensor_msg);
		_read_count++;
	}

	_prev_timestamp = sensor_msg.timestamp;
	perf_end(_read_perf);
}

void QuadEncoder::calculate_odometry()
{
	perf_begin(_odom_perf);

	// Only calculate odometry for wheel applications with at least 2 encoders
	if (_num_active_encoders < 2) {
		perf_end(_odom_perf);
		return;
	}

	// Check if we have valid encoder data for at least the first 2 encoders (left/right)
	bool valid_data = _encoder_data[0].valid && _encoder_data[1].valid;
	if (!valid_data) {
		perf_end(_odom_perf);
		return;
	}

	// Calculate wheel velocities in m/s
	float wheel_radius = _param_wheel_radius.get();
	float wheel_base = _param_wheel_base.get();

	// Use first two encoders as left/right wheels
	float v_left = _encoder_data[0].velocity_rad_s * wheel_radius;   // Left wheel
	float v_right = _encoder_data[1].velocity_rad_s * wheel_radius;  // Right wheel

	// For 4-wheel systems, average front and rear wheels
	if (_num_active_encoders >= 4 && _encoder_data[2].valid && _encoder_data[3].valid) {
		float v_rl = _encoder_data[2].velocity_rad_s * wheel_radius; // Rear Left
		float v_rr = _encoder_data[3].velocity_rad_s * wheel_radius; // Rear Right
		v_left = (v_left + v_rl) / 2.0f;
		v_right = (v_right + v_rr) / 2.0f;
	}

	// Differential drive kinematics
	// Calculate linear and angular velocities
	_linear_velocity = (v_left + v_right) / 2.0f;
	_angular_velocity = (v_right - v_left) / wheel_base;

	// Integrate position and heading
	if (_prev_timestamp > 0) {
		float dt = (_encoder_data[0].timestamp - _prev_timestamp) * 1e-6f;

		if (dt > 0.001f && dt < 1.0f) { // Sanity check
			// Update heading
			_heading += _angular_velocity * dt;

			// Normalize heading to [-pi, pi]
			while (_heading > M_PI) _heading -= 2.0f * M_PI;
			while (_heading < -M_PI) _heading += 2.0f * M_PI;

			// Update position
			_position_x += _linear_velocity * cosf(_heading) * dt;
			_position_y += _linear_velocity * sinf(_heading) * dt;
		}
	}

	// Publish odometry
	vehicle_odometry_s odom{};
	odom.timestamp = hrt_absolute_time();
	odom.timestamp_sample = _encoder_data[0].timestamp;

	// Position (NED frame)
	odom.position[0] = _position_x;  // North
	odom.position[1] = _position_y;  // East
	odom.position[2] = 0.0f;         // Down (ground vehicle)

	// Quaternion from heading (yaw only)
	float half_yaw = _heading * 0.5f;
	odom.q[0] = cosf(half_yaw);      // w
	odom.q[1] = 0.0f;                // x
	odom.q[2] = 0.0f;                // y
	odom.q[3] = sinf(half_yaw);      // z

	// Velocity (body frame)
	odom.velocity[0] = _linear_velocity; // Forward
	odom.velocity[1] = 0.0f;             // Right (assuming no lateral motion)
	odom.velocity[2] = 0.0f;             // Down

	// Angular velocity (body frame)
	odom.angular_velocity[0] = 0.0f;             // Roll rate
	odom.angular_velocity[1] = 0.0f;             // Pitch rate
	odom.angular_velocity[2] = _angular_velocity; // Yaw rate

	// Set frame ID and pose frame
	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;

	// Quality indicators
	odom.quality = 1; // Good quality (could be made dynamic based on encoder health)

	_vehicle_odometry_pub.publish(odom);

	perf_end(_odom_perf);
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

	// Reset odometry
	_position_x = 0.0;
	_position_y = 0.0;
	_heading = 0.0;
	_linear_velocity = 0.0;
	_angular_velocity = 0.0;
	_prev_timestamp = 0;

	PX4_INFO("Encoders and odometry reset");
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
	PX4_INFO("  Odometry enabled: %s", _param_enable_odom.get() ? "yes" : "no");

	if (_param_enable_odom.get()) {
		PX4_INFO("  Wheel base: %.3f m", (double)_param_wheel_base.get());
		PX4_INFO("  Wheel radius: %.3f m", (double)_param_wheel_radius.get());
	}

	PX4_INFO("Encoder status:");
	for (int i = 0; i < _num_active_encoders; i++) {
		const char* type_str = (_encoder_data[i].encoder_type == sensor_quad_encoder_s::TYPE_LINEAR) ? "LINEAR" : "ROTARY";
		const char* unit_str = (_encoder_data[i].encoder_type == sensor_quad_encoder_s::TYPE_LINEAR) ? "m/s" : "rad/s";

		PX4_INFO("  Enc %d: %s, type=%s, pos=%ld, vel=%.2f %s, ppr=%ld, gear=%.3f",
			i,
			_encoder_data[i].valid ? "OK" : "FAIL",
			type_str,
			(long)_encoder_data[i].position,
			(double)_encoder_data[i].velocity_rad_s,
			unit_str,
			(long)_encoder_data[i].pulses_per_rev,
			(double)_encoder_data[i].gear_ratio);
	}

	if (_param_enable_odom.get()) {
		PX4_INFO("Odometry:");
		PX4_INFO("  Position: (%.3f, %.3f) m", _position_x, _position_y);
		PX4_INFO("  Heading: %.2f deg", (double)(_heading * 180.0f / M_PI));
		PX4_INFO("  Linear vel: %.3f m/s", _linear_velocity);
		PX4_INFO("  Angular vel: %.3f rad/s", _angular_velocity);
	}

	PX4_INFO("Statistics:");
	PX4_INFO("  Read count: %u", _read_count);
	PX4_INFO("  Error count: %u", _error_count);

	if (_last_error_time > 0) {
		PX4_INFO("  Last error: %llu us ago", hrt_absolute_time() - _last_error_time);
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_read_perf);
	perf_print_counter(_odom_perf);
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

	QuadEncoder *instance = new QuadEncoder(instance_id);

	if (instance == nullptr) {
		PX4_ERR("Allocation failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return -1;
	}

	return 0;
}

int QuadEncoder::custom_command(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	const char *command = argv[1];

	if (!strcmp(command, "reset")) {
		QuadEncoder *instance = get_instance();
		if (instance) {
			instance->reset_encoders();
			return 0;
		}
		PX4_ERR("Driver not running");
		return 1;
	}

	return print_usage("unknown command");
}

void QuadEncoder::usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Generic GPIO-based quadrature encoder driver supporting both rotary and linear encoders.

This driver interfaces with the NuttX GPIO quadrature encoder framework
to provide encoder data for various applications including:
- Wheels and rotary actuators (TYPE_ROTARY)
- Linear actuators and linear motors (TYPE_LINEAR)
- Other position sensors

Features:
- Supports up to 8 encoders with auto-detection
- Configurable encoder types (rotary/linear)
- Gear ratio and screw pitch support for proper scaling
- Optional odometry calculation for wheel applications
- Publishes sensor_quad_encoder uORB topic with unified data

### Configuration
Use parameters QE_TYPE_0-3 to set encoder types:
- 0 = Rotary encoder (wheels, motors) - outputs rad/s and radians
- 1 = Linear encoder (actuators) - outputs m/s and meters

For linear encoders, set QE_GEAR_RATIO_X to screw pitch in mm/rev.
For rotary encoders, set QE_GEAR_RATIO_X to gear reduction ratio.

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset encoder positions and odometry");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int quad_encoder_main(int argc, char *argv[])
{
	return QuadEncoder::main(argc, argv);
}
