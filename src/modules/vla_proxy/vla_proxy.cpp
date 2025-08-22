#include "vla_proxy.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

VLAProxy::VLAProxy() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

VLAProxy::~VLAProxy()
{
	close_vla_connection();
}

bool VLAProxy::init()
{
	if (!_param_vla_proxy_enabled.get()) {
		PX4_INFO("VLA Proxy disabled");
		return false;
	}

	if (!init_vla_connection()) {
		PX4_ERR("Failed to initialize VLA connection");
		return false;
	}

	ScheduleOnInterval(SCHEDULE_INTERVAL);
	return true;
}

void VLAProxy::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Process vehicle commands (start/stop VLA)
	vehicle_command_s cmd;

	if (_vehicle_command_sub.update(&cmd)) {
		// Custom command for VLA control (using MAV_CMD_USER_1 for VLA start/stop)
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_USER_1) {
			if (cmd.param1 > 0.5f) {
				handle_vla_command(VLACommand::START);
			} else {
				handle_vla_command(VLACommand::STOP);
			}

			// Send acknowledgment
			vehicle_command_ack_s ack{};
			ack.command = cmd.command;
			ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			ack.timestamp = hrt_absolute_time();
			_vehicle_command_ack_pub.publish(ack);
		}
	}

	if (_vla_running && _vla_connected) {
		// Send robot status to VLA
		process_robot_status();

		// Receive and process VLA trajectory
		process_vla_trajectory();
	}
}

void VLAProxy::process_robot_status()
{
	const hrt_abstime now = hrt_absolute_time();

	// Rate limit status updates
	if (now - _last_status_sent < (1000_ms / _param_vla_update_rate.get())) {
		return;
	}

	RobotStatus status{};
	status.timestamp = now;

	// Get vehicle status
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.copy(&vehicle_status)) {
		status.armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		status.flight_mode = vehicle_status.nav_state;
		// Note: battery_remaining is often in battery_status topic, using 0.0 as placeholder
		status.battery_percentage = 0.0f;
	}

	// Get position and velocity
	vehicle_local_position_s local_pos;

	if (_vehicle_local_position_sub.copy(&local_pos)) {
		status.position[0] = local_pos.x;
		status.position[1] = local_pos.y;
		status.position[2] = local_pos.z;
		status.velocity[0] = local_pos.vx;
		status.velocity[1] = local_pos.vy;
		status.velocity[2] = local_pos.vz;
	}

	// Get attitude
	vehicle_attitude_s attitude;

	if (_vehicle_attitude_sub.copy(&attitude)) {
		status.quaternion[0] = attitude.q[0];
		status.quaternion[1] = attitude.q[1];
		status.quaternion[2] = attitude.q[2];
		status.quaternion[3] = attitude.q[3];

		// Angular velocities from attitude topic
		status.angular_vel[0] = attitude.rollspeed;
		status.angular_vel[1] = attitude.pitchspeed;
		status.angular_vel[2] = attitude.yawspeed;
	}

	send_status_to_vla(status);
	_last_status_sent = now;
}

void VLAProxy::process_vla_trajectory()
{
	std::deque<VLAWaypoint> new_trajectory;

	if (receive_trajectory_from_vla(new_trajectory)) {
		std::lock_guard<std::mutex> lock(_trajectory_mutex);
		_trajectory_buffer = std::move(new_trajectory);
		_last_trajectory_received = hrt_absolute_time();
	}

	// Check for trajectory timeout
	if (hrt_absolute_time() - _last_trajectory_received > _param_vla_timeout_ms.get() * 1000) {
		PX4_WARN("VLA trajectory timeout");
		_vla_running = false;
		return;
	}

	// Publish next waypoint
	std::lock_guard<std::mutex> lock(_trajectory_mutex);

	if (!_trajectory_buffer.empty()) {
		const VLAWaypoint &wp = _trajectory_buffer.front();

		// Check if waypoint timestamp is current
		if (wp.timestamp_us <= hrt_absolute_time()) {
			publish_trajectory_setpoint(wp);
			_trajectory_buffer.pop_front();
		}
	}
}

void VLAProxy::publish_trajectory_setpoint(const VLAWaypoint &waypoint)
{
	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();

	setpoint.position[0] = waypoint.position[0];
	setpoint.position[1] = waypoint.position[1];
	setpoint.position[2] = waypoint.position[2];

	setpoint.velocity[0] = waypoint.velocity[0];
	setpoint.velocity[1] = waypoint.velocity[1];
	setpoint.velocity[2] = waypoint.velocity[2];

	setpoint.acceleration[0] = waypoint.acceleration[0];
	setpoint.acceleration[1] = waypoint.acceleration[1];
	setpoint.acceleration[2] = waypoint.acceleration[2];

	setpoint.yaw = waypoint.yaw;
	setpoint.yawspeed = waypoint.yaw_rate;

	_trajectory_setpoint_pub.publish(setpoint);
}

void VLAProxy::handle_vla_command(VLACommand cmd)
{
	switch (cmd) {
	case VLACommand::START:
		PX4_INFO("Starting VLA");
		_vla_running = true;
		_last_trajectory_received = hrt_absolute_time();
		break;

	case VLACommand::STOP:
		PX4_INFO("Stopping VLA");
		_vla_running = false;
		_trajectory_buffer.clear();
		break;

	case VLACommand::PAUSE:
		PX4_INFO("Pausing VLA");
		_vla_running = false;
		break;

	case VLACommand::RESUME:
		PX4_INFO("Resuming VLA");
		_vla_running = true;
		break;
	}
}

bool VLAProxy::init_vla_connection()
{
	// Determine UART device based on parameter
	const char *uart_device;
	int dev_num = _param_vla_uart_dev.get();

	switch (dev_num) {
	case 0:
		uart_device = "/dev/ttyS3";
		break;
	case 1:
		uart_device = "/dev/ttyS4";
		break;
	case 2:
		uart_device = "/dev/ttyS5";
		break;
	case 3:
		uart_device = "/dev/ttyUSB0";
		break;
	default:
		snprintf(_uart_device, sizeof(_uart_device), "/dev/ttyS%d", dev_num + 3);
		uart_device = _uart_device;
		break;
	}

	// Open UART device
	_vla_uart_fd = open(uart_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_vla_uart_fd < 0) {
		PX4_ERR("Failed to open UART device %s: %s", uart_device, strerror(errno));
		return false;
	}

	// Configure UART settings
	struct termios uart_config;
	if (tcgetattr(_vla_uart_fd, &uart_config) < 0) {
		PX4_ERR("Failed to get UART attributes: %s", strerror(errno));
		close(_vla_uart_fd);
		_vla_uart_fd = -1;
		return false;
	}

	// Set baud rate
	int baud_rate = _param_vla_uart_baud.get();
	speed_t speed;

	switch (baud_rate) {
	case 9600:
		speed = B9600;
		break;
	case 19200:
		speed = B19200;
		break;
	case 38400:
		speed = B38400;
		break;
	case 57600:
		speed = B57600;
		break;
	case 115200:
		speed = B115200;
		break;
	case 230400:
		speed = B230400;
		break;
	case 460800:
		speed = B460800;
		break;
	case 921600:
		speed = B921600;
		break;
	default:
		PX4_WARN("Unsupported baud rate %d, using 115200", baud_rate);
		speed = B115200;
		break;
	}

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("Failed to set UART baud rate: %s", strerror(errno));
		close(_vla_uart_fd);
		_vla_uart_fd = -1;
		return false;
	}

	// Configure UART parameters: 8N1, no flow control
	uart_config.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
	uart_config.c_cflag |= CS8 | CREAD | CLOCAL;

	// Disable input processing
	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

	// Raw output
	uart_config.c_oflag &= ~OPOST;

	// Disable line processing
	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// Set read timeout (non-blocking with timeout)
	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 1; // 0.1 second timeout

	if (tcsetattr(_vla_uart_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("Failed to set UART attributes: %s", strerror(errno));
		close(_vla_uart_fd);
		_vla_uart_fd = -1;
		return false;
	}

	// Flush buffers
	tcflush(_vla_uart_fd, TCIOFLUSH);

	_vla_connected = true;
	PX4_INFO("Connected to VLA via UART %s at %d baud", uart_device, baud_rate);
	return true;
}

void VLAProxy::close_vla_connection()
{
	if (_vla_uart_fd >= 0) {
		close(_vla_uart_fd);
		_vla_uart_fd = -1;
	}

	_vla_connected = false;
}

void VLAProxy::send_status_to_vla(const RobotStatus &status)
{
	if (_vla_uart_fd < 0) { return; }

	// Create a simple frame format: [HEADER][SIZE][DATA][CHECKSUM]
	const uint8_t FRAME_HEADER = 0xAA;
	const uint8_t FRAME_TYPE_STATUS = 0x01;

	uint8_t buffer[128];
	size_t offset = 0;

	// Frame header
	buffer[offset++] = FRAME_HEADER;
	buffer[offset++] = FRAME_TYPE_STATUS;
	buffer[offset++] = sizeof(RobotStatus); // Data length

	// Copy status data
	memcpy(&buffer[offset], &status, sizeof(RobotStatus));
	offset += sizeof(RobotStatus);

	// Simple checksum (XOR of all data bytes)
	uint8_t checksum = 0;
	for (size_t i = 3; i < offset; i++) {
		checksum ^= buffer[i];
	}
	buffer[offset++] = checksum;

	// Send data via UART
	ssize_t sent = write(_vla_uart_fd, buffer, offset);

	if (sent < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_ERR("Failed to send status to VLA via UART: %s", strerror(errno));
			close_vla_connection();
		}
	} else if (sent != (ssize_t)offset) {
		PX4_WARN("Partial UART write: sent %zd of %zu bytes", sent, offset);
	}
}

bool VLAProxy::receive_trajectory_from_vla(std::deque<VLAWaypoint> &trajectory)
{
	if (_vla_uart_fd < 0) { return false; }

	static uint8_t rx_buffer[256];
	static size_t rx_index = 0;

	// Read available data
	uint8_t temp_buffer[64];
	ssize_t bytes_read = read(_vla_uart_fd, temp_buffer, sizeof(temp_buffer));

	if (bytes_read > 0) {
		// Add new data to receive buffer
		for (ssize_t i = 0; i < bytes_read && rx_index < sizeof(rx_buffer); i++) {
			rx_buffer[rx_index++] = temp_buffer[i];
		}

		// Process complete frames
		while (rx_index >= 4) { // Minimum frame size
			// Look for frame header
			if (rx_buffer[0] != 0xAA) {
				// Shift buffer to find next header
				memmove(rx_buffer, &rx_buffer[1], rx_index - 1);
				rx_index--;
				continue;
			}

			uint8_t frame_type = rx_buffer[1];
			uint8_t data_length = rx_buffer[2];

			// Check if we have complete frame
			size_t frame_length = 3 + data_length + 1; // header + type + length + data + checksum
			if (rx_index < frame_length) {
				break; // Wait for more data
			}

			// Verify checksum
			uint8_t calculated_checksum = 0;
			for (size_t i = 3; i < 3 + data_length; i++) {
				calculated_checksum ^= rx_buffer[i];
			}

			uint8_t received_checksum = rx_buffer[3 + data_length];

			if (calculated_checksum == received_checksum) {
				// Process valid frame
				if (frame_type == 0x02 && data_length == sizeof(VLAWaypoint)) {
					// Trajectory waypoint frame
					VLAWaypoint waypoint;
					memcpy(&waypoint, &rx_buffer[3], sizeof(VLAWaypoint));
					trajectory.push_back(waypoint);

					// Remove processed frame from buffer
					memmove(rx_buffer, &rx_buffer[frame_length], rx_index - frame_length);
					rx_index -= frame_length;
					return true;
				}
			} else {
				PX4_WARN("UART frame checksum mismatch");
			}

			// Remove invalid/processed frame
			memmove(rx_buffer, &rx_buffer[frame_length], rx_index - frame_length);
			rx_index -= frame_length;
		}

	} else if (bytes_read < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_ERR("Failed to receive trajectory from VLA via UART: %s", strerror(errno));
			close_vla_connection();
		}
	}

	return false;
}

int VLAProxy::task_spawn(int argc, char *argv[])
{
	VLAProxy *instance = new VLAProxy();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int VLAProxy::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VLAProxy::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
VLA (Vision-Language-Action) model proxy module.

This module acts as a bridge between PX4 and a VLA model, handling:
- Sending robot status to VLA via UART
- Receiving trajectory commands from VLA via UART
- Managing VLA start/stop commands

### Implementation
The module communicates with the VLA model via UART using a simple framed protocol.
Default UART device: /dev/ttyS3 at 115200 baud.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vla_proxy", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}

int vla_proxy_main(int argc, char *argv[])
{
	return VLAProxy::main(argc, argv);
}
