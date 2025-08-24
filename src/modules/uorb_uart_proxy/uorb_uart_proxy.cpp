/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "uorb_uart_proxy.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <drivers/drv_hrt.h>

UorbUartProxy::UorbUartProxy() :
	ModuleBase(MODULE_NAME),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_last_heartbeat_time(0),
	_last_statistics_time(0),
	_last_main_board_heartbeat(0),
	_tx_sequence(0),
	_board_id(0)
{
	// Initialize statistics
	memset(&_stats, 0, sizeof(_stats));
}

UorbUartProxy::~UorbUartProxy()
{
	// Stop work queue
	ScheduledWorkItem::deinit();
}

bool UorbUartProxy::init()
{
	// Update parameters
	updateParams();

	// Determine board ID from parameter
	_board_id = getBoardId();

	// Get UART parameters
	const char *device_path = _param_uart_dev.get();
	int32_t baudrate = _param_uart_baud.get();

	// Check if proxy is enabled
	int32_t enable;
	param_get(param_find("UART_PROXY_EN"), &enable);
	if (!enable) {
		PX4_INFO("UART proxy disabled by parameter");
		return false;
	}

	// Initialize UART transport
	if (_uart_transport.init(device_path, (speed_t)baudrate) < 0) {
		PX4_ERR("Failed to initialize UART transport on %s at %d baud", device_path, baudrate);
		return false;
	}

	PX4_INFO("UART proxy initialized on %s at %d baud (Board ID: %d)", device_path, baudrate, _board_id);

	// Start work queue
	ScheduledWorkItem::ScheduleNow();

	return true;
}

void UorbUartProxy::Run()
{
	if (!_uart_transport.isReady()) {
		return;
	}

	hrt_abstime now = hrt_absolute_time();

	// Process incoming messages (X7+ -> this NXT board)
	processIncomingMessages();

	// Process outgoing messages (this NXT board -> X7+)
	processOutgoingMessages();

	// Send periodic heartbeat
	if (now - _last_heartbeat_time > HEARTBEAT_INTERVAL_US) {
		sendHeartbeat();
		_last_heartbeat_time = now;
	}

	// Print statistics periodically
	if (now - _last_statistics_time > STATISTICS_INTERVAL_US) {
		printStatistics();
		_last_statistics_time = now;
	}

	// Schedule next run
	ScheduleDelayed(MAIN_LOOP_INTERVAL_US);
}

void UorbUartProxy::processIncomingMessages()
{
	distributed_uorb::UartFrame frame;
	while (_uart_transport.receiveFrame(frame) > 0) {
		_stats.rx_messages++;
		_stats.rx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);

		// Update main board heartbeat tracking
		if (frame.header.board_id == distributed_uorb::BOARD_ID_X7_PLUS) {
			_last_main_board_heartbeat = hrt_absolute_time();
		}

		// Process based on message type
		switch (static_cast<distributed_uorb::UartMessageId>(frame.header.msg_id)) {
		case distributed_uorb::UartMessageId::WHEEL_LOADER_SETPOINT: {
			if (frame.header.length == sizeof(wheel_loader_setpoint_s)) {
				wheel_loader_setpoint_s setpoint;
				memcpy(&setpoint, frame.payload, sizeof(setpoint));
				_wheel_loader_setpoint_pub.publish(setpoint);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid wheel loader setpoint length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::ACTUATOR_OUTPUTS_FRONT: {
			// Only process if this is the front board
			if (_board_id == distributed_uorb::BOARD_ID_NXT_FRONT && frame.header.length == sizeof(actuator_outputs_s)) {
				actuator_outputs_s outputs;
				memcpy(&outputs, frame.payload, sizeof(outputs));
				_actuator_outputs_pub.publish(outputs);
			} else if (_board_id != distributed_uorb::BOARD_ID_NXT_FRONT) {
				// Ignore messages not intended for this board
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid front actuator outputs length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::ACTUATOR_OUTPUTS_REAR: {
			// Only process if this is the rear board
			if (_board_id == distributed_uorb::BOARD_ID_NXT_REAR && frame.header.length == sizeof(actuator_outputs_s)) {
				actuator_outputs_s outputs;
				memcpy(&outputs, frame.payload, sizeof(outputs));
				_actuator_outputs_pub.publish(outputs);
			} else if (_board_id != distributed_uorb::BOARD_ID_NXT_REAR) {
				// Ignore messages not intended for this board
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid rear actuator outputs length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::VEHICLE_STATUS: {
			if (frame.header.length == sizeof(vehicle_status_s)) {
				vehicle_status_s status;
				memcpy(&status, frame.payload, sizeof(status));
				_vehicle_status_pub.publish(status);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid vehicle status length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::TRACTION_CONTROL: {
			if (frame.header.length == sizeof(traction_control_s)) {
				traction_control_s traction_control;
				memcpy(&traction_control, frame.payload, sizeof(traction_control));
				_traction_control_pub.publish(traction_control);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid traction control length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::BOOM_COMMAND: {
			// Only process if this is the rear board (boom control)
			if (_board_id == distributed_uorb::BOARD_ID_NXT_REAR && frame.header.length == sizeof(boom_command_s)) {
				boom_command_s boom_command;
				memcpy(&boom_command, frame.payload, sizeof(boom_command));
				_boom_command_pub.publish(boom_command);
			} else if (_board_id != distributed_uorb::BOARD_ID_NXT_REAR) {
				// Ignore messages not intended for this board
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid boom command length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::BUCKET_COMMAND: {
			// Only process if this is the front board (bucket control)
			if (_board_id == distributed_uorb::BOARD_ID_NXT_FRONT && frame.header.length == sizeof(bucket_command_s)) {
				bucket_command_s bucket_command;
				memcpy(&bucket_command, frame.payload, sizeof(bucket_command));
				_bucket_command_pub.publish(bucket_command);
			} else if (_board_id != distributed_uorb::BOARD_ID_NXT_FRONT) {
				// Ignore messages not intended for this board
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid bucket command length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::STEERING_COMMAND: {
			// Only process if this is the rear board (steering control)
			if (_board_id == distributed_uorb::BOARD_ID_NXT_REAR && frame.header.length == sizeof(steering_command_s)) {
				steering_command_s steering_command;
				memcpy(&steering_command, frame.payload, sizeof(steering_command));
				_steering_command_pub.publish(steering_command);
			} else if (_board_id != distributed_uorb::BOARD_ID_NXT_REAR) {
				// Ignore messages not intended for this board
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid steering command length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::HEARTBEAT: {
			// Heartbeat received from main board - already tracked above
			break;
		}

		case distributed_uorb::UartMessageId::LOAD_LAMP_COMMAND: {
			// Only process if this is the rear board (load lamp control)
			if (_board_id == distributed_uorb::BOARD_ID_NXT_REAR && frame.header.length == sizeof(load_lamp_command_s)) {
				load_lamp_command_s load_lamp_command;
				memcpy(&load_lamp_command, frame.payload, sizeof(load_lamp_command));
				_load_lamp_command_pub.publish(load_lamp_command);
			} else if (_board_id != distributed_uorb::BOARD_ID_NXT_REAR) {
				// Ignore messages not intended for this board
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid load lamp command length: %d", frame.header.length);
			}
			break;
		}

		default:
			_stats.rx_errors++;
			PX4_WARN("Unknown message ID: %d", frame.header.msg_id);
			break;
		}
	}
}

void UorbUartProxy::processOutgoingMessages()
{
	// Send limit sensor data (bucket limit - front board only)
	if (_board_id == distributed_uorb::BOARD_ID_NXT_FRONT) {
		limit_sensor_s limit_data;
		if (_limit_sensor_sub.update(&limit_data)) {
			distributed_uorb::UartFrame frame;
			frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::LIMIT_SENSOR_BUCKET);
			frame.header.board_id = _board_id;
			frame.header.length = sizeof(limit_data);
			frame.header.sequence = _tx_sequence++;
			frame.header.timestamp = hrt_absolute_time();

			memcpy(frame.payload, &limit_data, sizeof(limit_data));

			if (_uart_transport.sendFrame(frame) >= 0) {
				_stats.tx_messages++;
				_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
			} else {
				_stats.tx_errors++;
				PX4_WARN("Failed to send limit sensor data");
			}
		}
	}

	// Send slip estimation data
	slip_estimation_s slip_data;
	if (_slip_estimation_sub.update(&slip_data)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;

		// Use appropriate message ID based on board type
		if (_board_id == distributed_uorb::BOARD_ID_NXT_FRONT) {
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::SLIP_ESTIMATION_FRONT);
		} else {
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::SLIP_ESTIMATION_REAR);
		}

		frame.header.board_id = _board_id;
		frame.header.length = sizeof(slip_data);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &slip_data, sizeof(slip_data));

		if (_uart_transport.sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send slip estimation data");
		}
	}

	// Send boom status (rear board only)
	if (_board_id == distributed_uorb::BOARD_ID_NXT_REAR) {
		boom_status_s boom_status;
		if (_boom_status_sub.update(&boom_status)) {
			distributed_uorb::UartFrame frame;
			frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::BOOM_STATUS);
			frame.header.board_id = _board_id;
			frame.header.length = sizeof(boom_status);
			frame.header.sequence = _tx_sequence++;
			frame.header.timestamp = hrt_absolute_time();

			memcpy(frame.payload, &boom_status, sizeof(boom_status));

			if (_uart_transport.sendFrame(frame) >= 0) {
				_stats.tx_messages++;
				_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
			} else {
				_stats.tx_errors++;
				PX4_WARN("Failed to send boom status");
			}
		}
	}

	// Send bucket status (front board only)
	if (_board_id == distributed_uorb::BOARD_ID_NXT_FRONT) {
		bucket_status_s bucket_status;
		if (_bucket_status_sub.update(&bucket_status)) {
			distributed_uorb::UartFrame frame;
			frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::BUCKET_STATUS);
			frame.header.board_id = _board_id;
			frame.header.length = sizeof(bucket_status);
			frame.header.sequence = _tx_sequence++;
			frame.header.timestamp = hrt_absolute_time();

			memcpy(frame.payload, &bucket_status, sizeof(bucket_status));

			if (_uart_transport.sendFrame(frame) >= 0) {
				_stats.tx_messages++;
				_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
			} else {
				_stats.tx_errors++;
				PX4_WARN("Failed to send bucket status");
			}
		}
	}

	// Send steering status (rear board only)
	if (_board_id == distributed_uorb::BOARD_ID_NXT_REAR) {
		steering_status_s steering_status;
		if (_steering_status_sub.update(&steering_status)) {
			distributed_uorb::UartFrame frame;
			frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::STEERING_STATUS);
			frame.header.board_id = _board_id;
			frame.header.length = sizeof(steering_status);
			frame.header.sequence = _tx_sequence++;
			frame.header.timestamp = hrt_absolute_time();

			memcpy(frame.payload, &steering_status, sizeof(steering_status));

			if (_uart_transport.sendFrame(frame) >= 0) {
				_stats.tx_messages++;
				_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
			} else {
				_stats.tx_errors++;
				PX4_WARN("Failed to send steering status");
			}
		}
	}

	// Send HBridge status (multi-instance, NXT → X7+)
	hbridge_status_s hbridge_status;

	// Send HBridge status instance 0
	if (_hbridge_status_sub_0.update(&hbridge_status)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;

		// Use appropriate message ID based on board type and instance
		if (_board_id == distributed_uorb::BOARD_ID_NXT_FRONT) {
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::HBRIDGE_STATUS_FRONT_0);
		} else {
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::HBRIDGE_STATUS_REAR_0);
		}

		frame.header.board_id = _board_id;
		frame.header.length = sizeof(hbridge_status);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &hbridge_status, sizeof(hbridge_status));

		if (_uart_transport.sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send HBridge status instance 0");
		}
	}

	// Send HBridge status instance 1
	if (_hbridge_status_sub_1.update(&hbridge_status)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;

		// Use appropriate message ID based on board type and instance
		if (_board_id == distributed_uorb::BOARD_ID_NXT_FRONT) {
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::HBRIDGE_STATUS_FRONT_1);
		} else {
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::HBRIDGE_STATUS_REAR_1);
		}

		frame.header.board_id = _board_id;
		frame.header.length = sizeof(hbridge_status);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &hbridge_status, sizeof(hbridge_status));

		if (_uart_transport.sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send HBridge status instance 1");
		}
	}

	// Send sensor quad encoder data (multi-instance, NXT → X7+)
	sensor_quad_encoder_s sensor_data;

	// Send sensor quad encoder instance 0
	if (_sensor_quad_encoder_sub_0.update(&sensor_data)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;

		// Use appropriate message ID based on board type
		if (_board_id == distributed_uorb::BOARD_ID_NXT_FRONT) {
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::SENSOR_QUAD_ENCODER_FRONT);
		} else {
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::SENSOR_QUAD_ENCODER_REAR);
		}

		frame.header.board_id = _board_id;
		frame.header.length = sizeof(sensor_data);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &sensor_data, sizeof(sensor_data));

		if (_uart_transport.sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send sensor quad encoder instance 0");
		}
	}

	// Send sensor quad encoder instance 1
	if (_sensor_quad_encoder_sub_1.update(&sensor_data)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;

		// Use appropriate message ID based on board type
		if (_board_id == distributed_uorb::BOARD_ID_NXT_FRONT) {
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::SENSOR_QUAD_ENCODER_FRONT);
		} else {
			frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::SENSOR_QUAD_ENCODER_REAR);
		}

		frame.header.board_id = _board_id;
		frame.header.length = sizeof(sensor_data);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &sensor_data, sizeof(sensor_data));

		if (_uart_transport.sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send sensor quad encoder instance 1");
		}
	}
}

void UorbUartProxy::sendHeartbeat()
{
	distributed_uorb::UartFrame frame;
	frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
	frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::HEARTBEAT);
	frame.header.board_id = _board_id;
	frame.header.length = 0;
	frame.header.sequence = _tx_sequence++;
	frame.header.timestamp = hrt_absolute_time();

	if (_uart_transport.sendFrame(frame) >= 0) {
		_stats.tx_messages++;
		_stats.tx_bytes += sizeof(frame.header) + sizeof(frame.crc);
	} else {
		_stats.tx_errors++;
	}
}

void UorbUartProxy::printStatistics()
{
	hrt_abstime now = hrt_absolute_time();
	bool main_board_online = (now - _last_main_board_heartbeat) < HEARTBEAT_TIMEOUT_US;

	const char *board_name = (_board_id == distributed_uorb::BOARD_ID_NXT_FRONT) ? "FRONT" : "REAR";

	PX4_INFO("UART Proxy [%s] Stats - TX: %lu msgs, %lu bytes, %lu errors | RX: %lu msgs, %lu bytes, %lu errors",
		 board_name, _stats.tx_messages, _stats.tx_bytes, _stats.tx_errors,
		 _stats.rx_messages, _stats.rx_bytes, _stats.rx_errors);

	PX4_INFO("Main Board Status: %s", main_board_online ? "ONLINE" : "OFFLINE");
}

uint8_t UorbUartProxy::getBoardId()
{
	int32_t board_type = _param_board_type.get();

	switch (board_type) {
	case 0:
		return distributed_uorb::BOARD_ID_NXT_FRONT;
	case 1:
		return distributed_uorb::BOARD_ID_NXT_REAR;
	default:
		PX4_WARN("Invalid board type %d, defaulting to front", board_type);
		return distributed_uorb::BOARD_ID_NXT_FRONT;
	}
}

int UorbUartProxy::task_spawn(int argc, char *argv[])
{
	UorbUartProxy *instance = new UorbUartProxy();

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

int UorbUartProxy::print_status()
{
	PX4_INFO("UART Proxy Module Status");

	if (_uart_transport) {
		PX4_INFO("UART transport initialized");
		PX4_INFO("Board ID: %d", _board_id);
	} else {
		PX4_INFO("UART transport not initialized");
		return PX4_OK;
	}

	// Print current statistics
	const_cast<UorbUartProxy*>(this)->printStatistics();

	return PX4_OK;
}

int UorbUartProxy::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	if (!strcmp(argv[0], "status")) {
		return get_instance()->print_status();
	}

	return print_usage("unknown command");
}

int UorbUartProxy::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
uORB UART Proxy module for distributed uORB messaging over UART.

This module runs on NXT-Dual controller boards and provides transparent
uORB messaging to/from the X7+ main board via UART. It receives commands
from the main board and sends back status information.

### Examples
Start the proxy:
$ uorb_uart_proxy start

Check status:
$ uorb_uart_proxy status

Stop the proxy:
$ uorb_uart_proxy stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uorb_uart_proxy", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int uorb_uart_proxy_main(int argc, char *argv[])
{
	return UorbUartProxy::main(argc, argv);
}
