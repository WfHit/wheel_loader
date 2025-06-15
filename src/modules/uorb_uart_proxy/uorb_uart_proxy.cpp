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
	_param_uart_dev(PARAM_UART_PROXY_DEV),
	_param_uart_baud(PARAM_UART_PROXY_BAUD),
	_param_board_type(PARAM_UART_PROXY_TYPE),
	_uart_transport(nullptr),
	_last_heartbeat_time(0),
	_last_statistics_time(0),
	_last_main_board_heartbeat(0),
	_tx_sequence(0),
	_board_id(BOARD_ID_NXT_FRONT)
{
	// Initialize statistics
	memset(&_stats, 0, sizeof(_stats));
}

UorbUartProxy::~UorbUartProxy()
{
	// Stop work queue
	ScheduledWorkItem::deinit();

	// Clean up UART transport
	if (_uart_transport) {
		delete _uart_transport;
		_uart_transport = nullptr;
	}
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

	// Initialize UART transport
	_uart_transport = new UartTransport();
	if (!_uart_transport) {
		PX4_ERR("Failed to allocate UART transport");
		return false;
	}

	if (!_uart_transport->init(device_path, baudrate)) {
		PX4_ERR("Failed to initialize UART transport on %s at %d baud", device_path, baudrate);
		delete _uart_transport;
		_uart_transport = nullptr;
		return false;
	}

	PX4_INFO("UART proxy initialized on %s at %d baud (Board ID: %d)", device_path, baudrate, _board_id);

	// Start work queue
	ScheduledWorkItem::ScheduleNow();

	return true;
}

void UorbUartProxy::Run()
{
	if (!_uart_transport) {
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
	UartFrame frame;
	while (_uart_transport->receiveFrame(frame)) {
		_stats.rx_messages++;
		_stats.rx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);

		// Update main board heartbeat tracking
		if (frame.header.board_id == BOARD_ID_X7_PLUS) {
			_last_main_board_heartbeat = hrt_absolute_time();
		}

		// Process based on message type
		switch (static_cast<UartMessageId>(frame.header.msg_id)) {
		case UartMessageId::WHEEL_LOADER_COMMAND: {
			if (frame.header.length == sizeof(wheel_loader_command_s)) {
				wheel_loader_command_s command;
				memcpy(&command, frame.payload, sizeof(command));
				_wheel_loader_command_pub.publish(command);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid wheel loader command length: %d", frame.header.length);
			}
			break;
		}

		case UartMessageId::WHEEL_LOADER_SETPOINT: {  // DEPRECATED - backward compatibility
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

		case UartMessageId::BOOM_COMMAND: {
			if (frame.header.length == sizeof(boom_command_s)) {
				boom_command_s command;
				memcpy(&command, frame.payload, sizeof(command));
				_boom_command_pub.publish(command);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid boom command length: %d", frame.header.length);
			}
			break;
		}

		case UartMessageId::BUCKET_COMMAND: {
			if (frame.header.length == sizeof(bucket_command_s)) {
				bucket_command_s command;
				memcpy(&command, frame.payload, sizeof(command));
				_bucket_command_pub.publish(command);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid bucket command length: %d", frame.header.length);
			}
			break;
		}

		case UartMessageId::TRACTION_CONTROL: {
			if (frame.header.length == sizeof(traction_control_s)) {
				traction_control_s command;
				memcpy(&command, frame.payload, sizeof(command));
				_traction_control_pub.publish(command);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid traction control length: %d", frame.header.length);
			}
			break;
		}

		case UartMessageId::WHEEL_SPEEDS_SETPOINT: {
			if (frame.header.length == sizeof(wheel_speeds_setpoint_s)) {
				wheel_speeds_setpoint_s setpoint;
				memcpy(&setpoint, frame.payload, sizeof(setpoint));
				_wheel_speeds_setpoint_pub.publish(setpoint);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid wheel speeds setpoint length: %d", frame.header.length);
			}
			break;
		}

		case UartMessageId::STEERING_COMMAND: {
			if (frame.header.length == sizeof(steering_command_s)) {
				steering_command_s command;
				memcpy(&command, frame.payload, sizeof(command));
				_steering_command_pub.publish(command);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid steering command length: %d", frame.header.length);
			}
			break;
		}

		case UartMessageId::ACTUATOR_OUTPUTS_FRONT: {
			// Only process if this is the front board
			if (_board_id == BOARD_ID_NXT_FRONT && frame.header.length == sizeof(actuator_outputs_s)) {
				actuator_outputs_s outputs;
				memcpy(&outputs, frame.payload, sizeof(outputs));
				_actuator_outputs_pub.publish(outputs);
			} else if (_board_id != BOARD_ID_NXT_FRONT) {
				// Ignore messages not intended for this board
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid front actuator outputs length: %d", frame.header.length);
			}
			break;
		}

		case UartMessageId::ACTUATOR_OUTPUTS_REAR: {
			// Only process if this is the rear board
			if (_board_id == BOARD_ID_NXT_REAR && frame.header.length == sizeof(actuator_outputs_s)) {
				actuator_outputs_s outputs;
				memcpy(&outputs, frame.payload, sizeof(outputs));
				_actuator_outputs_pub.publish(outputs);
			} else if (_board_id != BOARD_ID_NXT_REAR) {
				// Ignore messages not intended for this board
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid rear actuator outputs length: %d", frame.header.length);
			}
			break;
		}

		case UartMessageId::VEHICLE_STATUS: {
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

		case UartMessageId::HEARTBEAT: {
			// Heartbeat received from main board - already tracked above
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
	// Send wheel loader status from this board
	wheel_loader_status_s status;
	if (_wheel_loader_status_sub.update(&status)) {
		UartFrame frame;
		frame.header.sync = UART_SYNC_PATTERN;

		// Use appropriate message ID based on board type
		if (_board_id == BOARD_ID_NXT_FRONT) {
			frame.header.msg_id = static_cast<uint8_t>(UartMessageId::WHEEL_LOADER_STATUS_FRONT);
		} else {
			frame.header.msg_id = static_cast<uint8_t>(UartMessageId::WHEEL_LOADER_STATUS_REAR);
		}

		frame.header.board_id = _board_id;
		frame.header.length = sizeof(status);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &status, sizeof(status));

		if (_uart_transport->sendFrame(frame)) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send wheel loader status");
		}
	}

	// Send boom status
	boom_status_s boom_status;
	if (_boom_status_sub.update(&boom_status)) {
		UartFrame frame;
		frame.header.sync = UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(UartMessageId::BOOM_STATUS);
		frame.header.board_id = _board_id;
		frame.header.length = sizeof(boom_status);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &boom_status, sizeof(boom_status));

		if (_uart_transport->sendFrame(frame)) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send boom status");
		}
	}

	// Send bucket status
	bucket_status_s bucket_status;
	if (_bucket_status_sub.update(&bucket_status)) {
		UartFrame frame;
		frame.header.sync = UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(UartMessageId::BUCKET_STATUS);
		frame.header.board_id = _board_id;
		frame.header.length = sizeof(bucket_status);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &bucket_status, sizeof(bucket_status));

		if (_uart_transport->sendFrame(frame)) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send bucket status");
		}
	}

	// Send traction control status
	traction_control_status_s traction_status;
	if (_traction_control_status_sub.update(&traction_status)) {
		UartFrame frame;
		frame.header.sync = UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(UartMessageId::TRACTION_CONTROL_STATUS);
		frame.header.board_id = _board_id;
		frame.header.length = sizeof(traction_status);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &traction_status, sizeof(traction_status));

		if (_uart_transport->sendFrame(frame)) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send traction control status");
		}
	}

	// Send wheel encoders
	wheel_encoders_s encoders;
	if (_wheel_encoders_sub.update(&encoders)) {
		UartFrame frame;
		frame.header.sync = UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(UartMessageId::WHEEL_ENCODERS);
		frame.header.board_id = _board_id;
		frame.header.length = sizeof(encoders);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &encoders, sizeof(encoders));

		if (_uart_transport->sendFrame(frame)) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send wheel encoders");
		}
	}

	// Send steering status
	steering_status_s steering_status;
	if (_steering_status_sub.update(&steering_status)) {
		UartFrame frame;
		frame.header.sync = UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(UartMessageId::STEERING_STATUS);
		frame.header.board_id = _board_id;
		frame.header.length = sizeof(steering_status);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &steering_status, sizeof(steering_status));

		if (_uart_transport->sendFrame(frame)) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send steering status");
		}
	}
}

void UorbUartProxy::sendHeartbeat()
{
	UartFrame frame;
	frame.header.sync = UART_SYNC_PATTERN;
	frame.header.msg_id = static_cast<uint8_t>(UartMessageId::HEARTBEAT);
	frame.header.board_id = _board_id;
	frame.header.length = 0;
	frame.header.sequence = _tx_sequence++;
	frame.header.timestamp = hrt_absolute_time();

	if (_uart_transport->sendFrame(frame)) {
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

	const char *board_name = (_board_id == BOARD_ID_NXT_FRONT) ? "FRONT" : "REAR";

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
		return BOARD_ID_NXT_FRONT;
	case 1:
		return BOARD_ID_NXT_REAR;
	default:
		PX4_WARN("Invalid board type %d, defaulting to front", board_type);
		return BOARD_ID_NXT_FRONT;
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
