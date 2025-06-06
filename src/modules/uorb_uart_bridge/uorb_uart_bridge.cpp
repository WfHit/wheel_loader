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

#include "uorb_uart_bridge.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <drivers/drv_hrt.h>

UorbUartBridge::UorbUartBridge() :
	ModuleBase(MODULE_NAME),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_uart_transport(nullptr),
	_last_heartbeat_time(0),
	_last_statistics_time(0)
{
	// Initialize statistics
	memset(&_stats, 0, sizeof(_stats));
}

UorbUartBridge::~UorbUartBridge()
{
	// Stop work queue
	ScheduledWorkItem::deinit();

	// Clean up UART transport
	if (_uart_transport) {
		delete _uart_transport;
		_uart_transport = nullptr;
	}
}

bool UorbUartBridge::init()
{
	// Get UART parameters
	char device_path[64];
	param_get(param_find("UART_BRIDGE_DEV"), device_path);

	int32_t baudrate;
	param_get(param_find("UART_BRIDGE_BAUD"), &baudrate);

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

	PX4_INFO("UART bridge initialized on %s at %d baud", device_path, baudrate);

	// Start work queue
	ScheduledWorkItem::ScheduleNow();

	return true;
}

void UorbUartBridge::Run()
{
	if (!_uart_transport) {
		return;
	}

	hrt_abstime now = hrt_absolute_time();

	// Process outgoing messages (X7+ -> NXT boards)
	processOutgoingMessages();

	// Process incoming messages (NXT boards -> X7+)
	processIncomingMessages();

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

void UorbUartBridge::processOutgoingMessages()
{
	// Process wheel loader setpoint (X7+ -> both NXT boards)
	wheel_loader_setpoint_s setpoint;
	if (_wheel_loader_setpoint_sub.update(&setpoint)) {
		UartFrame frame;
		frame.header.sync = UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(UartMessageId::WHEEL_LOADER_SETPOINT);
		frame.header.board_id = BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(setpoint);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &setpoint, sizeof(setpoint));

		if (_uart_transport->sendFrame(frame)) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send wheel loader setpoint");
		}
	}

	// Process actuator outputs for front wheel controller
	actuator_outputs_s actuator_outputs;
	if (_actuator_outputs_front_sub.update(&actuator_outputs)) {
		UartFrame frame;
		frame.header.sync = UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(UartMessageId::ACTUATOR_OUTPUTS_FRONT);
		frame.header.board_id = BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(actuator_outputs);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &actuator_outputs, sizeof(actuator_outputs));

		if (_uart_transport->sendFrame(frame)) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send front actuator outputs");
		}
	}

	// Process actuator outputs for rear wheel controller
	if (_actuator_outputs_rear_sub.update(&actuator_outputs)) {
		UartFrame frame;
		frame.header.sync = UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(UartMessageId::ACTUATOR_OUTPUTS_REAR);
		frame.header.board_id = BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(actuator_outputs);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &actuator_outputs, sizeof(actuator_outputs));

		if (_uart_transport->sendFrame(frame)) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send rear actuator outputs");
		}
	}

	// Process vehicle status
	vehicle_status_s vehicle_status;
	if (_vehicle_status_sub.update(&vehicle_status)) {
		UartFrame frame;
		frame.header.sync = UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(UartMessageId::VEHICLE_STATUS);
		frame.header.board_id = BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(vehicle_status);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &vehicle_status, sizeof(vehicle_status));

		if (_uart_transport->sendFrame(frame)) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send vehicle status");
		}
	}
}

void UorbUartBridge::processIncomingMessages()
{
	UartFrame frame;
	while (_uart_transport->receiveFrame(frame)) {
		_stats.rx_messages++;
		_stats.rx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);

		// Process based on message type
		switch (static_cast<UartMessageId>(frame.header.msg_id)) {
		case UartMessageId::WHEEL_LOADER_STATUS_FRONT: {
			if (frame.header.length == sizeof(wheel_loader_status_s)) {
				wheel_loader_status_s status;
				memcpy(&status, frame.payload, sizeof(status));
				_wheel_loader_status_front_pub.publish(status);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid front wheel loader status length: %d", frame.header.length);
			}
			break;
		}

		case UartMessageId::WHEEL_LOADER_STATUS_REAR: {
			if (frame.header.length == sizeof(wheel_loader_status_s)) {
				wheel_loader_status_s status;
				memcpy(&status, frame.payload, sizeof(status));
				_wheel_loader_status_rear_pub.publish(status);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid rear wheel loader status length: %d", frame.header.length);
			}
			break;
		}

		case UartMessageId::HEARTBEAT: {
			// Update board heartbeat tracking
			if (frame.header.board_id == BOARD_ID_NXT_FRONT) {
				_last_front_heartbeat = hrt_absolute_time();
			} else if (frame.header.board_id == BOARD_ID_NXT_REAR) {
				_last_rear_heartbeat = hrt_absolute_time();
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

void UorbUartBridge::sendHeartbeat()
{
	UartFrame frame;
	frame.header.sync = UART_SYNC_PATTERN;
	frame.header.msg_id = static_cast<uint8_t>(UartMessageId::HEARTBEAT);
	frame.header.board_id = BOARD_ID_X7_PLUS;
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

void UorbUartBridge::printStatistics()
{
	hrt_abstime now = hrt_absolute_time();
	bool front_online = (now - _last_front_heartbeat) < HEARTBEAT_TIMEOUT_US;
	bool rear_online = (now - _last_rear_heartbeat) < HEARTBEAT_TIMEOUT_US;

	PX4_INFO("UART Bridge Stats - TX: %lu msgs, %lu bytes, %lu errors | RX: %lu msgs, %lu bytes, %lu errors",
		 _stats.tx_messages, _stats.tx_bytes, _stats.tx_errors,
		 _stats.rx_messages, _stats.rx_bytes, _stats.rx_errors);

	PX4_INFO("Board Status - Front: %s, Rear: %s",
		 front_online ? "ONLINE" : "OFFLINE",
		 rear_online ? "ONLINE" : "OFFLINE");
}

int UorbUartBridge::task_spawn(int argc, char *argv[])
{
	UorbUartBridge *instance = new UorbUartBridge();

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

int UorbUartBridge::print_status()
{
	PX4_INFO("UART Bridge Module Status");

	if (_uart_transport) {
		PX4_INFO("UART transport initialized");
	} else {
		PX4_INFO("UART transport not initialized");
		return PX4_OK;
	}

	// Print current statistics
	const_cast<UorbUartBridge*>(this)->printStatistics();

	return PX4_OK;
}

int UorbUartBridge::custom_command(int argc, char *argv[])
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

int UorbUartBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
uORB UART Bridge module for distributed uORB messaging over UART.

This module runs on the X7+ main board and provides transparent uORB messaging
to/from NXT controller boards via UART. It forwards wheel loader commands and
receives status information from front and rear wheel controllers.

### Examples
Start the bridge:
$ uorb_uart_bridge start

Check status:
$ uorb_uart_bridge status

Stop the bridge:
$ uorb_uart_bridge stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uorb_uart_bridge", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int uorb_uart_bridge_main(int argc, char *argv[])
{
	return UorbUartBridge::main(argc, argv);
}
