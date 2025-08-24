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
	_last_statistics_time(0),
	_uart_initialized(false)
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
	// Just start the work queue - UART will be initialized in Run()
	ScheduledWorkItem::ScheduleNow();
	return true;
}

void UorbUartBridge::Run()
{
	// Initialize UART if not done yet
	if (!_uart_initialized) {
		// Get UART parameters
		char device_path[32];
		param_get(param_find("UART_BRIDGE_DEV"), device_path, sizeof(device_path));

		int32_t baudrate;
		param_get(param_find("UART_BRIDGE_BAUD"), &baudrate);

		int32_t enable;
		param_get(param_find("UART_BRIDGE_EN"), &enable);

		if (!enable) {
			PX4_INFO("UART bridge disabled by parameter");
			return;
		}

		// Initialize UART transport
		_uart_transport = new distributed_uorb::UartTransport();
		if (!_uart_transport) {
			PX4_ERR("Failed to allocate UART transport");
			return;
		}

		if (_uart_transport->init(device_path, (speed_t)baudrate) < 0) {
			PX4_ERR("Failed to initialize UART transport on %s at %d baud", device_path, baudrate);
			delete _uart_transport;
			_uart_transport = nullptr;
			return;
		}

		PX4_INFO("UART bridge initialized on %s at %d baud", device_path, baudrate);
		_uart_initialized = true;
	}

	if (!_uart_transport || !_uart_transport->isReady()) {
		ScheduleDelayed(MAIN_LOOP_INTERVAL_US);
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
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::WHEEL_LOADER_SETPOINT);
		frame.header.board_id = distributed_uorb::BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(setpoint);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &setpoint, sizeof(setpoint));

		if (_uart_transport->sendFrame(frame) >= 0) {
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
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::ACTUATOR_OUTPUTS_FRONT);
		frame.header.board_id = distributed_uorb::BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(actuator_outputs);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &actuator_outputs, sizeof(actuator_outputs));

		if (_uart_transport->sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send front actuator outputs");
		}
	}

	// Process actuator outputs for rear wheel controller
	if (_actuator_outputs_rear_sub.update(&actuator_outputs)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::ACTUATOR_OUTPUTS_REAR);
		frame.header.board_id = distributed_uorb::BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(actuator_outputs);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &actuator_outputs, sizeof(actuator_outputs));

		if (_uart_transport->sendFrame(frame) >= 0) {
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
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::VEHICLE_STATUS);
		frame.header.board_id = distributed_uorb::BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(vehicle_status);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &vehicle_status, sizeof(vehicle_status));

		if (_uart_transport->sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send vehicle status");
		}
	}

	// Process traction control (X7+ -> both NXT boards)
	traction_control_s traction_control;
	if (_traction_control_sub.update(&traction_control)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::TRACTION_CONTROL);
		frame.header.board_id = distributed_uorb::BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(traction_control);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &traction_control, sizeof(traction_control));

		if (_uart_transport->sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send traction control");
		}
	}

	// Process boom command (X7+ -> NXT rear)
	boom_command_s boom_command;
	if (_boom_command_sub.update(&boom_command)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::BOOM_COMMAND);
		frame.header.board_id = distributed_uorb::BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(boom_command);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &boom_command, sizeof(boom_command));

		if (_uart_transport->sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send boom command");
		}
	}

	// Process bucket command (X7+ -> NXT front)
	bucket_command_s bucket_command;
	if (_bucket_command_sub.update(&bucket_command)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::BUCKET_COMMAND);
		frame.header.board_id = distributed_uorb::BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(bucket_command);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &bucket_command, sizeof(bucket_command));

		if (_uart_transport->sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send bucket command");
		}
	}

	// Process steering command (X7+ -> NXT rear)
	steering_command_s steering_command;
	if (_steering_command_sub.update(&steering_command)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::STEERING_COMMAND);
		frame.header.board_id = distributed_uorb::BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(steering_command);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &steering_command, sizeof(steering_command));

		if (_uart_transport->sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send steering command");
		}
	}

	// Process load lamp command (X7+ -> NXT rear)
	load_lamp_command_s load_lamp_command;
	if (_load_lamp_command_sub.update(&load_lamp_command)) {
		distributed_uorb::UartFrame frame;
		frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
		frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::LOAD_LAMP_COMMAND);
		frame.header.board_id = distributed_uorb::BOARD_ID_X7_PLUS;
		frame.header.length = sizeof(load_lamp_command);
		frame.header.sequence = _tx_sequence++;
		frame.header.timestamp = hrt_absolute_time();

		memcpy(frame.payload, &load_lamp_command, sizeof(load_lamp_command));

		if (_uart_transport->sendFrame(frame) >= 0) {
			_stats.tx_messages++;
			_stats.tx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);
		} else {
			_stats.tx_errors++;
			PX4_WARN("Failed to send load lamp command");
		}
	}
}

void UorbUartBridge::processIncomingMessages()
{
	distributed_uorb::UartFrame frame;
	while (_uart_transport->receiveFrame(frame) > 0) {
		_stats.rx_messages++;
		_stats.rx_bytes += sizeof(frame.header) + frame.header.length + sizeof(frame.crc);

		// Process based on message type
		switch (static_cast<distributed_uorb::UartMessageId>(frame.header.msg_id)) {
		case distributed_uorb::UartMessageId::LIMIT_SENSOR_BUCKET: {
			if (frame.header.length == sizeof(limit_sensor_s)) {
				limit_sensor_s sensor_data;
				memcpy(&sensor_data, frame.payload, sizeof(sensor_data));
				_limit_sensor_bucket_pub.publish(sensor_data);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid limit sensor length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::SLIP_ESTIMATION_FRONT: {
			if (frame.header.length == sizeof(slip_estimation_s)) {
				slip_estimation_s slip_data;
				memcpy(&slip_data, frame.payload, sizeof(slip_data));
				_slip_estimation_front_pub.publish(slip_data);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid front slip estimation length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::SLIP_ESTIMATION_REAR: {
			if (frame.header.length == sizeof(slip_estimation_s)) {
				slip_estimation_s slip_data;
				memcpy(&slip_data, frame.payload, sizeof(slip_data));
				_slip_estimation_rear_pub.publish(slip_data);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid rear slip estimation length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::BOOM_STATUS: {
			if (frame.header.length == sizeof(boom_status_s)) {
				boom_status_s status;
				memcpy(&status, frame.payload, sizeof(status));
				_boom_status_pub.publish(status);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid boom status length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::BUCKET_STATUS: {
			if (frame.header.length == sizeof(bucket_status_s)) {
				bucket_status_s status;
				memcpy(&status, frame.payload, sizeof(status));
				_bucket_status_pub.publish(status);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid bucket status length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::STEERING_STATUS: {
			if (frame.header.length == sizeof(steering_status_s)) {
				steering_status_s status;
				memcpy(&status, frame.payload, sizeof(status));
				_steering_status_pub.publish(status);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid steering status length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::HEARTBEAT: {
			// Update board heartbeat tracking
			if (frame.header.board_id == distributed_uorb::BOARD_ID_NXT_FRONT) {
				_last_front_heartbeat = hrt_absolute_time();
			} else if (frame.header.board_id == distributed_uorb::BOARD_ID_NXT_REAR) {
				_last_rear_heartbeat = hrt_absolute_time();
			}
			break;
		}

		case distributed_uorb::UartMessageId::HBRIDGE_STATUS_FRONT_0: {
			if (frame.header.length == sizeof(hbridge_status_s)) {
				hbridge_status_s status;
				memcpy(&status, frame.payload, sizeof(status));
				_hbridge_status_front_0_pub.publish(status);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid HBridge front 0 status length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::HBRIDGE_STATUS_FRONT_1: {
			if (frame.header.length == sizeof(hbridge_status_s)) {
				hbridge_status_s status;
				memcpy(&status, frame.payload, sizeof(status));
				_hbridge_status_front_1_pub.publish(status);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid HBridge front 1 status length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::HBRIDGE_STATUS_REAR_0: {
			if (frame.header.length == sizeof(hbridge_status_s)) {
				hbridge_status_s status;
				memcpy(&status, frame.payload, sizeof(status));
				_hbridge_status_rear_0_pub.publish(status);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid HBridge rear 0 status length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::HBRIDGE_STATUS_REAR_1: {
			if (frame.header.length == sizeof(hbridge_status_s)) {
				hbridge_status_s status;
				memcpy(&status, frame.payload, sizeof(status));
				_hbridge_status_rear_1_pub.publish(status);
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid HBridge rear 1 status length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::SENSOR_QUAD_ENCODER_FRONT: {
			if (frame.header.length == sizeof(sensor_quad_encoder_s)) {
				sensor_quad_encoder_s sensor_data;
				memcpy(&sensor_data, frame.payload, sizeof(sensor_data));

				// Determine which front instance to publish to based on the sensor instance
				if (sensor_data.instance == 0) {
					_sensor_quad_encoder_front_0_pub.publish(sensor_data);
				} else if (sensor_data.instance == 1) {
					_sensor_quad_encoder_front_1_pub.publish(sensor_data);
				}
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid sensor quad encoder front length: %d", frame.header.length);
			}
			break;
		}

		case distributed_uorb::UartMessageId::SENSOR_QUAD_ENCODER_REAR: {
			if (frame.header.length == sizeof(sensor_quad_encoder_s)) {
				sensor_quad_encoder_s sensor_data;
				memcpy(&sensor_data, frame.payload, sizeof(sensor_data));

				// Determine which rear instance to publish to based on the sensor instance
				// Map rear board instances to global instances 2 and 3
				if (sensor_data.instance == 0) {
					sensor_data.instance = 2; // Remap to global instance 2
					_sensor_quad_encoder_rear_0_pub.publish(sensor_data);
				} else if (sensor_data.instance == 1) {
					sensor_data.instance = 3; // Remap to global instance 3
					_sensor_quad_encoder_rear_1_pub.publish(sensor_data);
				}
			} else {
				_stats.rx_errors++;
				PX4_WARN("Invalid sensor quad encoder rear length: %d", frame.header.length);
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
	distributed_uorb::UartFrame frame;
	frame.header.sync = distributed_uorb::UART_SYNC_PATTERN;
	frame.header.msg_id = static_cast<uint8_t>(distributed_uorb::UartMessageId::HEARTBEAT);
	frame.header.board_id = distributed_uorb::BOARD_ID_X7_PLUS;
	frame.header.length = 0;
	frame.header.sequence = _tx_sequence++;
	frame.header.timestamp = hrt_absolute_time();

	if (_uart_transport->sendFrame(frame) >= 0) {
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

	if (_uart_transport && _uart_initialized) {
		PX4_INFO("UART transport initialized and ready");

		// Print current statistics
		const_cast<UorbUartBridge*>(this)->printStatistics();
	} else {
		PX4_INFO("UART transport not initialized");
	}

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
