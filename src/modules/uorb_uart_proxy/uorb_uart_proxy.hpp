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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/wheel_loader_command.h>
#include <uORB/topics/wheel_loader_setpoint.h>
#include <uORB/topics/wheel_loader_status.h>
#include <uORB/topics/boom_command.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_command.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/traction_control.h>
#include <uORB/topics/traction_control_status.h>
#include <uORB/topics/wheel_speeds_setpoint.h>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/steering_command.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>

#include <lib/distributed_uorb/uart_transport/uart_transport.hpp>
#include <lib/distributed_uorb/uart_protocol/uart_protocol.hpp>
#include <lib/distributed_uorb/topic_registry/topic_registry.hpp>

using namespace time_literals;
using namespace distributed_uorb;

/**
 * uORB UART Proxy module for NXT boards
 *
 * This module runs on NXT-Dual controller boards and provides transparent
 * uORB messaging to/from the X7+ main board via UART. It receives commands
 * from the main board and sends back status information.
 */
class UorbUartProxy : public ModuleBase<UorbUartProxy>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	UorbUartProxy();
	~UorbUartProxy();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	/**
	 * Process incoming messages from X7+ main board
	 */
	void processIncomingMessages();

	/**
	 * Process outgoing messages to X7+ main board
	 */
	void processOutgoingMessages();

	/**
	 * Send heartbeat message to main board
	 */
	void sendHeartbeat();

	/**
	 * Print module statistics
	 */
	void printStatistics();

	/**
	 * Determine board ID based on parameter
	 */
	uint8_t getBoardId();

	// Module parameters
	DEFINE_PARAMETERS(
		(ParamString<32>) _param_uart_dev,
		(ParamInt<>) _param_uart_baud,
		(ParamInt<>) _param_board_type
	)

	// UART transport
	UartTransport *_uart_transport;

	// uORB subscriptions (outgoing to main board)
	uORB::Subscription _wheel_loader_status_sub{ORB_ID(wheel_loader_status)};
	uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
	uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};
	uORB::Subscription _traction_control_status_sub{ORB_ID(traction_control_status)};
	uORB::Subscription _wheel_encoders_sub{ORB_ID(wheel_encoders)};
	uORB::Subscription _steering_status_sub{ORB_ID(steering_status)};

	// uORB publications (incoming from main board)
	uORB::Publication<wheel_loader_command_s> _wheel_loader_command_pub{ORB_ID(wheel_loader_command)};
	uORB::Publication<wheel_loader_setpoint_s> _wheel_loader_setpoint_pub{ORB_ID(wheel_loader_setpoint)};  // DEPRECATED - backward compatibility
	uORB::Publication<boom_command_s> _boom_command_pub{ORB_ID(boom_command)};
	uORB::Publication<bucket_command_s> _bucket_command_pub{ORB_ID(bucket_command)};
	uORB::Publication<traction_control_s> _traction_control_pub{ORB_ID(traction_control)};
	uORB::Publication<wheel_speeds_setpoint_s> _wheel_speeds_setpoint_pub{ORB_ID(wheel_speeds_setpoint)};
	uORB::Publication<steering_command_s> _steering_command_pub{ORB_ID(steering_command)};
	uORB::Publication<actuator_outputs_s> _actuator_outputs_pub{ORB_ID(actuator_outputs)};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};

	// Communication statistics
	struct {
		uint32_t tx_messages;
		uint32_t tx_bytes;
		uint32_t tx_errors;
		uint32_t rx_messages;
		uint32_t rx_bytes;
		uint32_t rx_errors;
	} _stats;

	// Timing
	hrt_abstime _last_heartbeat_time;
	hrt_abstime _last_statistics_time;
	hrt_abstime _last_main_board_heartbeat;

	// Sequence tracking
	uint16_t _tx_sequence;

	// Configuration
	uint8_t _board_id;

	// Timing constants
	static constexpr uint32_t MAIN_LOOP_INTERVAL_US = 10_ms;
	static constexpr uint32_t HEARTBEAT_INTERVAL_US = 1_s;
	static constexpr uint32_t STATISTICS_INTERVAL_US = 10_s;
	static constexpr uint32_t HEARTBEAT_TIMEOUT_US = 3_s;

	// Module name
	static constexpr const char *MODULE_NAME = "uorb_uart_proxy";
};
