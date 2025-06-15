#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
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
#include <lib/distributed_uorb/topic_registry/topic_registry.hpp>

class UorbUartBridge : public ModuleBase<UorbUartBridge>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	UorbUartBridge();
	~UorbUartBridge() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	bool init();

private:
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UORB_UART_CFG>) _param_uart_config,  /**< UART config (0: disabled, 1: ttyS3, 2: ttyS4) */
		(ParamInt<px4::params::UORB_UART_BAUD>) _param_uart_baud    /**< UART baudrate */
	)

	// UART transport
	distributed_uorb::UartTransport _uart_transport;

	// Node information
	uint8_t _sequence_number{0};
	uint16_t _tx_sequence{0};

	// Subscriptions for outgoing topics
	uORB::Subscription _wheel_loader_command_sub{ORB_ID(wheel_loader_command)};
	uORB::Subscription _wheel_loader_setpoint_sub{ORB_ID(wheel_loader_setpoint)};  // DEPRECATED - backward compatibility
	uORB::Subscription _boom_command_sub{ORB_ID(boom_command)};
	uORB::Subscription _bucket_command_sub{ORB_ID(bucket_command)};
	uORB::Subscription _traction_control_sub{ORB_ID(traction_control)};
	uORB::Subscription _wheel_speeds_setpoint_sub{ORB_ID(wheel_speeds_setpoint)};
	uORB::Subscription _steering_command_sub{ORB_ID(steering_command)};
	uORB::Subscription _actuator_outputs_front_sub{ORB_ID(actuator_outputs), 0};
	uORB::Subscription _actuator_outputs_rear_sub{ORB_ID(actuator_outputs), 1};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// Publications for incoming topics
	uORB::Publication<wheel_loader_status_s> _wheel_loader_status_front_pub{ORB_ID(wheel_loader_status), 0};
	uORB::Publication<wheel_loader_status_s> _wheel_loader_status_rear_pub{ORB_ID(wheel_loader_status), 1};
	uORB::Publication<boom_status_s> _boom_status_pub{ORB_ID(boom_status)};
	uORB::Publication<bucket_status_s> _bucket_status_pub{ORB_ID(bucket_status)};
	uORB::Publication<traction_control_status_s> _traction_control_status_pub{ORB_ID(traction_control_status)};
	uORB::Publication<wheel_encoders_s> _wheel_encoders_pub{ORB_ID(wheel_encoders)};
	uORB::Publication<steering_status_s> _steering_status_pub{ORB_ID(steering_status)};

	// Statistics and heartbeat tracking
	struct {
		uint32_t tx_messages{0};
		uint32_t tx_bytes{0};
		uint32_t tx_errors{0};
		uint32_t rx_messages{0};
		uint32_t rx_bytes{0};
		uint32_t rx_errors{0};
	} _stats;
	
	uint32_t _tx_count{0};
	uint32_t _rx_count{0};
	uint32_t _tx_errors{0};
	uint32_t _rx_errors{0};
	hrt_abstime _last_heartbeat{0};
	hrt_abstime _last_front_heartbeat{0};
	hrt_abstime _last_rear_heartbeat{0};

	/**
	 * Initialize UART connection
	 */
	bool initUart();

	/**
	 * Process outgoing uORB messages
	 */
	void processOutgoingMessages();

	/**
	 * Process incoming UART messages
	 */
	void processIncomingMessages();

	/**
	 * Send a uORB message over UART
	 */
	bool sendMessage(uint16_t topic_id, const void *data, size_t data_size);

	/**
	 * Handle received UART frame
	 */
	void handleReceivedFrame(const distributed_uorb::UartFrame &frame);

	/**
	 * Send heartbeat message
	 */
	void sendHeartbeat();

	/**
	 * Get UART device path from parameter
	 */
	const char *getUartDevicePath();

	/**
	 * Get baudrate from parameter
	 */
	speed_t getBaudrate();
};

extern "C" __EXPORT int uorb_uart_bridge_main(int argc, char *argv[]);
