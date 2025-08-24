#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/limit_sensor.h>
#include <uORB/topics/slip_estimation.h>
#include <uORB/topics/traction_control.h>
#include <uORB/topics/boom_command.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_command.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/steering_command.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/load_lamp_command.h>
#include <uORB/topics/sensor_quad_encoder.h>

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

	// Constants
	static constexpr uint32_t MAIN_LOOP_INTERVAL_US = 10000;  // 10ms
	static constexpr uint32_t HEARTBEAT_INTERVAL_US = 1000000; // 1s
	static constexpr uint32_t STATISTICS_INTERVAL_US = 5000000; // 5s
	static constexpr uint32_t HEARTBEAT_TIMEOUT_US = 2000000;  // 2s

	// UART transport
	distributed_uorb::UartTransport *_uart_transport;
	bool _uart_initialized;

	// Node information
	distributed_uorb::NodeId _node_id{distributed_uorb::NodeId::X7_MAIN};
	uint8_t _sequence_number{0};
	uint16_t _tx_sequence{0};

	// Timing variables
	hrt_abstime _last_heartbeat_time{0};
	hrt_abstime _last_statistics_time{0};
	hrt_abstime _last_front_heartbeat{0};
	hrt_abstime _last_rear_heartbeat{0};

	// Subscriptions for outgoing topics (X7+ → NXT)
	uORB::Subscription _actuator_outputs_front_sub{ORB_ID(actuator_outputs), 0};
	uORB::Subscription _actuator_outputs_rear_sub{ORB_ID(actuator_outputs), 1};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _traction_control_sub{ORB_ID(traction_control)};
	uORB::Subscription _boom_command_sub{ORB_ID(boom_command)};
	uORB::Subscription _bucket_command_sub{ORB_ID(bucket_command)};
	uORB::Subscription _steering_command_sub{ORB_ID(steering_command)};
	uORB::Subscription _load_lamp_command_sub{ORB_ID(load_lamp_command)};

	// Publications for incoming topics (NXT → X7+)
	uORB::Publication<limit_sensor_s> _limit_sensor_bucket_pub{ORB_ID(limit_sensor)};
	uORB::Publication<slip_estimation_s> _slip_estimation_front_pub{ORB_ID(slip_estimation), 0};
	uORB::Publication<slip_estimation_s> _slip_estimation_rear_pub{ORB_ID(slip_estimation), 1};
	uORB::Publication<boom_status_s> _boom_status_pub{ORB_ID(boom_status)};
	uORB::Publication<bucket_status_s> _bucket_status_pub{ORB_ID(bucket_status)};
	uORB::Publication<steering_status_s> _steering_status_pub{ORB_ID(steering_status)};

	// HBridge status publications (multi-instance, NXT → X7+)
	uORB::Publication<hbridge_status_s> _hbridge_status_front_0_pub{ORB_ID(hbridge_status), 0};
	uORB::Publication<hbridge_status_s> _hbridge_status_front_1_pub{ORB_ID(hbridge_status), 1};
	uORB::Publication<hbridge_status_s> _hbridge_status_rear_0_pub{ORB_ID(hbridge_status), 2};
	uORB::Publication<hbridge_status_s> _hbridge_status_rear_1_pub{ORB_ID(hbridge_status), 3};

	// Sensor quad encoder publications (multi-instance, NXT → X7+)
	uORB::Publication<sensor_quad_encoder_s> _sensor_quad_encoder_front_0_pub{ORB_ID(sensor_quad_encoder), 0};
	uORB::Publication<sensor_quad_encoder_s> _sensor_quad_encoder_front_1_pub{ORB_ID(sensor_quad_encoder), 1};
	uORB::Publication<sensor_quad_encoder_s> _sensor_quad_encoder_rear_0_pub{ORB_ID(sensor_quad_encoder), 2};
	uORB::Publication<sensor_quad_encoder_s> _sensor_quad_encoder_rear_1_pub{ORB_ID(sensor_quad_encoder), 3};

	// Statistics structure
	struct {
		uint32_t tx_messages;
		uint32_t rx_messages;
		uint32_t tx_bytes;
		uint32_t rx_bytes;
		uint32_t tx_errors;
		uint32_t rx_errors;
	} _stats;

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
