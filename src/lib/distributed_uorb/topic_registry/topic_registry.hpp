#pragma once

#include <uORB/topics/wheel_loader_setpoint.h>
#include <uORB/topics/wheel_loader_status.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>

#include <lib/distributed_uorb/uart_protocol/uart_protocol.hpp>

namespace distributed_uorb {

// Topic IDs for distributed uORB
enum TopicId : uint16_t {
	// Commands from X7+ to NXT boards
	WHEEL_LOADER_SETPOINT = 0x1001,
	ACTUATOR_OUTPUTS_FRONT = 0x1002,
	ACTUATOR_OUTPUTS_REAR = 0x1003,

	// Feedback from NXT boards to X7+
	WHEEL_LOADER_STATUS_FRONT = 0x2001,
	WHEEL_LOADER_STATUS_REAR = 0x2002,

	// System topics
	VEHICLE_STATUS = 0x3001,
	SYSTEM_HEARTBEAT = 0xF001,
	TIME_SYNC = 0xF002
};

// Topic registry
static const TopicInfo topic_registry[] = {
	// Commands
	{WHEEL_LOADER_SETPOINT, "wheel_loader_setpoint", sizeof(wheel_loader_setpoint_s),
	 NodeId::X7_MAIN, {NodeId::NXT_FRONT, NodeId::NXT_REAR, NodeId::X7_MAIN}},

	{ACTUATOR_OUTPUTS_FRONT, "actuator_outputs_0", sizeof(actuator_outputs_s),
	 NodeId::X7_MAIN, {NodeId::NXT_FRONT, NodeId::X7_MAIN, NodeId::X7_MAIN}},

	{ACTUATOR_OUTPUTS_REAR, "actuator_outputs_1", sizeof(actuator_outputs_s),
	 NodeId::X7_MAIN, {NodeId::NXT_REAR, NodeId::X7_MAIN, NodeId::X7_MAIN}},

	// Feedback
	{WHEEL_LOADER_STATUS_FRONT, "wheel_loader_status_0", sizeof(wheel_loader_status_s),
	 NodeId::NXT_FRONT, {NodeId::X7_MAIN, NodeId::X7_MAIN, NodeId::X7_MAIN}},

	{WHEEL_LOADER_STATUS_REAR, "wheel_loader_status_1", sizeof(wheel_loader_status_s),
	 NodeId::NXT_REAR, {NodeId::X7_MAIN, NodeId::X7_MAIN, NodeId::X7_MAIN}},

	// System
	{VEHICLE_STATUS, "vehicle_status", sizeof(vehicle_status_s),
	 NodeId::X7_MAIN, {NodeId::NXT_FRONT, NodeId::NXT_REAR, NodeId::X7_MAIN}},
};

class TopicRegistry
{
public:
	static const TopicInfo *findByName(const char *name);
	static const TopicInfo *findById(uint16_t id);
	static bool isPublishedBy(uint16_t topic_id, NodeId node);
	static bool isSubscribedBy(uint16_t topic_id, NodeId node);
	static size_t getRegistrySize() { return sizeof(topic_registry) / sizeof(TopicInfo); }
};

} // namespace distributed_uorb
