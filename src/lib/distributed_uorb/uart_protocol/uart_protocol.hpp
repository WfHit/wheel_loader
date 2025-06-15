#pragma once

#include <cstdint>
#include <cstring>

namespace distributed_uorb {

// Protocol constants
static constexpr uint16_t UART_SYNC_PATTERN = 0xEB90;
static constexpr uint8_t SYNC_BYTE_1 = 0xEB;
static constexpr uint8_t SYNC_BYTE_2 = 0x90;
static constexpr uint8_t PROTOCOL_VERSION = 1;
static constexpr size_t MAX_PAYLOAD_SIZE = 256;
static constexpr size_t UART_FRAME_MAX_SIZE = 512;

// Board identifiers
static constexpr uint8_t BOARD_ID_X7_PLUS = 0x00;
static constexpr uint8_t BOARD_ID_NXT_FRONT = 0x01;
static constexpr uint8_t BOARD_ID_NXT_REAR = 0x02;

// Message IDs for wheel loader system
enum class UartMessageId : uint8_t {
	HEARTBEAT = 0x01,
	// Replace deprecated WHEEL_LOADER_SETPOINT with WHEEL_LOADER_COMMAND
	WHEEL_LOADER_COMMAND = 0x13,
	WHEEL_LOADER_STATUS_FRONT = 0x11,
	WHEEL_LOADER_STATUS_REAR = 0x12,
	
	// Hydraulic Control Messages (High Priority)
	BOOM_COMMAND = 0x14,
	BOOM_STATUS = 0x15,
	BUCKET_COMMAND = 0x16,
	BUCKET_STATUS = 0x17,
	
	// Drivetrain and Mobility (High Priority)
	ACTUATOR_OUTPUTS_FRONT = 0x20,
	ACTUATOR_OUTPUTS_REAR = 0x21,
	WHEEL_SPEEDS_SETPOINT = 0x22,
	WHEEL_ENCODERS = 0x23,
	STEERING_COMMAND = 0x24,
	STEERING_STATUS = 0x25,
	
	// Traction and Stability Control (High Priority)
	TRACTION_CONTROL = 0x26,
	TRACTION_CONTROL_STATUS = 0x27,
	PREDICTIVE_TRACTION = 0x28,
	
	// Load Management (High Priority)
	LOAD_SENSING = 0x29,
	LOAD_AWARE_TORQUE = 0x2A,
	
	// System Status (Existing)
	VEHICLE_STATUS = 0x30,
	
	// System Monitoring (Medium Priority)
	MODULE_STATUS = 0x42,
	SYSTEM_SAFETY = 0x43,
	LIMIT_SENSOR = 0x44,
	
	// Advanced Control (Medium Priority)
	WHEEL_DRIVE_COMMAND = 0x45,
	SLIP_ESTIMATION = 0x46,
	
	// Task Execution and Autonomy (Medium Priority)
	TASK_EXECUTION_COMMAND = 0x40,
	TERRAIN_ADAPTATION = 0x41,
	
	// Deprecated - kept for backward compatibility during transition
	WHEEL_LOADER_SETPOINT = 0x10  // DEPRECATED - use WHEEL_LOADER_COMMAND instead
};



// UART frame header structure
struct UartFrameHeader {
	uint16_t sync;        // Sync pattern
	uint8_t msg_id;       // Message ID
	uint8_t board_id;     // Board ID
	uint16_t length;      // Payload length
	uint16_t sequence;    // Sequence number
	uint32_t timestamp;   // Timestamp
} __attribute__((packed));

// UART frame structure
struct UartFrame {
	UartFrameHeader header;
	uint8_t payload[MAX_PAYLOAD_SIZE];
	uint16_t crc;

	UartFrame()
	{
		header.sync = UART_SYNC_PATTERN;
		header.msg_id = 0;
		header.board_id = 0;
		header.length = 0;
		header.sequence = 0;
		header.timestamp = 0;
		crc = 0;
		memset(payload, 0, MAX_PAYLOAD_SIZE);
	}

	bool isValid() const
	{
		return header.sync == UART_SYNC_PATTERN &&
		       header.length <= MAX_PAYLOAD_SIZE;
	}

	size_t totalSize() const
	{
		return sizeof(UartFrameHeader) + header.length + sizeof(crc);
	}
} __attribute__((packed));





// CRC16 calculation
uint16_t calculateCRC16(const uint8_t *data, size_t len);

} // namespace distributed_uorb
