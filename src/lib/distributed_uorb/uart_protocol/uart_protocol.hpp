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

	// Low-level actuator outputs
	ACTUATOR_OUTPUTS_FRONT = 0x20,
	ACTUATOR_OUTPUTS_REAR = 0x21,

	// System status
	VEHICLE_STATUS = 0x30,

	// Sensor data (NXT → X7+)
	SENSOR_QUAD_ENCODER_FRONT = 0x40,
	SENSOR_QUAD_ENCODER_REAR = 0x41,
	SENSOR_AS5600_BOOM = 0x42,
	LIMIT_SENSOR_BUCKET = 0x43,
	WHEEL_ENCODERS_FRONT = 0x44,
	WHEEL_ENCODERS_REAR = 0x45,

	// Slip estimation and traction (NXT → X7+, X7+ → NXT)
	SLIP_ESTIMATION_FRONT = 0x50,
	SLIP_ESTIMATION_REAR = 0x51,
	TRACTION_CONTROL = 0x52,

	// Hydraulic control commands (X7+ → NXT)
	BOOM_COMMAND = 0x60,
	BOOM_STATUS = 0x61,
	BUCKET_COMMAND = 0x62,
	BUCKET_STATUS = 0x63,

	// Steering control (X7+ → NXT)
	STEERING_COMMAND = 0x70,
	STEERING_STATUS = 0x71,

	// EKF and navigation data (X7+ → NXT)
	VEHICLE_LOCAL_POSITION = 0x80,
	VEHICLE_ATTITUDE = 0x81,
	VEHICLE_ODOMETRY = 0x82,

	// HBridge motor control and status (multi-instance)
	HBRIDGE_STATUS_FRONT_0 = 0x90,     // Front board, instance 0
	HBRIDGE_STATUS_FRONT_1 = 0x91,     // Front board, instance 1
	HBRIDGE_STATUS_REAR_0 = 0x92,      // Rear board, instance 0
	HBRIDGE_STATUS_REAR_1 = 0x93,      // Rear board, instance 1

	// Load lamp control commands (X7+ → NXT)
	LOAD_LAMP_COMMAND = 0xA0
};

// Message types (legacy - kept for compatibility)
enum class MessageType : uint8_t {
	DATA = 0x01,
	SUBSCRIBE = 0x02,
	ADVERTISE = 0x03,
	HEARTBEAT = 0x04,
	TIME_SYNC = 0x05,
	ACK = 0x06,
	NACK = 0x07
};

// Node identifiers (legacy - kept for compatibility)
enum class NodeId : uint8_t {
	X7_MAIN = 0x00,
	NXT_FRONT = 0x01,
	NXT_REAR = 0x02
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

// Legacy UART frame structure (kept for compatibility)
struct UARTFrame {
	uint8_t sync[2];
	uint8_t version;
	uint8_t msg_type;
	uint16_t topic_id;
	uint16_t payload_len;
	uint32_t timestamp;
	uint8_t sequence;
	uint8_t source_node;
	uint8_t payload[MAX_PAYLOAD_SIZE];
	uint16_t crc16;

	UARTFrame()
	{
		sync[0] = SYNC_BYTE_1;
		sync[1] = SYNC_BYTE_2;
		version = PROTOCOL_VERSION;
		msg_type = 0;
		topic_id = 0;
		payload_len = 0;
		timestamp = 0;
		sequence = 0;
		source_node = 0;
		crc16 = 0;
		memset(payload, 0, MAX_PAYLOAD_SIZE);
	}

	bool isValid() const
	{
		return sync[0] == SYNC_BYTE_1 &&
		       sync[1] == SYNC_BYTE_2 &&
		       version == PROTOCOL_VERSION &&
		       payload_len <= MAX_PAYLOAD_SIZE;
	}

	size_t totalSize() const
	{
		return sizeof(UARTFrame) - MAX_PAYLOAD_SIZE + payload_len;
	}
};

// Topic registry entry
struct TopicInfo {
	uint16_t id;
	const char *name;
	size_t size;
	NodeId publisher;
	NodeId subscribers[3];
};

// CRC16 calculation
uint16_t calculateCRC16(const uint8_t *data, size_t len);

} // namespace distributed_uorb
