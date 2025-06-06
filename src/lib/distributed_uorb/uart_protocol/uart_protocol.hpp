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
	WHEEL_LOADER_SETPOINT = 0x10,
	WHEEL_LOADER_STATUS_FRONT = 0x11,
	WHEEL_LOADER_STATUS_REAR = 0x12,
	ACTUATOR_OUTPUTS_FRONT = 0x20,
	ACTUATOR_OUTPUTS_REAR = 0x21,
	VEHICLE_STATUS = 0x30
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
