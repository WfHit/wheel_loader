/**
 * @file uart_protocol.hpp
 * @brief UART communication protocol for distributed uORB system
 */

#pragma once

#include <stdint.h>
#include <string.h>

namespace distributed_uorb {

// Protocol constants
static constexpr uint8_t UART_SYNC_BYTE1 = 0xEB;
static constexpr uint8_t UART_SYNC_BYTE2 = 0x90;
static constexpr uint8_t PROTOCOL_VERSION = 1;
static constexpr size_t MAX_PAYLOAD_SIZE = 256;

// Node IDs
enum NodeID : uint8_t {
    NODE_X7_MAIN = 0,
    NODE_NXT_FRONT = 1,
    NODE_NXT_REAR = 2,
    NODE_BROADCAST = 0xFF
};

// Message types
enum MessageType : uint8_t {
    MSG_PUBLISH = 0x01,      // Publishing data
    MSG_SUBSCRIBE = 0x02,    // Subscribe request
    MSG_ADVERTISE = 0x03,    // Topic advertisement
    MSG_HEARTBEAT = 0x04,    // Keep-alive
    MSG_TIME_SYNC = 0x05,    // Time synchronization
    MSG_ERROR = 0x06         // Error message
};

// UART packet structure
struct UARTPacket {
    uint8_t  sync[2];        // UART_SYNC_BYTE1, UART_SYNC_BYTE2
    uint8_t  version;        // Protocol version
    uint8_t  flags;          // Bit flags for packet options
    uint16_t topic_id;       // Topic identifier
    uint16_t payload_len;    // Data length
    uint32_t timestamp;      // Microseconds since boot
    uint8_t  sequence;       // Sequence number
    uint8_t  source_node;    // Node that sent this packet
    uint8_t  payload[MAX_PAYLOAD_SIZE];  // Actual data
    uint16_t crc16;          // Checksum (calculated over entire packet except CRC)
} __attribute__((packed));

// Packet flags
enum PacketFlags : uint8_t {
    FLAG_RELIABLE = 0x01,    // Requires acknowledgment
    FLAG_COMPRESSED = 0x02,  // Payload is compressed
    FLAG_FRAGMENTED = 0x04   // Part of fragmented message
};

class UARTProtocol {
public:
    /**
     * Calculate CRC16 checksum
     */
    static uint16_t calculate_crc16(const uint8_t* data, size_t len);

    /**
     * Validate packet integrity
     */
    static bool validate_packet(const UARTPacket& packet);

    /**
     * Encode a packet for transmission
     */
    static size_t encode_packet(UARTPacket& packet, uint8_t source_node,
                               uint16_t topic_id, MessageType msg_type,
                               const void* data, size_t len, uint8_t sequence = 0);

    /**
     * Decode received data into packet
     * Returns 0 on success, -1 on incomplete data, -2 on invalid packet
     */
    static int decode_packet(const uint8_t* buffer, size_t buffer_len,
                            UARTPacket& packet, size_t& bytes_consumed);

    /**
     * Create heartbeat packet
     */
    static void create_heartbeat(UARTPacket& packet, uint8_t node_id, uint8_t sequence);

    /**
     * Create time sync packet
     */
    static void create_time_sync(UARTPacket& packet, uint8_t node_id,
                                uint64_t timestamp, uint8_t sequence);
};

} // namespace distributed_uorb
