/**
 * @file uart_protocol.cpp
 * @brief Implementation of UART communication protocol
 */

#include "uart_protocol.hpp"

namespace distributed_uorb {

// CRC16-CCITT implementation
uint16_t UARTProtocol::calculate_crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

bool UARTProtocol::validate_packet(const UARTPacket& packet)
{
    // Check sync bytes
    if (packet.sync[0] != UART_SYNC_BYTE1 || packet.sync[1] != UART_SYNC_BYTE2) {
        return false;
    }

    // Check version
    if (packet.version != PROTOCOL_VERSION) {
        return false;
    }

    // Check payload length
    if (packet.payload_len > MAX_PAYLOAD_SIZE) {
        return false;
    }

    // Verify CRC
    uint16_t calculated_crc = calculate_crc16((const uint8_t*)&packet,
                                             sizeof(UARTPacket) - sizeof(packet.crc16));

    return calculated_crc == packet.crc16;
}

size_t UARTProtocol::encode_packet(UARTPacket& packet, uint8_t source_node,
                                  uint16_t topic_id, MessageType msg_type,
                                  const void* data, size_t len, uint8_t sequence)
{
    if (len > MAX_PAYLOAD_SIZE) {
        return 0; // Payload too large
    }

    // Fill packet header
    packet.sync[0] = UART_SYNC_BYTE1;
    packet.sync[1] = UART_SYNC_BYTE2;
    packet.version = PROTOCOL_VERSION;
    packet.flags = (uint8_t)msg_type;
    packet.topic_id = topic_id;
    packet.payload_len = (uint16_t)len;
    packet.timestamp = 0; // To be filled by caller with actual timestamp
    packet.sequence = sequence;
    packet.source_node = source_node;

    // Copy payload
    if (data && len > 0) {
        memcpy(packet.payload, data, len);
    }

    // Zero out unused payload space
    if (len < MAX_PAYLOAD_SIZE) {
        memset(packet.payload + len, 0, MAX_PAYLOAD_SIZE - len);
    }

    // Calculate and set CRC
    packet.crc16 = calculate_crc16((const uint8_t*)&packet,
                                  sizeof(UARTPacket) - sizeof(packet.crc16));

    return sizeof(UARTPacket);
}

int UARTProtocol::decode_packet(const uint8_t* buffer, size_t buffer_len,
                               UARTPacket& packet, size_t& bytes_consumed)
{
    bytes_consumed = 0;

    // Need at least packet size
    if (buffer_len < sizeof(UARTPacket)) {
        return -1; // Incomplete data
    }

    // Look for sync pattern
    size_t sync_pos = 0;
    bool found_sync = false;

    for (size_t i = 0; i <= buffer_len - 2; i++) {
        if (buffer[i] == UART_SYNC_BYTE1 && buffer[i + 1] == UART_SYNC_BYTE2) {
            sync_pos = i;
            found_sync = true;
            break;
        }
    }

    if (!found_sync) {
        bytes_consumed = buffer_len > 1 ? buffer_len - 1 : 0;
        return -1; // No sync found
    }

    // Check if we have complete packet after sync
    if (buffer_len - sync_pos < sizeof(UARTPacket)) {
        bytes_consumed = sync_pos;
        return -1; // Incomplete packet
    }

    // Copy packet data
    memcpy(&packet, buffer + sync_pos, sizeof(UARTPacket));
    bytes_consumed = sync_pos + sizeof(UARTPacket);

    // Validate packet
    if (!validate_packet(packet)) {
        return -2; // Invalid packet
    }

    return 0; // Success
}

void UARTProtocol::create_heartbeat(UARTPacket& packet, uint8_t node_id, uint8_t sequence)
{
    uint32_t status = 0; // Node status information
    encode_packet(packet, node_id, 0, MSG_HEARTBEAT, &status, sizeof(status), sequence);
}

void UARTProtocol::create_time_sync(UARTPacket& packet, uint8_t node_id,
                                   uint64_t timestamp, uint8_t sequence)
{
    encode_packet(packet, node_id, 0, MSG_TIME_SYNC, &timestamp, sizeof(timestamp), sequence);
}

} // namespace distributed_uorb
