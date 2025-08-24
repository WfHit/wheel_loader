/**
 * @file topic_definitions.hpp
 * @brief Topic definitions for wheel loader distributed system
 */

#pragma once

#include <stdint.h>

namespace distributed_uorb {

// Topic IDs for wheel loader system
enum TopicID : uint16_t {
    // Commands from X7+ to NXT boards (0x1000-0x1FFF)
    TOPIC_WHEEL_LOADER_CMD_FRONT = 0x1001,
    TOPIC_WHEEL_LOADER_CMD_REAR = 0x1002,
    TOPIC_TRACTION_CTRL_FRONT = 0x1003,
    TOPIC_TRACTION_CTRL_REAR = 0x1004,
    TOPIC_BUCKET_CMD = 0x1005,
    TOPIC_BOOM_CMD = 0x1006,
    TOPIC_STEERING_CMD = 0x1007,

    // Feedback from NXT boards to X7+ (0x2000-0x2FFF)
    TOPIC_WHEEL_ENCODER_FRONT = 0x2001,
    TOPIC_WHEEL_ENCODER_REAR = 0x2002,
    TOPIC_BUCKET_STATUS = 0x2003,
    TOPIC_BOOM_STATUS = 0x2004,
    TOPIC_STEERING_STATUS = 0x2005,
    TOPIC_TRACTION_STATUS_FRONT = 0x2006,
    TOPIC_TRACTION_STATUS_REAR = 0x2007,

    // System status (any board can publish) (0x3000-0x3FFF)
    TOPIC_SYSTEM_STATUS = 0x3001,
    TOPIC_NODE_HEALTH = 0x3002,
    TOPIC_ERROR_REPORT = 0x3003,

    // Reserved for future use (0x4000-0xFFFF)
};

// Topic mapping structure
struct TopicMapping {
    uint16_t id;
    const char* name;
    size_t size;
    bool is_command;         // True if X7+ publishes (commands)
    bool is_feedback;        // True if NXT publishes (feedback)
    uint8_t target_node;     // Which NXT node handles this (1=front, 2=rear, 0=both)
    const char* zenoh_key;   // Zenoh topic key for external monitoring
};

// Get topic mapping by ID
const TopicMapping* get_topic_mapping(uint16_t topic_id);

// Get all topic mappings
const TopicMapping* get_all_topic_mappings(size_t& count);

// Check if topic is valid for a specific node
bool is_topic_valid_for_node(uint16_t topic_id, uint8_t node_id);

} // namespace distributed_uorb
