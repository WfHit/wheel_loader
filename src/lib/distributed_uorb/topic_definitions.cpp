/**
 * @file topic_definitions.cpp
 * @brief Implementation of topic definitions
 */

#include "topic_definitions.hpp"
#include <uORB/topics/wheel_loader_command.h>
#include <uORB/topics/wheel_encoder.h>
#include <uORB/topics/bucket_command.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/traction_control_status.h>

namespace distributed_uorb {

// Topic mappings for wheel loader system
static const TopicMapping topic_mappings[] = {
    // Commands from X7+ to Front NXT
    {TOPIC_WHEEL_LOADER_CMD_FRONT, "wheel_loader_cmd_front",
     sizeof(wheel_loader_command_s), true, false, 1, "wl/cmd/front"},

    {TOPIC_TRACTION_CTRL_FRONT, "traction_ctrl_front",
     sizeof(traction_control_status_s), true, false, 1, "wl/traction/front"},

    {TOPIC_BUCKET_CMD, "bucket_cmd",
     sizeof(bucket_command_s), true, false, 1, "wl/cmd/bucket"},

    // Commands from X7+ to Rear NXT
    {TOPIC_WHEEL_LOADER_CMD_REAR, "wheel_loader_cmd_rear",
     sizeof(wheel_loader_command_s), true, false, 2, "wl/cmd/rear"},

    {TOPIC_TRACTION_CTRL_REAR, "traction_ctrl_rear",
     sizeof(traction_control_status_s), true, false, 2, "wl/traction/rear"},

    {TOPIC_STEERING_CMD, "steering_cmd",
     sizeof(wheel_loader_command_s), true, false, 2, "wl/cmd/steering"},

    // Feedback from Front NXT to X7+
    {TOPIC_WHEEL_ENCODER_FRONT, "wheel_encoder_front",
     sizeof(wheel_encoder_s), false, true, 1, "wl/feedback/encoder/front"},

    {TOPIC_BUCKET_STATUS, "bucket_status",
     sizeof(bucket_status_s), false, true, 1, "wl/feedback/bucket"},

    {TOPIC_TRACTION_STATUS_FRONT, "traction_status_front",
     sizeof(traction_control_status_s), false, true, 1, "wl/status/traction/front"},

    // Feedback from Rear NXT to X7+
    {TOPIC_WHEEL_ENCODER_REAR, "wheel_encoder_rear",
     sizeof(wheel_encoder_s), false, true, 2, "wl/feedback/encoder/rear"},

    {TOPIC_BOOM_STATUS, "boom_status",
     sizeof(boom_status_s), false, true, 2, "wl/feedback/boom"},

    {TOPIC_TRACTION_STATUS_REAR, "traction_status_rear",
     sizeof(traction_control_status_s), false, true, 2, "wl/status/traction/rear"},

    // System status (shared)
    {TOPIC_SYSTEM_STATUS, "system_status",
     32, false, false, 0, "wl/status/system"},  // Generic status message

    {TOPIC_NODE_HEALTH, "node_health",
     16, false, false, 0, "wl/status/health"},

    {TOPIC_ERROR_REPORT, "error_report",
     64, false, false, 0, "wl/status/error"},
};

static constexpr size_t topic_count = sizeof(topic_mappings) / sizeof(TopicMapping);

const TopicMapping* get_topic_mapping(uint16_t topic_id)
{
    for (size_t i = 0; i < topic_count; i++) {
        if (topic_mappings[i].id == topic_id) {
            return &topic_mappings[i];
        }
    }
    return nullptr;
}

const TopicMapping* get_all_topic_mappings(size_t& count)
{
    count = topic_count;
    return topic_mappings;
}

bool is_topic_valid_for_node(uint16_t topic_id, uint8_t node_id)
{
    const TopicMapping* mapping = get_topic_mapping(topic_id);
    if (!mapping) {
        return false;
    }

    // Check if this node should handle this topic
    if (mapping->target_node == 0) {
        return true; // Topic valid for all nodes
    }

    return mapping->target_node == node_id;
}

} // namespace distributed_uorb
