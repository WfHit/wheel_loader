
#include <lib/distributed_uorb/topic_registry/topic_registry.hpp>

#include <cstring>

namespace distributed_uorb {

const TopicInfo *TopicRegistry::findByName(const char *name)
{
	if (!name) {
		return nullptr;
	}

	for (size_t i = 0; i < getRegistrySize(); i++) {
		if (strcmp(topic_registry[i].name, name) == 0) {
			return &topic_registry[i];
		}
	}

	return nullptr;
}

const TopicInfo *TopicRegistry::findById(uint16_t id)
{
	for (size_t i = 0; i < getRegistrySize(); i++) {
		if (topic_registry[i].id == id) {
			return &topic_registry[i];
		}
	}

	return nullptr;
}

bool TopicRegistry::isPublishedBy(uint16_t topic_id, NodeId node)
{
	const TopicInfo *info = findById(topic_id);

	if (!info) {
		return false;
	}

	return info->publisher == node;
}

bool TopicRegistry::isSubscribedBy(uint16_t topic_id, NodeId node)
{
	const TopicInfo *info = findById(topic_id);

	if (!info) {
		return false;
	}

	// Check if node is in subscribers array
	for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
		if (info->subscribers[i] == node) {
			return true;
		}
	}

	return false;
}

} // namespace distributed_uorb
