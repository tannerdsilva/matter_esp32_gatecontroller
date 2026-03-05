#pragma once

#include <map>
#include <mutex>

#include <esp_log.h>
#include <esp_matter_client.h>
#include <app/ReadClient.h>
#include <app/clusters/bindings/binding-table.h>
#include <app/AttributePathParams.h>
#include <lib/core/TLVReader.h>

// the primary structure of ownership for a subscription.
struct Subscription {
	uint16_t local_ep;
	uint16_t remote_ep;
	uint64_t remote_node_id;
	uint8_t fabric_index;
	chip::DeviceProxy *peer;
	// remove the readclient if the sdk manages it internally (need to confirm)
	std::unique_ptr<chip::app::ReadClient> read_client;
};

// key structure to safely store 64-bit node id
struct BindingKey {
	uint64_t node_id;
	uint8_t fabric_index;
	uint16_t remote_ep;
	bool operator < (const BindingKey &other) const {
		if (node_id != other.node_id) return node_id < other.node_id;
		if (fabric_index != other.fabric_index) return fabric_index < other.fabric_index;
		return remote_ep < other.remote_ep;
	}
};

struct SubscriptionManager {
	// map of active subscriptions
	std::map<BindingKey, std::unique_ptr<Subscription>> m_subs;

	// mutex for thread safety
	std::mutex m_mutex;

	// add a binding; returns ESP_OK if already present or new
	esp_err_t AddBinding(const chip::app::Clusters::Binding::TableEntry &entry, std::map<BindingKey, std::unique_ptr<Subscription>> &new_sub_assemble);

	// finish update phase, removing stale subscriptions
	esp_err_t FinishAdditions(std::map<BindingKey, std::unique_ptr<Subscription>> &new_sub_assemble);

	// internal helpers
	esp_err_t StartSubscription(Subscription *sub);
	void AbortAndDelete(Subscription *sub);
};