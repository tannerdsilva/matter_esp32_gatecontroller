#pragma once

#include <map>

#include <esp_log.h>
#include <esp_matter_client.h>
#include <app/ReadClient.h>
#include <app/clusters/bindings/binding-table.h>
#include <app/AttributePathParams.h>
#include <lib/core/TLVReader.h>

// the primary structure of ownership for a subscription.
struct SubscriptionV2 {
	uint16_t local_ep;
	uint16_t remote_ep;
	uint64_t remote_node_id;
	uint8_t fabric_index;
	chip::DeviceProxy *peer;
	std::unique_ptr<chip::app::ReadClient> read_client;
};

struct SubscriptionManagerV2 {
	// map of active subscriptions
	std::map<uint64_t, std::unique_ptr<SubscriptionV2>> m_subs;

	// add a binding; returns ESP_OK if already present
    esp_err_t AddBinding(const chip::app::Clusters::Binding::TableEntry &entry, std::map<uint64_t, std::unique_ptr<SubscriptionV2>> &new_sub_assemble);
    // finish update phase, removing stale subscriptions
    esp_err_t FinishAdditions(std::map<uint64_t, std::unique_ptr<SubscriptionV2>> &new_sub_assemble);

    // internal helpers (public for simplicity)
    SubscriptionV2 *Find(const chip::app::Clusters::Binding::TableEntry &key);
    uint64_t MakeKey(uint8_t fabric, uint64_t node, uint16_t ep);
    void ReverseHash(uint64_t key, uint8_t &fabric, uint64_t &node, uint16_t &ep);
    esp_err_t StartSubscription(SubscriptionV2 *sub);
    void AbortAndDelete(SubscriptionV2 *sub);
};