#include "bindings_core.h"
#include <mutex>
#include <map>
#include <vector>
#include <esp_log.h>
#include <esp_matter_client.h>
#include <app/ReadClient.h>
#include <app/clusters/bindings/binding-table.h>
#include <app/AttributePathParams.h>
#include <lib/core/TLVReader.h>

const char *TAG = "BINDINGS_CORE";

// the primary structure of ownership for a subscription.
struct Subscription {
    uint16_t local_ep;
    uint16_t remote_ep;
    uint64_t remote_node_id;
    uint8_t fabric_index;
    chip::DeviceProxy *peer;
    std::unique_ptr<chip::app::ReadClient> read_client;
};

// manages all of the individual instances of subscriptions.
class SubscriptionManager {
	public:
		static SubscriptionManager &GetInstance() {
			static SubscriptionManager instance;
			return instance;
		}
		esp_err_t AddBinding(const chip::app::Clusters::Binding::TableEntry &entry);
		void RemoveBinding(const chip::app::Clusters::Binding::TableEntry &entry);
		Subscription *Find(const chip::app::Clusters::Binding::TableEntry &key);
	private:
		SubscriptionManager() = default;
		~SubscriptionManager() = default;
		SubscriptionManager(const SubscriptionManager &) = delete;
		SubscriptionManager &operator=(const SubscriptionManager &) = delete;

		std::mutex m_mutex;

		std::map<uint64_t, std::unique_ptr<Subscription>> m_subs;
};


// create a new key for the 
static uint64_t MakeKey(uint8_t fabric, uint64_t node, uint16_t ep) {
	return (static_cast<uint64_t>(fabric) << 56) | ((node & 0xFFFFFFFFFFFFULL) << 8) | (ep & 0xFFULL);
}

class BooleanStateSubscriptionCallback : public chip::app::ReadClient::Callback {
	public:
		// BooleanStateSubscriptionCallback(Subscription *sub) : m_subs(sub) {}
		void OnReportBegin() override {
			ESP_LOGI(TAG, "Report Begin");
		}
		void OnReportEnd() override {
			ESP_LOGI(TAG, "Report End");
		}

		void OnSubscriptionEstablished(chip::SubscriptionId aSubscriptionId) override {
			ESP_LOGE(TAG, "SUBSCRIPTION ESTABLISHED");
		}

		void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath, chip::TLV::TLVReader *aReader, const chip::app::StatusIB &aStatus) override {
			// Handle the attribute data
			ESP_LOGE(TAG, "\tATTRIBUTE DATA RECEIVED");
			if (aPath.mClusterId == chip::app::Clusters::BooleanState::Id) {
				ESP_LOGE(TAG, "\t\tATTRIBUTE DATA FOR BOOLEAN STATE CLUSTER");
				if (aPath.mAttributeId == chip::app::Clusters::BooleanState::Attributes::StateValue::Id) {
					ESP_LOGE(TAG, "\t\t\tReceived BooleanState attribute");
				}
			}
		}

		void OnEventData(const chip::app::EventHeader &aEventHeader, chip::TLV::TLVReader *apData, const chip::app::StatusIB *aStatus) override {
			ESP_LOGE(TAG, "EVENT DATA!");
		}

		void OnError(CHIP_ERROR aError) override {
			// Handle the error
			ESP_LOGE(TAG, "ReadClient Error: %s", ErrorStr(aError));
		}

		void OnDone(chip::app::ReadClient * apReadClient) override {
			// Cleanup after done
			ESP_LOGE(TAG, "ReadClient Done");
		}
};

// call _once_ for each new binding entry.
esp_err_t SubscriptionManager::AddBinding(const chip::app::Clusters::Binding::TableEntry &entry) {
	chip::app::AttributePathParams attr_path;
	attr_path.mEndpointId = entry.remote;
	attr_path.mClusterId = chip::app::Clusters::BooleanState::Id;
	attr_path.mAttributeId = chip::app::Clusters::BooleanState::Attributes::StateValue::Id;
	attr_path.mListIndex = 0;

	// attempt to read the attribute and if it works, create a subscription for it.
	esp_matter::client::request_handle_t req;
	req.type = esp_matter::client::READ_ATTR;
	req.attribute_path = attr_path;
	req.request_data = nullptr;

	// declare a single callback that will read the attribute response once, and if the value exists as expected, will create a subscription for the binding.
	auto once_cb = [](esp_matter::client::peer_device_t *peer, esp_matter::client::request_handle_t *req_handle, void *priv_data) {
		// priv points to the Subscription entry we have just allocated
		Subscription *sub = static_cast<Subscription *>(priv_data);

		/*
		esp_err_t readerr = esp_matter::client::interaction::read::send_request(
			peer,
			&req_handle->attribute_path,
			1,
			nullptr,
			0,
			*new class BooleanStateSubscriptionCallback(sub)
		);
		*/
		ESP_LOGI(TAG, "CASE session established...performing validation READ.");
		// ESP_LOGI(TAG, "CASE session established...performing validation READ. Node metadata: %d, %d, %d", peer->fabric_index, peer->node_id, peer->endpoint_id);
	};

	// allocate a subscription object and store it in the map *before* we start the async connection.
	auto sub = std::make_unique<Subscription>();
	sub->local_ep = entry.local;
	sub->remote_ep = entry.remote;
	sub->remote_node_id = entry.nodeId;
	sub->fabric_index = entry.fabricIndex;

	Subscription *sub_ptr = sub.get();
	// insert the subscription into the memorymap
	// std::lock_guard<std::mutex> lock(m_mutex);
	uint64_t key = MakeKey(entry.fabricIndex, entry.nodeId, entry.remote);
	m_subs.emplace(key, std::move(sub));

	esp_err_t rc = esp_matter::client::set_request_callback(once_cb, nullptr, sub_ptr);
	if (rc != ESP_OK) {
		ESP_LOGE(TAG, "failed to set client callback: %d", rc);
		RemoveBinding(entry);
		return rc;
	}

	// connect
	rc = esp_matter::client::connect(
		chip::Server::GetInstance().GetCASESessionManager(),
		entry.fabricIndex,
		entry.nodeId,
		&req
	);

	if (rc != ESP_OK) {
		ESP_LOGE(TAG, "failed to connect to peer: %d", rc);
		RemoveBinding(entry);
		return rc;
	}

	// if connect was successful, we are now waiting for the callback to fire with the result.
	return ESP_OK;
}

void SubscriptionManager::RemoveBinding(const chip::app::Clusters::Binding::TableEntry &entry) {
	uint64_t key = MakeKey(entry.fabricIndex, entry.nodeId, entry.remote);
	auto it = m_subs.find(key);
	if (it != m_subs.end()) {
		ESP_LOGW(TAG, "Removing subscription for fabric %d, node 0x%llx, ep %d", entry.fabricIndex, entry.nodeId, entry.remote);
		return;
	}

	Subscription *sub = it->second.get();

	if (sub->read_client) {
		// TODO: need to audit the source code and ensure this is the proper way to clean up a read client that may or may not have an active subscription. We may need to trigger an explicit "shutdown" of the read client before releasing it.
		sub->read_client.reset();
	}

	sub->peer = nullptr;

	m_subs.erase(it);
	// std::lock_guard<std::mutex> lock(m_mutex);
	// m_subs.erase(key);
}

Subscription *SubscriptionManager::Find(const chip::app::Clusters::Binding::TableEntry &key) {
	uint64_t map_key = MakeKey(key.fabricIndex, key.nodeId, key.remote);
	auto it = m_subs.find(map_key);
	if (it != m_subs.end()) {
		return it->second.get();
	}
	return nullptr;
}

void handle_binding_changed_event() {
	ESP_LOGE(TAG, "BINDING TABLE CHANGED");
	chip::app::Clusters::Binding::TableEntry binding_table;
	chip::app::Clusters::Binding::Table gtableInstance = chip::app::Clusters::Binding::Table::GetInstance();
	size_t tableSize = gtableInstance.Size();
	size_t i = 0;
	for (i = 0; i < tableSize; i++) {
		// store the current entry on the stack.
		chip::app::Clusters::Binding::TableEntry bindingTableEntry = gtableInstance.GetAt(i);
       
		// validate that this endpoint has a cluster ID that is not null and that we support it.
		if ((bindingTableEntry.clusterId != std::nullopt) && (bindingTableEntry.clusterId != chip::app::Clusters::BooleanState::Id)) {
			// not a cluster we can interact with, skip it.
			continue;
		}

		// check if we already have a subscription
		if (SubscriptionManager::GetInstance().Find(bindingTableEntry) != nullptr) {
			// we already have a subscription for this binding, skip it.
			continue;
		}

		// all checks done. ask the manager to start a subscription.
		esp_err_t rc = SubscriptionManager::GetInstance().AddBinding(bindingTableEntry);
		if (rc != ESP_OK) {
			ESP_LOGE(TAG, "failed to add binding subscription");
			continue;
		}
	}

	// cleanup the removed entries.
	std::vector<chip::app::Clusters::Binding::TableEntry> current_entries;
	for (i = 0; i < tableSize; i++) {
		current_entries.push_back(gtableInstance.GetAt(i));
	}
}
