#include "bindings_core.h"
#include <mutex>
#include <vector>

const char *TAG = "BINDINGS_CORE";

class BooleanStateSubscriptionCallback : public chip::app::ReadClient::Callback {
	public:
		BooleanStateSubscriptionCallback() = default;
		~BooleanStateSubscriptionCallback() override = default;

		void OnReportBegin() override {};
		void OnReportEnd() override {};
		void OnSubscriptionEstablished(chip::SubscriptionId aSubscriptionId) override {
			// ESP_LOGI(TAG, "subscription established with id: " PRIu64, aSubscriptionId);
		};
		void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath, chip::TLV::TLVReader *aReader, const chip::app::StatusIB &aStatus) override {};
		void OnEventData(const chip::app::EventHeader &aEventHeader, chip::TLV::TLVReader *apData, const chip::app::StatusIB *aStatus) override {};
		void OnError(CHIP_ERROR aError) override {};
		void OnDone(chip::app::ReadClient * apReadClient) override {};
};

uint64_t SubscriptionManager::MakeKey(uint8_t fabric, uint64_t node, uint16_t ep) {
	return (static_cast<uint64_t>(fabric) << 56) | ((node & 0xFFFFFFFFFFFFULL) << 8) | (ep & 0xFFULL);
}

void SubscriptionManager::ReverseHash(uint64_t key, uint8_t &fabric, uint64_t &node, uint16_t &ep) {
	fabric = (key >> 56) & 0xFF;
	node = (key >> 8) & 0xFFFFFFFFFFFFULL;
	ep = key & 0xFF;
}

Subscription *SubscriptionManager::Find(const chip::app::Clusters::Binding::TableEntry &key) {
	uint64_t map_key = MakeKey(key.fabricIndex, key.nodeId, key.remote);
	auto it = m_subs.find(map_key);
	if (it != m_subs.end()) {
		return it->second.get();
	}
	return nullptr;
}

esp_err_t SubscriptionManager::StartSubscription(Subscription *sub) {
	// build the attribute path that we want to subscribe to based on the subscription details.
	chip::app::AttributePathParams attr_path;
	attr_path.mEndpointId = sub->remote_ep;
	attr_path.mClusterId = chip::app::Clusters::BooleanState::Id;
	attr_path.mAttributeId = chip::app::Clusters::BooleanState::Attributes::StateValue::Id;
	attr_path.mListIndex = 0;

	// build the request_handle that tells the binding manager we want to read this attribute before we subscribe to it.
	esp_matter::client::request_handle_t req {};
	req.type = esp_matter::client::READ_ATTR;
	req.attribute_path = attr_path;
	req.request_data = nullptr;

	// register a "one shot" request callback that will be invoked once the CASE session is established. inside of this callback, we will create a ReadClient that subscribes to the attribute and store that client inside the subscription object.
	auto once_cb = [](esp_matter::client::peer_device_t *peer, esp_matter::client::request_handle_t *req_handle, void *priv_data) {
		Subscription *sub = static_cast<Subscription *>(priv_data);
		ESP_LOGE(TAG, "CASE session established...performing validation READ.");
		// the readclient takes ownership of the callback object.
		auto cb = new BooleanStateSubscriptionCallback();
		esp_err_t rc = esp_matter::client::interaction::read::send_request(
            peer,
            &req_handle->attribute_path,
            1,
            nullptr,
            0,
            *cb
        );
		if (rc != ESP_OK) {
			ESP_LOGE(TAG, "failed to send read request: %d", rc);
			delete cb;
			return;
		}
		return (void)rc;
	};

	// store a pointer to the subscription in the request callback priv data.
	esp_err_t rc = esp_matter::client::set_request_callback(once_cb, nullptr, sub);
	if (rc != ESP_OK) {
		ESP_LOGE(TAG, "failed to set client callback: %d", rc);
		return rc;
	}

	// establish the CASE session.
	rc = esp_matter::client::connect(
		chip::Server::GetInstance().GetCASESessionManager(),
		sub->fabric_index,
		sub->remote_node_id,
		&req
	);
	if (rc != ESP_OK) {
		ESP_LOGE(TAG, "failed to connect to peer: %d", rc);
		// clean up the callback since we won't get a chance to use it.
		esp_matter::client::set_request_callback(nullptr, nullptr, nullptr);
		return rc;
	}
	return rc;
}

// call _once_ for each new binding entry.
esp_err_t SubscriptionManager::AddBinding(const chip::app::Clusters::Binding::TableEntry &entry, std::map<uint64_t, std::unique_ptr<Subscription>> &new_sub_assemble) {
	// build the map key for this entry.
	uint64_t key = MakeKey(entry.fabricIndex, entry.nodeId, entry.remote);
	
	// check if the subscription already exists
	auto it_old = m_subs.find(key);
	if (it_old != m_subs.end()) {
		// subscription already exists, and needs to continue to exist. shift ownership of the subscription from the old map to the new map so that it doesn't get cleaned up, and mark it as "added" by putting it in the new map.
		new_sub_assemble[key] = std::move(it_old->second);
		m_subs.erase(it_old);
		return ESP_OK;
	}

	// no existing subscription, create a new one and start it.
	auto sub = std::make_unique<Subscription>();
	sub->fabric_index = entry.fabricIndex;
	sub->remote_node_id = entry.nodeId;
	sub->remote_ep = entry.remote;
	sub->local_ep = entry.local;

	// keep a raw pointer to the subscription.
	Subscription *sub_ptr = sub.get();

	// insert into the temporary map that is being built during this update phase.
	new_sub_assemble[key] = std::move(sub);

	// start the subscription now. the callback will be owned by the ReadClient that is stored in the Subscription.
	esp_err_t rc = StartSubscription(sub_ptr);
	if (rc != ESP_OK) {
		new_sub_assemble.erase(key);
	}
	return ESP_OK;
}

esp_err_t SubscriptionManager::FinishAdditions(std::map<uint64_t, std::unique_ptr<Subscription>> &new_sub_assemble) {
	for (auto &kv : m_subs) {
		Subscription *sub = kv.second.get();
		AbortAndDelete(sub);
	}
	m_subs.clear();
	m_subs.swap(new_sub_assemble);
	return ESP_OK;
}

void myFooFunction() {
	ESP_LOGI(TAG, "This is a function in the bindings core file.");
}