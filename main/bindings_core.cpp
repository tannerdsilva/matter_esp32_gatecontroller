#include "bindings_core.h"
#include <mutex>
#include <vector>

const char *TAG = "BINDINGS_CORE";

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
		// auto *cb = new class BooleanStateSubscriptionCallback(sub);
		// the sdk will free the ReadClient once OnDone() fires.
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
		// move the existing unique_ptr to the new map to indicate that this subscription should be retained. the manager no longer owns this subscription after this point,
		new_sub_assemble.emplace(key, std::move(it_old->second));
		m_subs.erase(it_old);
		return ESP_OK;
	}
	
	auto sub = std::make_unique<Subscription>();
	sub->local_ep = entry.local;
	sub->remote_ep = entry.remote;
	sub->remote_node_id = entry.nodeId;
	sub->fabric_index = entry.fabricIndex;

	// keep a raw pointer for the callback (the map will own the Subscription)
	Subscription *sub_raw = sub.get();

	// insert into the *temporary* map that the caller is building.
	new_sub_assemble.emplace(key, std::move(sub));
	
	// ---------------------
	/*
	// attribute path params.
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

		esp_err_t readerr = esp_matter::client::interaction::read::send_request(
			peer,
			&req_handle->attribute_path,
			1,
			nullptr,
			0,
			*new class BooleanStateSubscriptionCallback(sub)
		);
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
	*/
	// if connect was successful, we are now waiting for the callback to fire with the result.
	return ESP_OK;
}


/*
void handle_binding_changed_event() {
	chip::app::Clusters::Binding::Table gtableInstance = chip::app::Clusters::Binding::Table::GetInstance();
	size_t tableSize = gtableInstance.Size();
	size_t i = 0;

	// iterate through the existing cached values and see if any of them have been removed
	for (const auto &kv : SubscriptionManager::GetInstance().m_subs) {
		const Subscription *sub = kv.second.get();
		chip::app::Clusters::Binding::TableEntry current_entry;
		current_entry.fabricIndex = sub->fabric_index;
		current_entry.nodeId = sub->remote_node_id;
		current_entry.remote = sub->remote_ep;
		current_entry.local = sub->local_ep;
		bool found = false;
		for (i = 0; i < tableSize; i++) {
			chip::app::Clusters::Binding::TableEntry iter_entry = gtableInstance.GetAt(i);
			if ((iter_entry.fabricIndex == current_entry.fabricIndex) &&
				(iter_entry.nodeId == current_entry.nodeId) &&
				(iter_entry.remote == current_entry.remote) &&
				(iter_entry.local == current_entry.local)) {
				found = true;
				break;
			}
		}
		if (!found) {
			ESP_LOGW(TAG, "Subscription for fabric %d, node 0x%llx, ep %d was removed. Cleaning up.", current_entry.fabricIndex, current_entry.nodeId, current_entry.remote);
			SubscriptionManager::GetInstance().RemoveBinding(current_entry);
		}

		if (!found) {
			ESP_LOGW(TAG, "Subscription for fabric %d, node 0x%llx, ep %d was removed. Cleaning up.", current_entry.fabricIndex, current_entry.nodeId, current_entry.remote);
			SubscriptionManager::GetInstance().RemoveBinding(current_entry);
		}
	}
	for (i = 0; i < tableSize; i++) {
		// store the current entry on the stack.
		chip::app::Clusters::Binding::TableEntry bindingTableEntry = gtableInstance.GetAt(i);
       
		// validate that this endpoint has a cluster ID that is not null and that we support it.
		if ((bindingTableEntry.clusterId != std::nullopt) && (bindingTableEntry.clusterId.value() != chip::app::Clusters::BooleanState::Id)) {
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

	std::vector<chip::app::Clusters::Binding::TableEntry> to_remove;
	
}
*/