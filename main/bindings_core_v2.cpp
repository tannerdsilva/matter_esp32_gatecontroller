#include "bindings_core_v2.h"
#include <app/server/Server.h>
#include <esp_matter_core.h>

const char *TAG = "BINDINGS_CORE";

// Static callback registration function
void connect_callback(chip::DeviceProxy *peer_device, esp_matter::client::request_handle_t *req_handle, void *priv_data) {
	auto *manager = static_cast<SubscriptionManager*>(priv_data);
	if (!manager || !peer_device) {
		return;
	}

	// Process all pending subscriptions for this device
	std::vector<std::unique_ptr<Subscription>> pending_subs;
	{
		std::lock_guard<std::mutex> lock(manager->m_mutex);
		
		// Take all pending subscriptions for any node
		// (esp_matter client uses same proxy for all connections to same device)
		auto it = manager->m_pending_subs.begin();
		while (it != manager->m_pending_subs.end()) {
			if (!it->second.empty()) {
				pending_subs = std::move(it->second);
				manager->m_pending_subs.erase(it++);
			} else {
				++it;
			}
		}
	}

	// Create subscriptions for all pending entries
	for (auto &sub : pending_subs) {
		auto *cb = new SubscriptionCallback();

		chip::app::AttributePathParams attr_path;
		attr_path.mEndpointId = sub->remote_ep;
		attr_path.mClusterId = chip::app::Clusters::BooleanState::Id;
		attr_path.mAttributeId = chip::app::Clusters::BooleanState::Attributes::StateValue::Id;
		attr_path.mListIndex = 0;

		esp_err_t rc = esp_matter::client::interaction::subscribe::send_request(
			peer_device,
			&attr_path,
			1,
			nullptr,
			0,
			0,
			1000,
			true,
			true,
			*cb
		);

		if (rc == ESP_OK) {
			{
				std::lock_guard<std::mutex> lock(manager->m_mutex);
				BindingKey key = {
					sub->remote_node_id,
					sub->fabric_index,
					sub->remote_ep
				};
				manager->m_subs[key] = std::move(sub);
			}
		} else {
			delete cb;
		}
	}
}

esp_err_t SubscriptionManager::StartSubscription(Subscription *sub) {
	if (!sub) {
		return ESP_ERR_INVALID_ARG;
	}
	chip::CASESessionManager *case_mgr = chip::Server::GetInstance().GetCASESessionManager();
    esp_matter::client::request_handle_t req;
    req.type = esp_matter::client::READ_ATTR;
	esp_err_t rc = esp_matter::client::connect(
		case_mgr,
		sub->fabric_index,
		sub->remote_node_id,
		nullptr
	);

	return rc;
}

esp_err_t SubscriptionManager::AddBinding(
	const chip::app::Clusters::Binding::TableEntry &entry,
	std::map<BindingKey, std::unique_ptr<Subscription>> &new_sub_assemble) {

	BindingKey key = {
		entry.nodeId,
		entry.fabricIndex,
		entry.remote
	};

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		if (m_subs.find(key) != m_subs.end()) {
			return ESP_OK;
		}
	}

	auto sub = std::make_unique<Subscription>();
	sub->local_ep = entry.local;
	sub->remote_ep = entry.remote;
	sub->remote_node_id = entry.nodeId;
	sub->fabric_index = entry.fabricIndex;
	sub->peer = nullptr;

	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_pending_subs[sub->remote_node_id].push_back(std::move(sub));
	}

	new_sub_assemble[key] = nullptr;

	return StartSubscription(nullptr);
}

esp_err_t SubscriptionManager::FinishAdditions(
	std::map<BindingKey, std::unique_ptr<Subscription>> &new_sub_assemble) {

	std::lock_guard<std::mutex> lock(m_mutex);

	for (auto it = m_subs.begin(); it != m_subs.end(); ) {
		if (new_sub_assemble.find(it->first) == new_sub_assemble.end()) {
			AbortAndDelete(it->second.get());
			it = m_subs.erase(it);
		} else {
			++it;
		}
	}

	return ESP_OK;
}

void SubscriptionManager::AbortAndDelete(Subscription *sub) {
	if (!sub) return;
	delete sub;
}

SubscriptionManager::~SubscriptionManager() {
	std::lock_guard<std::mutex> lock(m_mutex);

	for (auto &kv : m_subs) {
		AbortAndDelete(kv.second.get());
	}

	for (auto &kv : m_pending_subs) {
		for (auto &sub : kv.second) {
			AbortAndDelete(sub.get());
		}
	}

	m_subs.clear();
	m_pending_subs.clear();
}
