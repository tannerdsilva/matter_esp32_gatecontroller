#include "bindings_core_v2.h"
#include <mutex>
#include <vector>
#include <app/server/Server.h>

const char *TAG = "BINDINGS_CORE";

// mutex to protect shared map access
std::mutex s_mutex;

// callback class for handling subscription events
class SubscriptionCallback : public chip::app::ReadClient::Callback {
public:
    void OnReportBegin() override {
        ESP_LOGI("BINDINGS_CORE", "SUBSCRIPTION REPORT BEGIN");
    }
    void OnReportEnd() override {
        ESP_LOGI("BINDINGS_CORE", "SUBSCRIPTION REPORT END");
    }
    void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath, chip::TLV::TLVReader *aReader, const chip::app::StatusIB &aStatus) override {
        ESP_LOGI("BINDINGS_CORE", "ATTRIBUTE UPDATE RECEIVED");
    }
    void OnEventData(const chip::app::EventHeader &aEventHeader, chip::TLV::TLVReader *apData, const chip::app::StatusIB *aStatus) override {
        ESP_LOGI("BINDINGS_CORE", "EVENT UPDATE RECEIVED");
    }
    void OnError(CHIP_ERROR aError) override {
        ESP_LOGE("BINDINGS_CORE", "SUBSCRIPTION ERROR");
    }
    void OnSubscriptionEstablished(chip::SubscriptionId aSubscriptionId) override {
        ESP_LOGI("BINDINGS_CORE", "SUBSCRIPTION ESTABLISHED");
    }
    void OnDone(chip::app::ReadClient *apReadClient) override {
        ESP_LOGI("BINDINGS_CORE", "SUBSCRIPTION TERMINATED");
    }
};

// helper to create a unique key from binding entry
static BindingKey MakeKey(const chip::app::Clusters::Binding::TableEntry &entry) {
    BindingKey newKey = {
        .node_id = entry.nodeId,
        .fabric_index = entry.fabricIndex,
        .remote_ep = entry.remote
    };
    return newKey;
}

// helper to create a key from node id and endpoint
static BindingKey MakeKey(uint64_t node, uint8_t fabric, uint16_t ep) {
    return BindingKey{node, fabric, ep};
}

// start a new subscription on the remote peer
esp_err_t SubscriptionManager::StartSubscription(Subscription *sub) {
    auto *cb = new SubscriptionCallback();

	chip::app::AttributePathParams attr_path;
    attr_path.mEndpointId = sub->remote_ep;
    attr_path.mClusterId = chip::app::Clusters::BooleanState::Id;
    attr_path.mAttributeId = chip::app::Clusters::BooleanState::Attributes::StateValue::Id;
    attr_path.mListIndex = 0;

	esp_err_t rc = esp_matter::client::interaction::subscribe::send_request(
        sub->peer,
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

    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "failed to send subscribe request: %d", rc);
        // Clean up the callback if request failed
        delete cb; 
    }

    return rc;
}

// add a binding to the pending list
esp_err_t SubscriptionManager::AddBinding(const chip::app::Clusters::Binding::TableEntry &entry, std::map<BindingKey, std::unique_ptr<Subscription>> &new_sub_assemble) {
    BindingKey key = MakeKey(entry);

    // check if subscription already exists in current active list
    {
        std::lock_guard<std::mutex> lock(s_mutex);
        if (m_subs.find(key) != m_subs.end()) {
            return ESP_OK; // skip, already active
        }
    }

    auto sub = std::make_unique<Subscription>();
    sub->fabric_index   = entry.fabricIndex;
    sub->remote_node_id = entry.nodeId;
    sub->remote_ep      = entry.remote;
    sub->local_ep       = entry.local;
    sub->peer           = nullptr; // Will be set by StartSubscription

    new_sub_assemble[key] = std::move(sub);
    return ESP_OK;
}

// finish the update phase and remove stale subscriptions
esp_err_t SubscriptionManager::FinishAdditions(
    std::map<BindingKey, std::unique_ptr<Subscription>> &new_sub_assemble) {
    std::lock_guard<std::mutex> lock(s_mutex);

    // iterate old subscriptions to find removed ones
    for (auto it = m_subs.begin(); it != m_subs.end(); ) {
        // if key is not in new list, abort and remove
        if (new_sub_assemble.find(it->first) == new_sub_assemble.end()) {
            AbortAndDelete(it->second.get());
            it = m_subs.erase(it);
        } else {
            ++it; // keep existing active subscription
        }
    }

    // add new subscriptions and start them
    for (auto &kv : new_sub_assemble) {
        esp_err_t rc = StartSubscription(kv.second.get());
        if (rc != ESP_OK) {
            // failed to start, clean up local map entry
            AbortAndDelete(kv.second.get());
            m_subs.erase(kv.first);
        } else {
            // successfully started, move to active map
            m_subs[kv.first] = std::move(kv.second);
        }
    }

    return ESP_OK;
}

// cleanup and delete a specific subscription
void SubscriptionManager::AbortAndDelete(Subscription *sub) {
    if (!sub) return;
    // Delete the subscription struct
    delete sub;
}
