#include "bindings_core_v2.h"
#include <mutex>
#include <vector>

#include <vector>

const char *TAG = "BINDINGS_CORE";

class BooleanStateSubscriptionCallback : public chip::app::ReadClient::Callback {
public:
    BooleanStateSubscriptionCallback() = default;
    ~BooleanStateSubscriptionCallback() override = default;
    void OnReportBegin() override {
		ESP_LOGI(TAG, "SUBSCRIPTION REPORT BEGIN");
	}
    void OnReportEnd() override {
		ESP_LOGI(TAG, "SUBSCRIPTION REPORT END");
	}
    void OnSubscriptionEstablished(chip::SubscriptionId aSubscriptionId) override {
		ESP_LOGI(TAG, "SUBSCRIPTION ESTABLISHED");
	}
    void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath, chip::TLV::TLVReader *aReader, const chip::app::StatusIB &aStatus) override {
		ESP_LOGI(TAG, "ATTRIBUTE UPDATE RECEIVED");
	}
    void OnEventData(const chip::app::EventHeader &aEventHeader, chip::TLV::TLVReader *apData, const chip::app::StatusIB *aStatus) override {
		ESP_LOGI(TAG, "EVENT UPDATE RECEIVED");
	}
    void OnError(CHIP_ERROR aError) override {
		ESP_LOGE(TAG, "SUBSCRIPTION ERROR");
	}
    void OnDone(chip::app::ReadClient *apReadClient) override {
		ESP_LOGI(TAG, "SUBSCRIPTION DONE");
	}
};

uint64_t SubscriptionManagerV2::MakeKey(uint8_t fabric, uint64_t node, uint16_t ep) {
    return (static_cast<uint64_t>(fabric) << 56) | ((node & 0xFFFFFFFFFFFFULL) << 8) | (ep & 0xFFULL);
}

void SubscriptionManagerV2::ReverseHash(uint64_t key, uint8_t &fabric, uint64_t &node, uint16_t &ep) {
    fabric = (key >> 56) & 0xFF;
    node   = (key >> 8) & 0xFFFFFFFFFFFFULL;
    ep     = key & 0xFF;
}

SubscriptionV2 *SubscriptionManagerV2::Find(const chip::app::Clusters::Binding::TableEntry &key) {
    uint64_t map_key = MakeKey(key.fabricIndex, key.nodeId, key.remote);
    auto it = m_subs.find(map_key);
    return (it != m_subs.end()) ? it->second.get() : nullptr;
}

esp_err_t SubscriptionManagerV2::StartSubscription(SubscriptionV2 *sub) {
    chip::app::AttributePathParams attr_path;
    attr_path.mEndpointId   = sub->remote_ep;
    attr_path.mClusterId    = chip::app::Clusters::BooleanState::Id;
    attr_path.mAttributeId  = chip::app::Clusters::BooleanState::Attributes::StateValue::Id;
    attr_path.mListIndex    = 0;

    esp_matter::client::request_handle_t req{};
    req.type           = esp_matter::client::READ_ATTR;
    req.attribute_path = attr_path;
    req.request_data   = nullptr;

    auto once_cb = [](esp_matter::client::peer_device_t *peer,
                      esp_matter::client::request_handle_t *req_handle,
                      void *priv_data) {
        SubscriptionV2 *sub = static_cast<SubscriptionV2 *>(priv_data);
        ESP_LOGE(TAG, "CASE session established...performing validation READ.");
        auto *cb = new BooleanStateSubscriptionCallback();
        esp_err_t rc = esp_matter::client::interaction::read::send_request(
            peer,
            &req_handle->attribute_path,
            1,
            nullptr,
            0,
            *cb);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "failed to send read request: %d", rc);
            delete cb;
            return;
        }
    };

    esp_err_t rc = esp_matter::client::set_request_callback(once_cb, nullptr, sub);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "failed to set client callback: %d", rc);
        return rc;
    }

    rc = esp_matter::client::connect(
        chip::Server::GetInstance().GetCASESessionManager(),
        sub->fabric_index,
        sub->remote_node_id,
        &req);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "failed to connect to peer: %d", rc);
        esp_matter::client::set_request_callback(nullptr, nullptr, nullptr);
    }
    return rc;
}

esp_err_t SubscriptionManagerV2::AddBinding(
    const chip::app::Clusters::Binding::TableEntry &entry,
    std::map<uint64_t, std::unique_ptr<SubscriptionV2>> &new_sub_assemble) {
    uint64_t key = MakeKey(entry.fabricIndex, entry.nodeId, entry.remote);

    auto it_old = m_subs.find(key);
    if (it_old != m_subs.end()) {
        new_sub_assemble[key] = std::move(it_old->second);
        m_subs.erase(it_old);
        return ESP_OK;
    }

    auto sub = std::make_unique<SubscriptionV2>();
    sub->fabric_index   = entry.fabricIndex;
    sub->remote_node_id = entry.nodeId;
    sub->remote_ep      = entry.remote;
    sub->local_ep       = entry.local;

    SubscriptionV2 *sub_ptr = sub.get();
    new_sub_assemble[key] = std::move(sub);

    esp_err_t rc = StartSubscription(sub_ptr);
    if (rc != ESP_OK) {
        new_sub_assemble.erase(key);
    }
    return ESP_OK;
}

esp_err_t SubscriptionManagerV2::FinishAdditions(
    std::map<uint64_t, std::unique_ptr<SubscriptionV2>> &new_sub_assemble) {
    for (auto &kv : m_subs) {
        AbortAndDelete(kv.second.get());
    }
    m_subs.clear();
    m_subs.swap(new_sub_assemble);
    return ESP_OK;
}
