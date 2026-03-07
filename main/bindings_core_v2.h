#pragma once

#include <map>
#include <mutex>
#include <vector>
#include <memory>
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

// callback class for handling subscription events
class SubscriptionCallback : public chip::app::ReadClient::Callback {
public:
    void OnReportBegin() override {}
    void OnReportEnd() override {}
    void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath, 
                        chip::TLV::TLVReader *aReader, 
                        const chip::app::StatusIB &aStatus) override {}
    void OnEventData(const chip::app::EventHeader &aEventHeader, 
                    chip::TLV::TLVReader *apData, 
                    const chip::app::StatusIB *aStatus) override {}
    void OnError(CHIP_ERROR aError) override {}
    void OnSubscriptionEstablished(chip::SubscriptionId aSubscriptionId) override {}
    void OnDone(chip::app::ReadClient *apReadClient) override {}
};

// Subscription manager for handling binding-based subscriptions
class SubscriptionManager {
public:
    SubscriptionManager() = default;
    ~SubscriptionManager();

    // Add a binding; returns ESP_OK if already present or new
    esp_err_t AddBinding(const chip::app::Clusters::Binding::TableEntry &entry, 
                         std::map<BindingKey, std::unique_ptr<Subscription>> &new_sub_assemble);

    // Finish update phase, removing stale subscriptions
    esp_err_t FinishAdditions(std::map<BindingKey, std::unique_ptr<Subscription>> &new_sub_assemble);

    // Start a subscription (async - uses connect callback pattern)
    esp_err_t StartSubscription(Subscription *sub);

    // Internal helpers
    void AbortAndDelete(Subscription *sub);

    // Get active subscription count
    size_t GetSubscriptionCount() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_subs.size();
    }

    // Friend declaration for connect_callback to access private members
    friend void connect_callback(chip::DeviceProxy *peer_device, 
                                esp_matter::client::request_handle_t *req_handle, 
                                void *priv_data);

private:
    // Map of active subscriptions
    std::map<BindingKey, std::unique_ptr<Subscription>> m_subs;

    // Map of pending subscriptions waiting for connect callback
    std::map<uint64_t, std::vector<std::unique_ptr<Subscription>>> m_pending_subs;

    // Mutex for thread safety
    std::mutex m_mutex;
};

// external connect callback for esp_matter client
extern void connect_callback(chip::DeviceProxy *peer_device, 
                            esp_matter::client::request_handle_t *req_handle, 
                            void *priv_data);
