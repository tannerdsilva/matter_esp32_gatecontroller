#include <cstddef>
#include <cstdio>
#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <app_priv.h>
#include <app_reset.h>

#ifdef CONFIG_SUBSCRIBE_AFTER_BINDING
#include "ClientCallbackHandler.hpp"

#include <app/util/binding-table.h>
static const char *TAG = "MATTER_CLIENT";

class MatterClientReadHandler : public chip::app::ReadClient::Callback {
public:
	void OnSubscriptionEstablished(chip::SubscriptionId aSubscriptionId) override {
		ESP_LOGI(TAG, "SUBSCRIPTION ESTABLISHED");
	}

    void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath, chip::TLV::TLVReader *aReader, const chip::app::StatusIB &aStatus) override {
        // Handle the attribute data
		ESP_LOGI(TAG, "ATTRIBUTE DATA RECEIVED");
        if (aPath.mClusterId == chip::app::Clusters::BooleanState::Id) {
            if (aPath.mAttributeId == chip::app::Clusters::BooleanState::Attributes::StateValue::Id) {
                ESP_LOGI(TAG, "Received BooleanState attribute");
            }
        }
    }

    void OnEventData(const chip::app::EventHeader &aEventHeader, chip::TLV::TLVReader * apData, const chip::app::StatusIB *aStatus) override {
       ESP_LOGI(TAG, "EVENT DATA!");
    }

    void OnError(CHIP_ERROR aError) override {
        // Handle the error
        ESP_LOGI(TAG, "ReadClient Error: %s", ErrorStr(aError));
    }

    void OnDone(chip::app::ReadClient * apReadClient) override {
        // Cleanup after done
        ESP_LOGI(TAG, "ReadClient Done");
    }
};

static RemoteSubscription* find_or_create_subscription(const EmberBindingTableEntry &binding) {
    for (auto &s : g_subscriptions) {
        if (s.nodeId == binding.nodeId && s.fabricIndex == binding.fabricIndex &&
            s.endpoint == binding.remote && s.clusterId == binding.clusterId.value()) {
            return &s;
        }
    }
	ESP_LOGI(TAG, "CREATING NEW SUBSCRIPTION FOR NODE 0x" ChipLogFormatX64 " ENDPOINT %d CLUSTER " ChipLogFormatMEI, ChipLogValueX64(binding.nodeId), binding.remote, ChipLogValueMEI(binding.clusterId.value()));
	RemoteSubscription new_sub{};
    new_sub.nodeId       = binding.nodeId;
    new_sub.fabricIndex  = binding.fabricIndex;
    new_sub.endpoint     = binding.remote;
    new_sub.clusterId    = binding.clusterId.value();
    g_subscriptions.push_back(new_sub);
    return &g_subscriptions.back();
}
#endif
