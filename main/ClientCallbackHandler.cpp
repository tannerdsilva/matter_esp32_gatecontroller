#include "ClientCallbackHandler.hpp"

#include <cstddef>
#include <cstdio>
#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <app_priv.h>
#include <app_reset.h>

#include <app/server/Server.h>
#include <lib/core/Optional.h>

#ifdef CONFIG_SUBSCRIBE_AFTER_BINDING
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
#endif
