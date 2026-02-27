#include "bindings_core.h"
#include <unordered_map>
#include <mutex>
#include <map>
#include <esp_log.h>
#include <esp_matter_client.h>
#include <app/ReadClient.h>
#include <app/clusters/bindings/binding-table.h>
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

class BooleanStateSubscriptionCallback : public chip::app::ReadClient::Callback {
	public:
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

void handle_binding_changed_event() {
	ESP_LOGI(TAG, "BINDING TABLE CHANGED");
	chip::app::Clusters::Binding::TableEntry binding_table;
	for (auto &binding : chip::app::Clusters::Binding::Table::GetInstance()) {

	}
}
