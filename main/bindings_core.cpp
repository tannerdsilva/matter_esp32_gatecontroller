#include "bindings_core.h"
#include <unordered_map>
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
		// esp_err_t AddBinding(const chip::)
};

void handle_binding_changed_event() {
	ESP_LOGI(TAG, "BINDING TABLE CHANGED");
	for (auto &binding : chip::app::Clusters::Binding::Table::GetInstance()) {

	}
}
