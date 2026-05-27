#include "bindings_cluster.h"
#include <esp_matter_endpoint.h>
#include <esp_matter_cluster.h>
#include <esp_matter_attribute.h>
#include <esp_matter_data_model.h>

#include <app/clusters/bindings/binding-table.h>
#include <esp_log.h>

#define TAG "binding_cluster"

struct BindingKey {
	uint16_t fabric_index;
	uint64_t node_id;
	uint16_t cluster_id;
	uint16_t remote_ep;

	BindingKey(const chip::app::Clusters::Binding::TableEntry& entry)
		: fabric_index(static_cast<uint16_t>(entry.fabricIndex)),
		node_id(static_cast<uint64_t>(entry.nodeId)),
		cluster_id(entry.clusterId.has_value() ? static_cast<uint16_t>(entry.clusterId.value()) : 0),
		remote_ep(static_cast<uint16_t>(entry.remote)) {}

	bool operator==(const BindingKey& other) const {
		return fabric_index == other.fabric_index &&
			node_id == other.node_id &&
			cluster_id == other.cluster_id &&
			remote_ep == other.remote_ep;
	}
};

using namespace esp_matter;

esp_err_t endpoint_create_binding_cluster(esp_matter::endpoint_t *endpoint, binding_cluster_context_t *context) {
    if (endpoint == nullptr) return ESP_ERR_INVALID_ARG;
	// create the binding cluster
    cluster::binding::config_t bind_cfg;
    cluster_t *cluster = cluster::binding::create(endpoint, &bind_cfg, CLUSTER_FLAG_SERVER);
	if (cluster == nullptr) {
		ESP_LOGE(TAG, "Failed to create binding cluster");
		return ESP_ERR_NO_MEM;
	}
    // create the binding list attribute so that its contents can be found through power cycles.
    attribute_t *binding_attr = cluster::binding::attribute::create_binding(cluster, nullptr, 0, 0);
	if (binding_attr == nullptr) {
		ESP_LOGE(TAG, "Failed to create binding attribute");
		return ESP_ERR_NO_MEM;
	}

    context->local_endpoint = endpoint;
    context->cluster = cluster;
    context->binding_attr = binding_attr;

    ESP_LOGI(TAG, "Binding cluster created with persistence on endpoint");
    return ESP_OK;
}
esp_err_t get_binding_count(const binding_cluster_context_t *context) {
    if (context == nullptr || context->binding_attr == nullptr) {
        ESP_LOGE(TAG, "Context or binding attribute is null");
        return ESP_ERR_INVALID_ARG;
    }

    // Get attribute type to understand the value structure
    esp_matter_val_type_t type = attribute::get_val_type(context->binding_attr);
    // ESP_LOGI(TAG, "Binding attribute type: %d", type);

    // For Binding cluster, use the Binding Table API instead
    // The Binding Table API manages the attribute internally
    chip::app::Clusters::Binding::Table &table = chip::app::Clusters::Binding::Table::GetInstance();
    size_t count = table.Size();
    
    // ESP_LOGI(TAG, "Binding count: %zu", count);
    return count;
}

esp_err_t read_binding_attribute(const binding_cluster_context_t *context) {
    if (context == nullptr) {
        ESP_LOGE(TAG, "Context is null");
        return ESP_ERR_INVALID_ARG;
    }

    // Use the Binding Table API to iterate bindings
    chip::app::Clusters::Binding::Table &table = chip::app::Clusters::Binding::Table::GetInstance();
    size_t count = table.Size();

    if (count > 0) {
        // ESP_LOGI(TAG, "Found %zu bindings", count);
        
        for (size_t i = 0; i < count; i++) {
            chip::app::Clusters::Binding::TableEntry entry = table.GetAt(i);
        }
    } else {
        ESP_LOGI(TAG, "No bindings in table");
    }

    return ESP_OK;
}
