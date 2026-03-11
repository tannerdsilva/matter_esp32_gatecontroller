#include "bindings_cluster.h"
#include <esp_matter_endpoint.h>
#include <esp_matter_cluster.h>
#include <esp_matter_attribute.h>
#include <esp_matter_data_model.h>

#include <app/clusters/bindings/binding-table.h>
#include <esp_log.h>

#define TAG "binding_cluster"

using namespace esp_matter;

esp_err_t endpoint_create_binding_cluster(esp_matter::endpoint_t *endpoint, binding_cluster_context_t *context) {
    if (endpoint == nullptr) return ESP_ERR_INVALID_ARG;
	// create the binding cluster
    cluster::binding::config_t bind_cfg;
    cluster_t *cluster = cluster::binding::create(endpoint, &bind_cfg, CLUSTER_FLAG_SERVER);
    // create the binding list attribute so that its contents can be found through power cycles.
    attribute_t *binding_attr = cluster::binding::attribute::create_binding(cluster, nullptr, 0, 0);

    context->local_endpoint = endpoint;
    context->cluster = cluster;
    context->binding_attr = binding_attr;

    ESP_LOGI(TAG, "Binding cluster created with persistence on endpoint");
    return ESP_OK;
}

esp_err_t read_binding_table(const binding_cluster_context_t *context) {
    if (context == nullptr) {
        ESP_LOGE(TAG, "Context is null");
        return ESP_ERR_INVALID_ARG;
    }

    // Use the singleton Table API (handles persistence automatically)
    chip::app::Clusters::Binding::Table &table = chip::app::Clusters::Binding::Table::GetInstance();
    
    size_t count = table.Size();
    ESP_LOGI(TAG, "Found %zu bindings in NVS storage", count);

    for (size_t i = 0; i < count; i++) {
        chip::app::Clusters::Binding::TableEntry entry = table.GetAt(i);
        ESP_LOGI(TAG, "BINDING TABLE");
    }

    return ESP_OK;
}
