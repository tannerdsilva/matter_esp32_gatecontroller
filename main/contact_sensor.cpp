#include "contact_sensor.h"

#include <esp_matter_endpoint.h>
#include <esp_matter_cluster.h>
#include <esp_matter_feature.h>
#include <esp_matter_attribute.h>
#include <esp_matter_attribute_utils.h>

using namespace esp_matter;
endpoint_t *endpoint_create_contact_sensor(esp_matter::node_t *node, contact_sensor_context_t *context) {
	endpoint::contact_sensor::config_t contact_sensor_config;
	context->endpoint = endpoint::contact_sensor::create(node, &contact_sensor_config, CLUSTER_FLAG_SERVER, NULL);
	context->boolean_state_cluster = cluster::boolean_state::create(context->endpoint, nullptr, CLUSTER_FLAG_SERVER);
	context->state_attribute = cluster::boolean_state::attribute::create_state_value(context->boolean_state_cluster, 0);
	return context->endpoint;
}

esp_err_t contact_sensor_set_state(contact_sensor_context_t *context, bool state) {
    if (context == nullptr || context->state_attribute == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_matter_attr_val_t val = esp_matter_bool(state);
    return attribute::set_val(context->state_attribute, &val, true);
}