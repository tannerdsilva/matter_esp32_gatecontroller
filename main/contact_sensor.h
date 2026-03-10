#pragma once

#include <esp_matter.h>
#include <esp_matter_core.h>

struct contact_sensor_context {
	esp_matter::endpoint_t *endpoint;
	esp_matter::cluster_t *boolean_state_cluster;
	esp_matter::attribute_t *state_attribute;
};
typedef struct contact_sensor_context contact_sensor_context_t;

esp_matter::endpoint_t *endpoint_create_contact_sensor(esp_matter::node_t *node, contact_sensor_context_t *context);