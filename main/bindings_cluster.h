#pragma once

#include <esp_matter.h>
#include <esp_matter_core.h>
#include <stdint.h>

struct binding_cluster_context {
	esp_matter::endpoint_t *local_endpoint;
	esp_matter::cluster_t *cluster;
	esp_matter::attribute_t *binding_attr;
};
typedef struct binding_cluster_context binding_cluster_context_t;

// Create the binding cluster on the specified endpoint
esp_err_t endpoint_create_binding_cluster(esp_matter::endpoint_t *endpoint, binding_cluster_context_t *context);
esp_err_t read_binding_attribute(const binding_cluster_context_t *context);
esp_err_t get_binding_count(const binding_cluster_context_t *context);
