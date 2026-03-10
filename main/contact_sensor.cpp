#include "contact_sensor.h"

#include <esp_matter_endpoint.h>
#include <esp_matter_cluster.h>
#include <esp_matter_feature.h>

using namespace esp_matter;
endpoint_t *endpoint_create_contact_sensor(esp_matter::node_t *node, contact_sensor_context_t *context) {
	endpoint::contact_sensor::config_t contact_sensor_config;
	endpoint_t *contact_sensor_endpoint endpoint::contact_sensor::create(node, &contact_sensor_config, CLUSTER_FLAG_SERVER, NULL);
	cluster_t *boolean_state_cluster = cluster::boolean_state::create(contact_sensor_endpoint, nullptr, CLUSTER_FLAG_SERVER);
	attribute_t *stateAttribute = cluster::boolean_state::attribute::create_state_value(boolean_state_cluster, 0);
}