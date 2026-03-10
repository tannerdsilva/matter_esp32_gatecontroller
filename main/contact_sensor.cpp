#include "contact_sensor.h"

#include <esp_matter_endpoint.h>
#include <esp_matter_cluster.h>
#include <esp_matter_feature.h>

using namespace esp_matter;
endpoint_t *endpoint_create_contact_sensor(esp_matter::node_t *node) {
	endpoint::contact_sensor::config_t contact_sensor_config;
	return endpoint::contact_sensor::create(node, &contact_sensor_config, CLUSTER_FLAG_SERVER, NULL);
}