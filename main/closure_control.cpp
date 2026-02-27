// #ifdef CONFIG_MODE_PRIMARY_CLOSURE
#include "closure_control.h"

#include <esp_matter_endpoint.h>
#include <esp_matter_cluster.h>
#include <esp_matter_feature.h>

using namespace esp_matter;
endpoint_t *endpoint_create_closure(node_t *node) {
	endpoint::closure::config_t closure_controller_config;
	closure_controller_config.closure_control.feature_flags = cluster::closure_control::feature::motion_latching::get_id();
	return endpoint::closure::create(node, &closure_controller_config, CLUSTER_FLAG_SERVER, NULL);
}

// #endif // CONFIG_MODE_PRIMARY_CLOSURE