#include "window_covering.h"

#include <esp_matter_endpoint.h>
#include <esp_matter_cluster.h>
#include <esp_matter_feature.h>

esp_matter::endpoint_t *endpoint_create_window_covering(esp_matter::node_t *node) {
	esp_matter::endpoint::window_covering::config_t window_covering_controller_config;
	window_covering_controller_config.window_covering.feature_flags = esp_matter::cluster::window_covering::feature::lift::get_id();
	return esp_matter::endpoint::window_covering::create(node, &window_covering_controller_config, esp_matter::CLUSTER_FLAG_SERVER, NULL);
}