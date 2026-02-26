#ifdef CONFIG_MODE_PRIMARY_CLOSURE
#include "closure_control.hpp"

#include <esp_matter_endpoint.h>

using namespace esp_matter;
endpoint_t *endpoint_create_closure(node_t *node) {
	endpoint::closure::config_t closure_controller_config = {};
	return endpoint::closure::create(node, &closure_controller_config, CLUSTER_FLAG_SERVER, NULL);
}

#endif // CONFIG_MODE_PRIMARY_CLOSURE