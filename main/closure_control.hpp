#pragma once

#include <esp_matter_core.h>

esp_err_t endpoint_create_closure_controller(node_t *node, endpoint_t **endpoint_out);