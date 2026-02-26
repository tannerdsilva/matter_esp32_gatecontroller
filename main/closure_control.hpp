#ifdef CONFIG_MODE_PRIMARY_CLOSURE
#pragma once

#include <esp_matter_core.h>

esp_matter::endpoint_t *endpoint_create_closure(esp_matter::node_t *node);

#endif // CONFIG_MODE_PRIMARY_CLOSURE