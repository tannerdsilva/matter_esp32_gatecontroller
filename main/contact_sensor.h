#pragma once

#include <esp_matter.h>
#include <esp_matter_core.h>

esp_matter::endpoint_t *endpoint_create_contact_sensor(esp_matter::node_t *node);