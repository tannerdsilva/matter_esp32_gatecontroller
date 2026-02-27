#pragma once

#include "wcman.h"
#include <esp_matter.h>
#include <esp_matter_core.h>

static chip::app::Clusters::WindowCovering::MyWindowCoveringManager window_covering_manager;
esp_matter::endpoint_t *endpoint_create_window_covering(esp_matter::node_t *node);