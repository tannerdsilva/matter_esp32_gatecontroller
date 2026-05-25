/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "esp_matter_client.h"
#include <cstddef>
#include <cstdio>
#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include "bsp/esp-bsp.h"

#include <app_priv.h>
#include <app_reset.h>

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;

static const char *TAG = "DRIVER";
extern uint16_t switch_endpoint_id;
extern esp_matter::endpoint_t* s_cover_endpoint;

// Helper to update local position directly (suitable for local button press in an emulator)
static void update_local_position(uint16_t target_pos) {
    esp_matter_attr_val_t new_val = esp_matter_int16(target_pos);
    esp_err_t err = esp_matter::attribute::set_val(switch_endpoint_id, 
        chip::app::Clusters::WindowCovering::Id, 
        chip::app::Clusters::WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, 
        &new_val);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Updated local position to %d", target_pos);
    } else {
        ESP_LOGE(TAG, "Failed to update position: %d", err);
    }
}

// Toggle button handler for window covering
static void app_driver_button_toggle_cb(void *arg, void *data) {
    ESP_LOGI(TAG, "Toggle button pressed");
    
    // Get current position
    esp_matter_attr_val_t pos_val;
    esp_err_t err = esp_matter::attribute::get_val(switch_endpoint_id, 
        chip::app::Clusters::WindowCovering::Id, 
        chip::app::Clusters::WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, 
        &pos_val);
        
    if (err == ESP_OK && pos_val.type == ESP_MATTER_VAL_TYPE_UINT16) {
        uint16_t current_pos = pos_val.val.u16;
        
        if (current_pos < 10) {
            // Open
            update_local_position(100);
        } else if (current_pos > 90) {
            // Close
            update_local_position(0);
        } else {
            // Stop (In this emulator, we just maintain current position)
            ESP_LOGI(TAG, "Stop button pressed (maintaining position %d)", current_pos);
        }
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get position: %d", err);
    }
}

// Stub for client subscribe callback (unused in local server mode)
static void app_driver_client_callback(client::peer_device_t *peer_device, client::request_handle_t *req_handle, void *priv_data) {
    (void)peer_device;
    (void)req_handle;
    (void)priv_data;
}

// Stub for group invoke callback (unused)
static void app_driver_client_group_invoke_command_callback(uint8_t fabric_index, client::request_handle_t *req_handle, void *priv_data) {
    (void)fabric_index;
    (void)req_handle;
    (void)priv_data;
}

app_driver_handle_t app_driver_switch_init() {
    button_handle_t btns[BSP_BUTTON_NUM];
    ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
    ESP_ERROR_CHECK(iot_button_register_cb(btns[0], BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL));
    
    // Register callbacks (even if mostly stubs for local mode, they satisfy the API)
    client::set_request_callback(app_driver_client_callback, app_driver_client_group_invoke_command_callback, NULL);
    ESP_LOGI(TAG, "APP DRIVER INITIALIZED");
    return (app_driver_handle_t)btns[0];
}
