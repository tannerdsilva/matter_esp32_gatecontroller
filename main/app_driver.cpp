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

#include "driver/gpio.h"

#include <app_priv.h>
#include <app_reset.h>

static const char *TAG = "DRIVER";
static const char *TAG_CLIENT = "CLIENT";

/* ------------------------------------------------------------------ */
/*  Motor relay — single GPIO, momentary pulse                         */
/* ------------------------------------------------------------------ */
#define MOTOR_RELAY_GPIO    GPIO_NUM_1
#define MOTOR_RELAY_PULSE_MS 500        /* ms the relay is held active     */
#define RELAY_HALF_PULSE_MS  600         /* half-cycle for "open"/"close"   */
#define STARTUP_ANIM_DELAY_MS 3000       /* total animation duration ~3s    */

/* ------------------------------------------------------------------ */
/*  Startup animation: simulates opening then closing for ~3 seconds    */
/* ------------------------------------------------------------------ */
static void motor_relay_startup_animation(void)
{
    ESP_LOGI(TAG, "=== Starting motor relay self-test animation ===");
    
    // Phase 1: "Open" — pulse relay to move toward open position
    gpio_set_level(MOTOR_RELAY_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(RELAY_HALF_PULSE_MS));
    gpio_set_level(MOTOR_RELAY_GPIO, 0);
    ESP_LOGI(TAG, "  [anim] Opening...");
    
    // Pause to let the "gate" reach fully open position
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Phase 2: "Close" — pulse relay to move toward closed position
    gpio_set_level(MOTOR_RELAY_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(RELAY_HALF_PULSE_MS));
    gpio_set_level(MOTOR_RELAY_GPIO, 0);
    ESP_LOGI(TAG, "  [anim] Closing...");
    
    // Pause to let the "gate" reach fully closed position
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Finalize at closed state (position = 0)
    gpio_set_level(MOTOR_RELAY_GPIO, 0);
    ESP_LOGI(TAG, "  [anim] Complete — defaulting to CLOSED (0%%)");
}

static void motor_relay_init(void)
{
    // Note: gpio_config_t fields must be in declaration order!
    gpio_config_t io_conf = {
        .pin_bit_mask   = (1ULL << MOTOR_RELAY_GPIO),  // First field
        .mode           = GPIO_MODE_OUTPUT,            // Second field
        .pull_up_en     = GPIO_PULLUP_DISABLE,         // Third field
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,       // Fourth field
        .intr_type      = GPIO_INTR_DISABLE,           // Fifth field
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(MOTOR_RELAY_GPIO, 0);
    
    // Run startup animation before marking init complete
    motor_relay_startup_animation();
    
    ESP_LOGI(TAG, "Motor relay initialised on GPIO %d (%d ms pulse)",
             MOTOR_RELAY_GPIO, MOTOR_RELAY_PULSE_MS);
}

void motor_relay_toggle(void)
{
    gpio_set_level(MOTOR_RELAY_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(MOTOR_RELAY_PULSE_MS));
    gpio_set_level(MOTOR_RELAY_GPIO, 0);
    ESP_LOGI(TAG, "Motor relay toggled (%d ms pulse)", MOTOR_RELAY_PULSE_MS);
}

/* ------------------------------------------------------------------ */
/*  Matter / button driver                                             */
/* ------------------------------------------------------------------ */
using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;

extern uint16_t switch_endpoint_id;
extern esp_matter::endpoint_t* s_cover_endpoint;

// Helper to update local position directly (suitable for local button press)
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
        
        // Determine intent based on current position
        if (current_pos < 10) {
            // Near closed → open
            ESP_LOGI(TAG, "Closing position detected, toggling to OPEN");
        } else if (current_pos > 90) {
            // Near open → close  
            ESP_LOGI(TAG, "Open position detected, toggling to CLOSE");
        } else {
            // Mid-position → toggle (could be stop or direction change)
            ESP_LOGI(TAG, "Mid position %d, toggling relay", current_pos);
        }
    }
    
    // Pulse the relay
    motor_relay_toggle();
}

// Stub for client subscribe callback
static void app_driver_client_callback(client::peer_device_t *peer_device, 
                                       client::request_handle_t *req_handle, 
                                       void *priv_data) {
	ESP_LOGI(TAG_CLIENT, "Client callback triggered");
    (void)peer_device;
    (void)req_handle;
    (void)priv_data;
}

// Stub for group invoke callback
static void app_driver_client_group_invoke_command_callback(uint8_t fabric_index,
                                                            client::request_handle_t *req_handle,
                                                            void *priv_data) {
    (void)fabric_index;
    (void)req_handle;
    (void)priv_data;
}

app_driver_handle_t app_driver_switch_init() {
    motor_relay_init();
    
    button_handle_t btns[BSP_BUTTON_NUM];
    ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
    ESP_ERROR_CHECK(iot_button_register_cb(btns[0], BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL));
    
    // Register callbacks (satisfy API even if unused in local server mode)
    client::set_request_callback(app_driver_client_callback, app_driver_client_group_invoke_command_callback, NULL);
    ESP_LOGI(TAG, "APP DRIVER INITIALIZED");
    return (app_driver_handle_t)btns[0];
}
