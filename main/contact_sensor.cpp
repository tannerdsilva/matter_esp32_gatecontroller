// contact_sensor.cpp
#include "contact_sensor.h"
#include "esp_matter.h"      // ESP-Matter C API
#include "esp_log.h"

static const char *TAG = "ContactSensor";

// We need a global way to access the context for the read callback
// In a real app, you might manage multiple endpoints, so an array is safer.
static contact_sensor_context_t g_sensor_context = {0};

void contact_sensor_init(contact_sensor_context_t *context) {
    if (!context) {
        ESP_LOGE(TAG, "Context is null");
        return;
    }
    
    // Copy context to global for read callback access
    g_sensor_context = *context;
    
    // Initialize state
    g_sensor_context.current_state = false; // Default closed
    
    ESP_LOGI(TAG, "Contact Sensor Initialized on Endpoint ID: %i", g_sensor_context.endpoint_id);
}

esp_err_t contact_sensor_set_state(contact_sensor_context_t *context, bool state) {
    if (!context) {
        return ESP_ERR_INVALID_ARG;
    }

    // Update local state
    context->current_state = state;

    // Update global state for read callback
    g_sensor_context.current_state = state;

    ESP_LOGD(TAG, "Setting sensor state: %i", state);

    // --- MAPPING FROM endpoint_config.h ---
    // Endpoint: 1 (FIXED_ENDPOINT_ARRAY index 1)
    // Cluster: Boolean State (0x00000045)
    // Attribute: StateValue (0x00000000)
    
    uint16_t endpoint_id = 1; 
    uint16_t cluster_id = chip::app::Clusters::BooleanState::Id; // Boolean State
    uint16_t attribute_id = chip::app::Clusters::BooleanState::Attributes::StateValue::Id; // StateValue

    esp_matter_attr_val_t val = esp_matter_bool(state);

    // Write the attribute
    esp_err_t ret = esp_matter_attribute_write(endpoint_id, cluster_id, attribute_id, &val, true);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write attribute %i - error %s", attribute_id, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/* 
 * Read Callback Implementation
 * 
 * The Matter stack will call this function when a controller reads the attribute.
 * You must register this with the ZAP generated code. 
 * 
 * In ESP-Matter Zap integration, the generated matter_endpoints.c usually 
 * looks for a function named: MatterBooleanStateCluster_AttributeReadCallback
 * OR you need to hook it into the generic callback dispatcher.
 * 
 * Since we are using the ESP-Matter wrapper, check if there is a callback 
 * registration macro in your main.cpp.
 */
void contact_sensor_read_callback(uint16_t endpoint_id, uint16_t cluster_id, uint16_t attribute_id, uint8_t *buffer, uint16_t *length) {
    // Verify it's our attribute
    if (endpoint_id == 1 && cluster_id == chip::app::Clusters::BooleanState::Id && attribute_id == chip::app::Clusters::BooleanState::Attributes::StateValue::Id) {
        bool state = g_sensor_context.current_state;
        
        // Copy boolean to buffer (Matter stores bools as uint8_t 0 or 1)
        buffer[0] = state ? 1 : 0;
        *length = 1;
    } else {
        ESP_LOGE(TAG, "Unexpected read path");
        *length = 0;
    }
}
