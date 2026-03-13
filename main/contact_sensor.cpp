// contact_sensor.cpp
#include "contact_sensor.h"
#include "esp_matter.h"
#include "esp_log.h"

static const char *TAG = "ContactSensor";

// Global context for read callback access
static contact_sensor_context_t g_sensor_context = {0};

esp_err_t contact_sensor_init(contact_sensor_context_t *context) {
	if (!context) {
		ESP_LOGE(TAG, "Context is null");
		return ESP_ERR_NO_MEM;
	}
	
	// Copy context to global for read callback access
	g_sensor_context = *context;
	
	// Initialize state (Closed = false, Open = true)
	g_sensor_context.current_state = false; 
	
	ESP_LOGI(TAG, "Contact Sensor Initialized on Endpoint ID: %i", g_sensor_context.endpoint_id);
	return ESP_OK;
}

esp_err_t contact_sensor_update_state(contact_sensor_context_t *context, bool new_state) {
	if (!context) {
		return ESP_ERR_INVALID_ARG;
	}

	// Update local state
	context->current_state = new_state;
	g_sensor_context.current_state = new_state;

	ESP_LOGD(TAG, "Setting sensor state: %i", new_state);

	// --- MAPPING FROM endpoint_config.h ---
	// Endpoint: 1 (FIXED_ENDPOINT_ARRAY index 1)
	// Cluster: Boolean State (0x0045)
	// Attribute: StateValue (0x0000)
	
	uint16_t endpoint_id = 1; 
	uint16_t cluster_id = 0x0045; // Boolean State
	uint16_t attribute_id = 0x0000; // StateValue

	esp_matter_attr_val_t val = esp_matter_bool(new_state);

	// Write the attribute
	esp_err_t ret = esp_matter::attribute::set_val(endpoint_id, cluster_id, attribute_id, &val, true);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to write attribute %i - error %s", attribute_id, esp_err_to_name(ret));
		return ret;
	}

	return ESP_OK;
}

/*
* ZAP Init Callback Implementation
* This matches the name in CodeDrivenCallback.h
*/
void MatterBooleanStateClusterInitCallback(chip::EndpointId endpointId) {
	ESP_LOGI(TAG, "Boolean State Cluster Initialized on Endpoint %d", endpointId);
	
	// Initialize our context with the endpoint ID provided by ZAP
	// Note: In your config, this should be Endpoint 1
	g_sensor_context.endpoint_id = (uint16_t)endpointId;
	g_sensor_context.current_state = false;
	
	contact_sensor_init(&g_sensor_context);
}

/*
* Read Callback Implementation
* This is called when the Matter stack queries the attribute value.
*/
void contact_sensor_read_callback(uint16_t endpoint_id, uint16_t cluster_id, uint16_t attribute_id, uint8_t *buffer, uint16_t *length) {
	ESP_LOGD(TAG, "Read Callback: Endpoint %d, Cluster 0x%X, Attr 0x%X", endpoint_id, cluster_id, attribute_id);
	
	// Verify it's our attribute (Endpoint 1, Boolean State Cluster)
	if (endpoint_id == 1 && cluster_id == 0x0045 && attribute_id == 0x0000) {
		bool state = g_sensor_context.current_state;
		
		// Copy boolean to buffer (Matter stores bools as uint8_t 0 or 1)
		buffer[0] = state ? 1 : 0;
		*length = 1;
	} else {
		ESP_LOGE(TAG, "Unexpected read path");
		*length = 0;
	}
}
