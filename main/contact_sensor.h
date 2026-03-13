#pragma once

#include <stdint.h>
#include <esp_err.h>
#include "endpoint_config.h"

typedef struct {
	bool current_state;
	uint16_t endpoint_id;
} contact_sensor_context_t;

// initialize the sensor context and state.
esp_err_t contact_sensor_init(contact_sensor_context_t *context);

// update the sensor state and report the change to the Matter data model.
esp_err_t contact_sensor_update_state(contact_sensor_context_t *context, bool new_state);

// callback implementation
void contact_sensor_read_callback(uint16_t endpoint_id, uint16_t cluster_id, uint16_t attribute_id, uint8_t *buffer, uint16_t *length);
