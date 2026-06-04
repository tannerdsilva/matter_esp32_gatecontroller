#pragma once

#include <esp_err.h>
#include <esp_matter.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

typedef void *app_driver_handle_t;

/** Initialize the switch driver
 *
 * This initializes the switch driver associated with the selected board.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
app_driver_handle_t app_driver_switch_init();

void datamodel_animation_timer_init(void);
esp_err_t _datamodel_animation_timer_cancel(void);

void motor_relay_init(void);
void motor_relay_toggle_async(void);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                           \
	{ \
		.radio_mode = RADIO_MODE_NATIVE, \
	}

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG() \
	{ \
		.host_connection_mode = HOST_CONNECTION_MODE_NONE, \
	}

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
    {                                                                                   \
        .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
    }
#endif
