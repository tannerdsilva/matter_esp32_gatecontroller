/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <app/clusters/bindings/binding-table.h>
#include <app/clusters/boolean-state-server/boolean-state-cluster.h>
#include <esp_matter_providers.h>
#include <esp_matter_attribute.h>
#include <platform/CHIPDeviceEvent.h>

#include "bindings_core.h"
#ifdef CONFIG_MODE_PRIMARY_CLOSURE
#include "closure_control.h"
#elifdef CONFIG_MODE_WINDOW_COVERING_LEGACY
#include "window_covering.h"
#endif

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>

// led indicator support
#include "led_indicator.hpp"
led_indicator_subsystem_t led_indicator_subsystem;

#define LED_GPIO   GPIO_NUM_8
#define BLINK_MS   200
#define LED_COUNT   1

// pin layouts picked based on the esp32-h2 dev board.
#define GPIO_OUTPUT_COVER_OPEN			GPIO_NUM_2
#define GPIO_OUTPUT_COVER_CLOSE			GPIO_NUM_3
#define GPIO OUTPUT_COVER_STOP			GPIO_NUM_4
#define GPIO_INPUT_COVER_CLOSED			GPIO_NUM_10

#ifdef CONFIG_SUBSCRIBE_AFTER_BINDING
// #include "ACLAssist.hpp"
#include <app/clusters/bindings/binding-table.h>
#include <esp_matter_client.h>
#include <app/AttributePathParams.h>
#include <access/SubjectDescriptor.h>   // <-- provides chip::Access::Target
#include <access/AccessControl.h>       // <-- AccessControl singleton

#include <app/ConcreteAttributePath.h>
#include <lib/core/TLVReader.h>
#include <app/server/Server.h>
// #include "ClientCallbackHandler.hpp"
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
#include <custom_provider/dynamic_commissionable_data_provider.h>
#endif

#if CONFIG_ENABLE_SNTP_TIME_SYNC
#include <app/clusters/time-synchronization-server/DefaultTimeSyncDelegate.h>
#endif

static const char *TAG = "app_main";
uint16_t switch_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;

#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
dynamic_commissionable_data_provider g_dynamic_passcode_provider;
#endif

static void init_event_cb(void *ptr, uint16_t endpoint_id) {
	ESP_LOGI(TAG, "init_event_cb called for endpoint_id=%d", endpoint_id);
}

static esp_err_t override_stop_cmd_handler(const ConcreteCommandPath &command_path, TLVReader &tlv_data, void *opaque_ptr) {
	ESP_LOGE(TAG, "override_stop_cmd_handler");
	return ESP_ERR_INVALID_ARG;
}

static esp_err_t override_cmd_handler(const ConcreteCommandPath &command_path, TLVReader &tlv_data, void *opaque_ptr) {
	ESP_LOGE(TAG, "override_cmd_handler");
	return ESP_ERR_INVALID_ARG;
}

static uint16_t event_stage = 0;
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg) {
    switch (event->Type) {
	case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
		ESP_LOGI(TAG, "interface IP Address Changed");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
		ESP_LOGI(TAG, "commissioning complete");
		event_stage = 2;
		led_indicator_set_color(&led_indicator_subsystem, 0, 255, 0); // green
		break;

	case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
		ESP_LOGI(TAG, "commissioning failed, fail safe timer expired");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
		ESP_LOGI(TAG, "commissioning session started");
		event_stage = 1;
		led_indicator_set_color(&led_indicator_subsystem, 0, 16, 16); // dim blue
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
		ESP_LOGI(TAG, "commissioning session stopped");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
		ESP_LOGI(TAG, "commissioning window opened");
		event_stage = 0;
		led_indicator_set_color(&led_indicator_subsystem, 0, 0, 128); // blue
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
		ESP_LOGI(TAG, "commissioning window closed");
		if (event_stage == 0) {
			led_indicator_set_color(&led_indicator_subsystem, 255, 0, 0); // red
		}
		break;
	case chip::DeviceLayer::DeviceEventType::kServerReady:
		ESP_LOGI(TAG, "server ready");
		break;

	case chip::DeviceLayer::DeviceEventType::kBindingsChangedViaCluster: {
		ESP_LOGI(TAG, "bindings changed via cluster");
		break;
    }
    break;

    default:
        ESP_LOGI(TAG, "unhandled event %d", static_cast<int>(event->Type));
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id, uint8_t effect_variant, void *priv_data) {
    ESP_LOGI(TAG, "identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data) {
	ESP_LOGI(TAG, "attribute updated: endpoint_id=%d", endpoint_id);
    return ESP_OK;
}

extern "C" void app_main() {
	esp_err_t err = ESP_OK;
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_LOGW(TAG, "NVS partition was truncated or old version – erasing");
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);
	ESP_LOGI(TAG, "NVS init OK – ready for Matter");

	app_driver_handle_t switch_handle = app_driver_switch_init();
	app_reset_button_register(switch_handle);
	led_indicator_init(&led_indicator_subsystem, LED_GPIO);

	node::config_t node_config;
	node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
	ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "failed to create Matter node"));
	
	#ifdef CONFIG_MODE_PRIMARY_CLOSURE
	
	// CLOSURE MODE (matter v1.5)
	
	endpoint_t *closure_endpoint = endpoint_create_closure(node);
	ABORT_APP_ON_FAILURE(closure_endpoint != nullptr, ESP_LOGE(TAG, "failed to create closure endpoint"));
	switch_endpoint_id = endpoint::get_id(closure_endpoint);
	ESP_LOGI(TAG, "Closure Control Endpoint created with endpoint id %d", switch_endpoint_id);
	
	#elifdef CONFIG_MODE_WINDOW_COVERING_LEGACY
	
	// WINDOW COVERING LEGACY MODE (matter v1.0)
	
	endpoint_t *window_covering_endpoint = endpoint_create_window_covering(node);
	ABORT_APP_ON_FAILURE(window_covering_endpoint != nullptr, ESP_LOGE(TAG, "failed to create window covering endpoint"));
	switch_endpoint_id = endpoint::get_id(window_covering_endpoint);
	ESP_LOGI(TAG, "Window Covering Endpoint created with endpoint id %d", switch_endpoint_id);
	
	#endif

	// the root endpoint of the data model.
	endpoint_t *root_node_ep = endpoint::get_first(node);
	cluster::binding::config_t bind_cfg;
	cluster::binding::create(root_node_ep, &bind_cfg, CLUSTER_FLAG_SERVER);

	#ifdef CONFIG_ENABLE_SNTP_TIME_SYNC
	ABORT_APP_ON_FAILURE(root_node_ep != nullptr, ESP_LOGE(TAG, "Failed to find root node endpoint"));
	cluster::time_synchronization::config_t time_sync_cfg;
	static chip::app::Clusters::TimeSynchronization::DefaultTimeSyncDelegate time_sync_delegate;
	time_sync_cfg.delegate = &time_sync_delegate;
	cluster_t *time_sync_cluster = cluster::time_synchronization::create(root_node_ep, &time_sync_cfg, CLUSTER_FLAG_SERVER);
	ABORT_APP_ON_FAILURE(time_sync_cluster != nullptr, ESP_LOGE(TAG, "Failed to create time_sync_cluster"));
	cluster::time_synchronization::feature::time_zone::config_t tz_cfg;
	cluster::time_synchronization::feature::time_zone::add(time_sync_cluster, &tz_cfg);
	#endif

	/*
	window_covering_device::config_t window_covering_device_config(static_cast<uint8_t>(chip::app::Clusters::WindowCovering::EndProductType::kUnknown));
	window_covering_device_config.window_covering.feature_flags = (uint32_t)chip::app::Clusters::WindowCovering::Feature::kLift;
	window_covering_device_config.window_covering.type = (uint8_t)chip::app::Clusters::WindowCovering::Type::kUnknown;
	window_covering_device_config.window_covering.config_status = 0x01;
	window_covering_device_config.window_covering.operational_status = 0x00;
	window_covering_device_config.window_covering.mode = 0x00;
	window_covering_device_config.window_covering.delegate = &window_covering_manager;
	endpoint_t *endpoint = window_covering_device::create(node, &window_covering_device_config, ENDPOINT_FLAG_NONE, NULL);
	cover_set_endpoint_id(endpoint::get_id(endpoint));
	cluster_t *windowCoveringCluser = cluster::window_covering::create(endpoint, &window_covering_device_config.window_covering, MATTER_CLUSTER_FLAG_INIT_FUNCTION | MATTER_CLUSTER_FLAG_SERVER);
	
	// create the up/open command
	command_t *upOrOpenCommand = cluster::window_covering::command::create_up_or_open(windowCoveringCluser);
	command::set_user_callback(upOrOpenCommand, override_cmd_handler);

	// create the down/close command
	command_t *downOrCloseCommand = cluster::window_covering::command::create_down_or_close(windowCoveringCluser);
	command::set_user_callback(downOrCloseCommand, override_cmd_handler);

	// create the stop command
	command_t *stopCommand = cluster::window_covering::command::create_stop_motion(windowCoveringCluser);
	command::set_user_callback(stopCommand, override_cmd_handler);

	// create the mandatory attributes for the window covering cluster.
	cover_currentPositionLiftPercent100ths = cluster::window_covering::attribute::create_target_position_lift_percent_100ths(windowCoveringCluser, 0);
	cover_targetPositionLiftPercent100ths = cluster::window_covering::attribute::create_target_position_lift_percent_100ths(windowCoveringCluser, 0);
	cover_operationalStatus = cluster::window_covering::attribute::create_operational_status(windowCoveringCluser, 0);
	cover_configStatus = cluster::window_covering::attribute::create_config_status(windowCoveringCluser, 0);
	cover_mode = cluster::window_covering::attribute::create_mode(windowCoveringCluser, 0);

	// 
	#ifdef CONFIG_SUBSCRIBE_AFTER_BINDING
	esp_matter::endpoint::contact_sensor::config_t contact_sensor_config = {};
	endpoint_t *contact_sensor_ep = esp_matter::endpoint::contact_sensor::create(node, &contact_sensor_config, CLUSTER_FLAG_SERVER, NULL);

	esp_matter::cluster_t *bool_srv = esp_matter::cluster::create(
        root_ep,
        chip::app::Clusters::BooleanState::Id,
        NULL);

	if (!bool_srv) {
		ESP_LOGE(TAG, "Failed to create boolean state cluster");
		return;
	}

	// add the required attribute for the boolean state cluster
	attribute_t *state_attr = cluster::boolean_state::attribute::create_state_value(bool_srv, false);

	if (!state_attr) {
		ESP_LOGE(TAG, "Failed to create state value attribute");
		return;
	}
	*/
	// endpoint_t *contact_sensor_ep = esp_matter::endpoint::contact_sensor::create(node, NULL, CLUSTER_FLAG_SERVER, NULL);
	// #endif

	ESP_LOGI(TAG, "window covering created with endpoint_id %d", switch_endpoint_id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
	/* Set OpenThread platform config */
	esp_openthread_platform_config_t config = {
		.radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
		.host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
		.port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
	};
	set_openthread_platform_config(&config);
#endif

#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
	esp_matter::set_custom_commissionable_data_provider(&g_dynamic_passcode_provider);
#endif

	err = esp_matter::start(app_event_cb, NULL);
	ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "failed to start Matter, err:%d", err));
}
