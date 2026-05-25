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
#include <esp_matter_client.h>
#include <app/clusters/bindings/binding-table.h>
#include <app/clusters/boolean-state-server/boolean-state-cluster.h>
#include <app/clusters/window-covering-server/window-covering-server.h>
#include <esp_matter_providers.h>
#include <esp_matter_attribute.h>
#include <platform/CHIPDeviceEvent.h>

#include <app/ReadClient.h>
#include <app/ConcreteAttributePath.h>
#include <lib/core/TLVReader.h>
#include <app/server/Server.h>

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>

// led indicator support
#include "led_indicator.hpp"
led_indicator_subsystem_t led_indicator_subsystem;

#define LED_GPIO		GPIO_NUM_8
#define BLINK_MS		200
#define LED_COUNT		1

// pin layouts picked based on the esp32-h2 dev board.
#define GPIO_OUTPUT_COVER_OPEN			GPIO_NUM_2
#define GPIO_OUTPUT_COVER_CLOSE			GPIO_NUM_3
#define GPIO_OUTPUT_COVER_STOP			GPIO_NUM_4
#define GPIO_INPUT_COVER_CLOSED			GPIO_NUM_10

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
#include <custom_provider/dynamic_commissionable_data_provider.h>
#endif

#if CONFIG_ENABLE_SNTP_TIME_SYNC
#include <app/clusters/time-synchronization-server/DefaultTimeSyncDelegate.h>
#endif

#define TAG "app_main"

uint16_t switch_endpoint_id = 0;
static esp_matter::endpoint_t* s_cover_endpoint = nullptr;

// Callback for remote boolean state updates
class BooleanStateReadCallback : public chip::app::ReadClient::Callback {
public:
    virtual void OnSubscriptionEstablished(chip::SubscriptionId aSubscriptionId) override {
        ESP_LOGI(TAG, "Subscription established");
    }

    virtual void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath, 
                                 chip::TLV::TLVReader *aReader, 
                                 const chip::app::StatusIB &aStatus) override {
        if (aStatus.mStatus != chip::Protocols::InteractionModel::Status::Success) {
            return;
        }

        bool is_open = false;
        aReader->Next(); // Skip tag
        aReader->Get(is_open);

        uint16_t position = is_open ? 100 : 0;
        esp_matter_attr_val_t new_val = esp_matter_int16(position);
        esp_matter::attribute::set_val(switch_endpoint_id, 
            chip::app::Clusters::WindowCovering::Id, 
            chip::app::Clusters::WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, 
            &new_val);
            
        ESP_LOGI(TAG, "Remote contact: %s -> Local pos: %d", is_open ? "OPEN" : "CLOSED", position);
    }

    virtual void OnError(CHIP_ERROR aError) override {
        ESP_LOGI(TAG, "ReadClient Error: %s", ErrorStr(aError));
    }

    virtual void OnDone(chip::app::ReadClient * apReadClient) override {
        ESP_LOGI(TAG, "ReadClient Done");
    }
};

static BooleanStateReadCallback s_callback_handler;

// Helper to start subscription once peer device is available
void start_remote_subscription(chip::app::Clusters::Binding::TableEntry entry) {
    // Allocate request handle on heap as it must persist until callback
    esp_matter::client::request_handle_t *req_handle = new esp_matter::client::request_handle_t();
    req_handle->type = esp_matter::client::SUBSCRIBE_ATTR;
    
    // Store remote endpoint in attribute_path for later use
    req_handle->attribute_path.mEndpointId = entry.remote;
    req_handle->attribute_path.mClusterId = chip::app::Clusters::BooleanState::Id;
    req_handle->attribute_path.mAttributeId = chip::app::Clusters::BooleanState::Attributes::StateValue::Id;
    
    // Pass remote endpoint ID as request_data (can be retrieved in callback)
    req_handle->request_data = (void*)(uintptr_t)entry.remote;
    
    esp_err_t err = esp_matter::client::connect(nullptr, entry.fabricIndex, entry.nodeId, req_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate connection to node %llu", (unsigned long long)entry.nodeId);
        delete req_handle;
    }
}

// Callback invoked by client::connect when session is ready
static void connection_success_callback(esp_matter::client::peer_device_t *peer_device, esp_matter::client::request_handle_t *req_handle, void *priv_data) {
    if (!peer_device) {
        ESP_LOGE(TAG, "Peer device is null in connection success callback");
        return;
    }
    
    ESP_LOGI(TAG, "Connection established, starting subscription");
    
    // Recover remote endpoint ID from request_data
    chip::EndpointId remote_ep = (chip::EndpointId)(uintptr_t)req_handle->request_data;
    
    // Start subscription to the remote endpoint
    esp_err_t err = esp_matter::client::interaction::subscribe::send_request(
        peer_device, 
        &req_handle->attribute_path, 
        1, // attr_path_size
        nullptr, // event_path
        0,       // event_path_size
        1000,    // min_interval_ms
        5000,    // max_interval_ms
        true,    // keep_subscription
        true,    // auto_resubscribe
        s_callback_handler
    );
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start subscription: %d", err);
    } else {
        ESP_LOGI(TAG, "Subscription started to remote endpoint %d", remote_ep);
    }
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
		
	case chip::DeviceLayer::DeviceEventType::kBindingsChangedViaCluster: {
		ESP_LOGI(TAG, "bindings changed via cluster");
		
		chip::app::Clusters::Binding::Table &table = chip::app::Clusters::Binding::Table::GetInstance();
		size_t count = table.Size();

		for (size_t i = 0; i < count; i++) {
			chip::app::Clusters::Binding::TableEntry entry = table.GetAt(i);
			
			// only care about boolean state bindings
			if (entry.clusterId.has_value() && entry.clusterId.value() == chip::app::Clusters::BooleanState::Id) {
				ESP_LOGI(TAG, "Found BooleanState binding to node %llu ep %d", (unsigned long long)entry.nodeId, entry.remote);
				
                esp_err_t err = esp_matter::client::set_request_callback(
                    (esp_matter::client::request_callback_t)connection_success_callback,
                    nullptr,
                    nullptr
                );
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to register request callback");
                }
                
                start_remote_subscription(entry);
                break;
			}
		}
		break;
    }
    break;
    default:
        ESP_LOGI(TAG, "unhandled event %d", static_cast<int>(event->Type));
        break;
    }
}

static esp_err_t app_identification_cb(esp_matter::identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id, uint8_t effect_variant, void *priv_data) {
	ESP_LOGI(TAG, "identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
	return ESP_OK;
}

static esp_err_t app_attribute_update_cb(esp_matter::attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data) {
	ESP_LOGI(TAG, "attribute updated: endpoint_id=%d", endpoint_id);
	return ESP_OK;
}


extern "C" void app_main() {
	// nvs initialization.
	esp_err_t err = ESP_OK;
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_LOGW(TAG, "NVS partition was truncated or old version – erasing");
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);
	ESP_LOGI(TAG, "NVS init OK – ready for Matter");

	// initialize the led indicator subsystem
	app_driver_handle_t switch_handle = app_driver_switch_init();
	app_reset_button_register(switch_handle);
	led_indicator_init(&led_indicator_subsystem, LED_GPIO);

	// create the root Matter node and endpoint
	esp_matter::node::config_t node_config;
	esp_matter::node_t *node = esp_matter::node::create(&node_config, app_attribute_update_cb, app_identification_cb);
	ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "failed to create Matter node"));
	
	// create window covering endpoint using new API
	esp_matter::endpoint::window_covering::config_t wc_config;
	
	// FIX: Enable Lift Feature (0x0001) and Tilt Feature (0x0002) if needed.
	// For a gate, we definitely need Lift.
	wc_config.window_covering.feature_flags = 0x0001; // Enable Lift

	esp_matter::endpoint_t *window_covering_endpoint = esp_matter::endpoint::window_covering::create(node, &wc_config, 0, nullptr);
	ABORT_APP_ON_FAILURE(window_covering_endpoint != nullptr, ESP_LOGE(TAG, "failed to create window covering endpoint"));
	
	switch_endpoint_id = esp_matter::endpoint::get_id(window_covering_endpoint);
	s_cover_endpoint = window_covering_endpoint;

	ESP_LOGI(TAG, "Window Covering Endpoint created with endpoint id %d", switch_endpoint_id);

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
	
	esp_matter::client::binding_manager_init();
	// Updated start() signature expects intptr_t instead of void*
	err = esp_matter::start(app_event_cb, (intptr_t)0);
	ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "failed to start Matter, err:%d", err));
}
