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

#include "wcman.h"

// led indicator support
#include "led_indicator.hpp"
led_indicator_subsystem_t led_indicator_subsystem;

#define LED_GPIO		GPIO_NUM_8
#define BLINK_MS		200
#define LED_COUNT		1

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

// --- Delegate instance ---
static chip::app::Clusters::WindowCovering::MyWindowCoveringManager s_cover_delegate;

/* ================================================================
   Step-by-step manual construction of the Window Covering endpoint
   ================================================================ */
static esp_err_t create_manual_window_covering_endpoint(esp_matter::node_t *node)
{
    esp_err_t err = ESP_OK;
    esp_matter::endpoint_t *endpoint = nullptr;
    
    /* ---------------------------------------------------------------
       1. Create the endpoint (no flags, no priv_data for now)
       --------------------------------------------------------------- */
    endpoint = esp_matter::endpoint::create(node, 0, nullptr);
    if (endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create endpoint");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Endpoint created (internal ID: %d)", 
             esp_matter::endpoint::get_id(endpoint));
    
    /* ---------------------------------------------------------------
       2. Add the Window Covering device type to this endpoint
          DeviceType ID = 0x0202, Version = 5
       --------------------------------------------------------------- */
    err = esp_matter::endpoint::add_device_type(
        endpoint,
        ESP_MATTER_WINDOW_COVERING_DEVICE_TYPE_ID,
        ESP_MATTER_WINDOW_COVERING_DEVICE_TYPE_VERSION);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device type: %d", err);
        return err;
    }
    ESP_LOGI(TAG, "Device type 0x%04X v%d added", 
             ESP_MATTER_WINDOW_COVERING_DEVICE_TYPE_ID,
             ESP_MATTER_WINDOW_COVERING_DEVICE_TYPE_VERSION);
    
    /* ---------------------------------------------------------------
       3. Create the Descriptor Server cluster (mandatory on every endpoint)
          This populates: deviceTypeList, serverList, clientList, partsList
       --------------------------------------------------------------- */
    esp_matter::cluster::descriptor::config_t descriptor_config;
    esp_matter::cluster::descriptor::create(endpoint, &descriptor_config, 
                                             esp_matter::CLUSTER_FLAG_SERVER);
    ESP_LOGI(TAG, "Descriptor cluster created");
    
    /* ---------------------------------------------------------------
       4. Create the Identify Server cluster
          Required by the Matter spec for all endpoints that are not
          part of the root node. We use a 10-second identify time.
       --------------------------------------------------------------- */
    esp_matter::cluster::identify::config_t identify_config;
    identify_config.identify_time = 10;                    // 10 seconds
    identify_config.identify_type = chip::to_underlying(chip::app::Clusters::Identify::IdentifyTypeEnum::kActuator);
    esp_matter::cluster::identify::create(endpoint, &identify_config, 
                                           esp_matter::CLUSTER_FLAG_SERVER);
    ESP_LOGI(TAG, "Identify cluster created (10s timeout)");
    
    /* ---------------------------------------------------------------
       5. Create the Groups Server cluster
          Required for scene management support. Minimal config is fine.
       --------------------------------------------------------------- */
    esp_matter::cluster::common::config_t groups_config;
    esp_matter::cluster::groups::create(endpoint, &groups_config, 
                                         esp_matter::CLUSTER_FLAG_SERVER);
    ESP_LOGI(TAG, "Groups cluster created");
    
    /* ================================================================
       6. Create the Window Covering Server cluster — line by line
       ================================================================ */
    
    // 6a. Build the window covering configuration struct
    // Constructor sets end_product_type at construction time (it's const)
    esp_matter::cluster::window_covering::config_t wc_config(0);
    
    // 6b. Set type to Lift (0x00) — most appropriate for a gate/door
    // The spec only defines: 0=Lift, 1=Tilt, 2=Reserved
    wc_config.type = 0x00;
    
    // 6c. Configure feature flags:
    //     0x0001 = kLift          — basic lift capability
    //     0x0002 = kPositionAwareLift — target & current position attributes
    //     Together these two features are REQUIRED for HandleMovement() to fire.
    wc_config.feature_flags = 
        (uint32_t)chip::app::Clusters::WindowCovering::Feature::kLift
        | (uint32_t)chip::app::Clusters::WindowCovering::Feature::kPositionAwareLift;
    
    // 6d. Configure Position Aware Lift feature attributes (mandatory when that bit is set)
    // nullable<T> is a global template class, not in esp_matter namespace
    wc_config.features.position_aware_lift.target_position_lift_percent_100ths = 
        nullable<uint16_t>(0);
    wc_config.features.position_aware_lift.current_position_lift_percent_100ths = 
        nullable<uint16_t>(0);
    
    // 6e. Assign our delegate — this is the critical link to your HandleMovement()
    wc_config.delegate = &s_cover_delegate;
    
    // 6f. Create the cluster with the fully configured struct
    esp_matter::cluster::window_covering::create(endpoint, &wc_config, 
                                                  esp_matter::CLUSTER_FLAG_SERVER);
    ESP_LOGI(TAG, "Window Covering cluster created (feature flags = 0x%" PRIx32 ")", wc_config.feature_flags);
    
    /* ---------------------------------------------------------------
       7. Store the endpoint ID and handle for later use
       --------------------------------------------------------------- */
    switch_endpoint_id = esp_matter::endpoint::get_id(endpoint);
    s_cover_endpoint = endpoint;
    
    ESP_LOGI(TAG, "=== Manual Window Covering endpoint ready (ID=%d) ===", switch_endpoint_id);
    return ESP_OK;
}


/* ================================================================
   Remote Boolean State subscription (existing code, unchanged)
   ================================================================ */

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
        esp_matter_attr_val_t current_pos;
        esp_err_t err = esp_matter::attribute::get_val(
            switch_endpoint_id,
            chip::app::Clusters::WindowCovering::Id,
            chip::app::Clusters::WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id,
            &current_pos);
    
        if (err == ESP_OK && current_pos.type == ESP_MATTER_VAL_TYPE_UINT16) {
            uint16_t current = current_pos.val.u16;
            if (current != position) {
                ESP_LOGI(TAG, "Remote contact changed: %s -> updating to %d%%", 
                        is_open ? "OPEN" : "CLOSED", position);
                
                esp_matter_attr_val_t new_val = esp_matter_int16(position);
                esp_matter::attribute::set_val(switch_endpoint_id,
                    chip::app::Clusters::WindowCovering::Id,
                    chip::app::Clusters::WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id,
                    &new_val);
            }
        }
    }

    virtual void OnError(CHIP_ERROR aError) override {
        ESP_LOGI(TAG, "ReadClient Error: %s", ErrorStr(aError));
    }

    virtual void OnDone(chip::app::ReadClient * apReadClient) override {
        ESP_LOGI(TAG, "ReadClient Done");
    }
};

static BooleanStateReadCallback s_callback_handler;

void start_remote_subscription(chip::app::Clusters::Binding::TableEntry entry) {
    esp_matter::client::request_handle_t *req_handle = new esp_matter::client::request_handle_t();
    req_handle->type = esp_matter::client::SUBSCRIBE_ATTR;
    
    req_handle->attribute_path.mEndpointId = entry.remote;
    req_handle->attribute_path.mClusterId = chip::app::Clusters::BooleanState::Id;
    req_handle->attribute_path.mAttributeId = chip::app::Clusters::BooleanState::Attributes::StateValue::Id;
    
    req_handle->request_data = (void*)(uintptr_t)entry.remote;
    
    esp_err_t err = esp_matter::client::connect(nullptr, entry.fabricIndex, entry.nodeId, req_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate connection to node %llu", (unsigned long long)entry.nodeId);
        delete req_handle;
    }
}

static void connection_success_callback(esp_matter::client::peer_device_t *peer_device, esp_matter::client::request_handle_t *req_handle, void *priv_data) {
    if (!peer_device) {
        ESP_LOGE(TAG, "Peer device is null in connection success callback");
        return;
    }
    
    ESP_LOGI(TAG, "Connection established, starting subscription");
    
    chip::EndpointId remote_ep = (chip::EndpointId)(uintptr_t)req_handle->request_data;
    
    esp_err_t err = esp_matter::client::interaction::subscribe::send_request(
        peer_device, 
        &req_handle->attribute_path, 
        1,
        nullptr, 
        0,       
        1000,    
        5000,    
        true,    
        true,    
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
			
			if (entry.clusterId.has_value() && entry.clusterId.value() == chip::app::Clusters::BooleanState::Id) {
				ESP_LOGI(TAG, "Found BooleanState binding to node %llu ep %d", 
				         (unsigned long long)entry.nodeId, entry.remote);
                
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

	/* ================================================================
	   Create root node with standard callbacks
	   ================================================================ */
	esp_matter::node::config_t node_config;
	esp_matter::node_t *node = esp_matter::node::create(
        &node_config, 
        app_attribute_update_cb, 
        app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "failed to create Matter node"));
    
    /* ================================================================
       Manually build the Window Covering endpoint from scratch
       ================================================================ */
    err = create_manual_window_covering_endpoint(node);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "failed to create window covering endpoint"));

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
	err = esp_matter::start(app_event_cb, (intptr_t)0);
	ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "failed to start Matter, err:%d", err));
}
