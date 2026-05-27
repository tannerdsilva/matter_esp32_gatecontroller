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
static const char *TAG_BIND = "BIND_MGR";
static const char *TAG_EVENT = "APP_EVENT_CB";
static uint16_t event_stage = 0;
uint16_t switch_endpoint_id = 0;
static esp_matter::endpoint_t* s_cover_endpoint = nullptr;

using namespace esp_matter::client;

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


/*
================================================================
Remote Boolean State subscription (existing code, unchanged)
================================================================
*/

static bool s_contact_sensor_bound = false;
static chip::NodeId s_remote_node_id = 0;
static chip::EndpointId s_remote_endpoint = 0;

class BooleanStateReadCallback : public chip::app::ReadClient::Callback {
public:
    virtual void OnSubscriptionEstablished(chip::SubscriptionId aSubscriptionId) override {
        ESP_LOGI(TAG_BIND, "✅ Subscription established to remote contact sensor");
        s_contact_sensor_bound = true;
    }

    virtual void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath, 
                                 chip::TLV::TLVReader *aReader, 
                                 const chip::app::StatusIB &aStatus) override {
        if (aStatus.mStatus != chip::Protocols::InteractionModel::Status::Success) {
            ESP_LOGE(TAG_BIND, "⚠️ Subscription update failed, status: 0x%" PRIx32, 
                     static_cast<uint32_t>(aStatus.mStatus));
            return;
        }

        bool is_open = false;
        
        CHIP_ERROR err_read = aReader->Get(is_open);
		is_open = !is_open; 
        if (err_read != CHIP_NO_ERROR) {
            ESP_LOGE(TAG_BIND, "⚠️ Failed to read attribute value: %s (TLV type: %d)", 
                     ErrorStr(err_read), aReader->GetType());
            return;
        }

        uint16_t position = is_open ? 100 : 0;
        ESP_LOGI(TAG_BIND, "📡 Contact sensor changed: %s (pos=%d%%)", 
                 is_open ? "OPEN" : "CLOSED", position);
        
        // Update Window Covering data model
        esp_matter_attr_val_t current_pos;
        esp_err_t err = esp_matter::attribute::get_val(
            switch_endpoint_id,
            chip::app::Clusters::WindowCovering::Id,
            chip::app::Clusters::WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id,
            &current_pos);
    
        if (err == ESP_OK) {
			if (current_pos.type == ESP_MATTER_VAL_TYPE_UINT16) {
				uint16_t current = current_pos.val.u16;
				if (current != position) {
					esp_matter_attr_val_t new_val = esp_matter_int16(position);
					err = esp_matter::attribute::set_val(switch_endpoint_id,
						chip::app::Clusters::WindowCovering::Id,
						chip::app::Clusters::WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id,
						&new_val);
					
					if (err == ESP_OK) {
						ESP_LOGI(TAG_BIND, "✅ Window covering updated to %d%%", position);
					} else {
						ESP_LOGE(TAG_BIND, "❌ Failed to update window covering: %d", err);
					}
				} else {
					ESP_LOGI(TAG_BIND, "ℹ️ Position unchanged (%d%%), skipping update", position);
				}
			} else {
				ESP_LOGE(TAG_BIND, "❌ Unexpected attribute type: %d", current_pos.type);
			}
		} else {
			ESP_LOGE(TAG_BIND, "❌ Failed to read current position: %d", err);
		}
    }


    virtual void OnError(CHIP_ERROR aError) override {
        ESP_LOGE(TAG_BIND, "❌ Subscription error: %s", ErrorStr(aError));
        s_contact_sensor_bound = false;
    }

    virtual void OnDone(chip::app::ReadClient * apReadClient) override {
        ESP_LOGI(TAG_BIND, "ℹ️ Subscription ended");
        s_contact_sensor_bound = false;
    }
};

static BooleanStateReadCallback s_callback_handler;

static const char *TAG_CLIENT = "CLIENT";

static void app_client_callback(esp_matter::client::peer_device_t *peer_device, 
                                esp_matter::client::request_handle_t *req_handle, 
                                void *priv_data) {
	ESP_LOGI(TAG_CLIENT, "✅🔥 app_client_callback FIRED! Starting subscription...");
	if (!peer_device || !req_handle) {
        ESP_LOGE(TAG_CLIENT, "❌ Peer or request handle is null");
        return;
    }
    
    ESP_LOGI(TAG_CLIENT, "✅ Secure session ready, starting subscription");
    
    uint16_t min_interval = 1;   // seconds
    uint16_t max_interval = 5;   // seconds
    bool keep_subscription = true;
    bool auto_resubscribe = true;

    esp_err_t err = esp_matter::client::interaction::subscribe::send_request(
        peer_device, 
        &req_handle->attribute_path, 
        1, // attribute count
        nullptr, 
        0, // event count
        min_interval, 
        max_interval, 
        keep_subscription, 
        auto_resubscribe, 
        s_callback_handler
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG_CLIENT, "❌ Failed to send subscription request: %d", err);
    }
}

// Group callback (required by API signature even if unused)
static void app_client_group_callback(uint8_t fabric_index, 
                                      esp_matter::client::request_handle_t *req_handle, 
                                      void *priv_data) {
    (void)fabric_index; 
    (void)req_handle; 
    (void)priv_data;
}

/* ------------------------------------------------------------------
 * Helper to revive subscriptions for existing bindings on startup
 * ------------------------------------------------------------------ */
static void revive_existing_bindings(void) {
    ESP_LOGI(TAG_BIND, "🔄 Reviving existing binding subscriptions from NVS...");
    
    // Get the table instance (this is already loaded from NVS by this point)
    chip::app::Clusters::Binding::Table &table = chip::app::Clusters::Binding::Table::GetInstance();
    size_t count = table.Size();
    ESP_LOGI(TAG_BIND, "🔄 There are %i entries stored in the nvs binding table storage.", count);
    for (size_t i = 0; i < count; i++) {
        chip::app::Clusters::Binding::TableEntry entry = table.GetAt(i);
        
        // Only process unicast bindings
        if (entry.type != chip::app::Clusters::Binding::MATTER_UNICAST_BINDING) continue;
        
        uint32_t target_cluster = entry.clusterId.has_value() ? entry.clusterId.value() : 0;
        
        // Match the same criteria as your kBindingsChangedViaCluster handler
        bool is_contact_binding = (target_cluster == chip::app::Clusters::BooleanState::Id ||
                                   target_cluster == chip::app::Clusters::OnOff::Id ||
                                   !entry.clusterId.has_value()); // Blank cluster ID fallback
        
        if (is_contact_binding) {
            ESP_LOGI(TAG_BIND, "   Reviving binding: Node=0x%llX, Ep=%d, Cluster=0x%" PRIx32, 
                     (unsigned long long)entry.nodeId, entry.remote, target_cluster);
            
            // Allocate request handle to register as a pending subscription
            esp_matter::client::request_handle_t *req_handle = chip::Platform::New<esp_matter::client::request_handle_t>();
            if (!req_handle) {
                ESP_LOGW(TAG_BIND, "   ⚠️ Failed to allocate request handle");
                continue;
            }
            
            req_handle->type = esp_matter::client::SUBSCRIBE_ATTR;
            req_handle->attribute_path.mEndpointId = entry.remote;
            req_handle->attribute_path.mClusterId = chip::app::Clusters::BooleanState::Id;
            req_handle->attribute_path.mAttributeId = 0x0000; // StateValue
            req_handle->request_data = nullptr;
            
            // Trigger the exact same callback chain that a new binding would use.
            // The esp-matter Binding Manager will handle the CASE session handshake.
            esp_err_t err = esp_matter::client::cluster_update(0, req_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG_BIND, "   ❌ Failed to trigger cluster_update: %d", err);
                chip::Platform::Delete(req_handle);
            }
        }
    }
}

static esp_err_t start_remote_subscription(chip::app::Clusters::Binding::TableEntry entry) {
    // 1. Validate Node and Endpoint
    if (!entry.remote || !entry.nodeId) {
        ESP_LOGE(TAG_BIND, "❌ Invalid binding entry: ep=%d node=0x%llX", 
                 entry.remote, (unsigned long long)entry.nodeId);
        return ESP_FAIL;
    }

    // 2. HARD CODE the fallback values to bypass Controller Constraint Errors
    // Cluster 0x45 is BooleanState Server (69 decimal)
    // Attribute 0x0000 is StateValue
    uint32_t target_cluster_id = 0x0045; 
    uint32_t target_attr_id = 0x0000;    
    
    ESP_LOGI(TAG_BIND, "🔗 Force-connecting to Node=0x%llX, Ep=%d, Cluster=0x%" PRIx32, 
             (unsigned long long)entry.nodeId, entry.remote, target_cluster_id);

    // 3. Allocate and CLEAR the request handle
    esp_matter::client::request_handle_t *req_handle = new esp_matter::client::request_handle_t();
    if (!req_handle) {
        ESP_LOGE(TAG_BIND, "❌ Failed to allocate request handle");
        return ESP_FAIL;
    }

    // 4. Build the attribute path explicitly
    req_handle->type = esp_matter::client::SUBSCRIBE_ATTR;
    req_handle->attribute_path.mEndpointId = entry.remote;
    req_handle->attribute_path.mClusterId = target_cluster_id;
    req_handle->attribute_path.mAttributeId = target_attr_id;
    
    req_handle->request_data = (void*)(uintptr_t)entry.remote;

    // 5. Log the exact values being passed to the stack for debugging
    ESP_LOGI(TAG_BIND, "📤 Sending: Cluster=0x%" PRIx32 ", Attribute=0x%" PRIx32 " (Type=%d)", 
             req_handle->attribute_path.mClusterId,
             req_handle->attribute_path.mAttributeId,
             req_handle->type);

    // 6. Initiate the connection
    esp_err_t err = esp_matter::client::connect(nullptr, entry.fabricIndex, entry.nodeId, req_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BIND, "❌ client::connect failed: %d (0x%02X)", err, err);
        delete req_handle;
        return err;
    }
    
    s_remote_node_id = entry.nodeId;
    s_remote_endpoint = entry.remote;
    
    ESP_LOGI(TAG_BIND, "✅ Connection initiated successfully");
    return ESP_OK;
}

static void connection_success_callback(esp_matter::client::peer_device_t *peer_device, 
                                        esp_matter::client::request_handle_t *req_handle, 
                                        void *priv_data) {
    if (!peer_device || !req_handle) {
        ESP_LOGE(TAG_BIND, "❌ Peer or request handle is null");
        return;
    }
    
    ESP_LOGI(TAG_BIND, "✅ Connection established, starting subscription");
    
    chip::EndpointId remote_ep = (chip::EndpointId)(uintptr_t)req_handle->request_data;
    
    esp_err_t err = esp_matter::client::interaction::subscribe::send_request(
        peer_device, 
        &req_handle->attribute_path, 
        1,
        nullptr, 
        0,       
        1000,    // min interval: 1 second
        5000,    // max interval: 5 seconds
        true,    // keep_subscription
        true,    // auto_resubscribe
        s_callback_handler
    );
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BIND, "❌ Failed to start subscription: %d", err);
    } else {
        ESP_LOGI(TAG_BIND, "✅ Subscription started to remote endpoint %d", remote_ep);
    }
}

/* ================================================================
Event callback with improved binding handling
================================================================ */
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg) {
	switch (event->Type) {
	case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
		ESP_LOGI(TAG_EVENT, "🌐 IP Address Changed");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
		ESP_LOGI(TAG_EVENT, "✅ Commissioning complete");
		event_stage = 2;
		led_indicator_set_color(&led_indicator_subsystem, 0, 255, 0); // green
		
		break;

	case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
		ESP_LOGI(TAG_EVENT, "❌ Commissioning failed, fail safe timer expired");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
		ESP_LOGI(TAG_EVENT, "🔧 Commissioning session started");
		event_stage = 1;
		led_indicator_set_color(&led_indicator_subsystem, 0, 16, 16); // dim blue
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
		ESP_LOGI(TAG_EVENT, "🔒 Commissioning session stopped");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
		ESP_LOGI(TAG_EVENT, "🪟 Commissioning window opened");
		event_stage = 0;
		led_indicator_set_color(&led_indicator_subsystem, 0, 0, 128); // blue
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
		ESP_LOGI(TAG_EVENT, "🔒 Commissioning window closed");
		if (event_stage == 0) {
			led_indicator_set_color(&led_indicator_subsystem, 255, 0, 0); // red
		}
		break;

	case chip::DeviceLayer::DeviceEventType::kServerReady:
		ESP_LOGI(TAG_EVENT, "💡 Matter stack initialized, reviving bindings...");
		chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t arg) {
			(void)arg;
			// Give the esp-matter framework ~500ms to finish its internal NVS load
			vTaskDelay(pdMS_TO_TICKS(1000));
			ESP_LOGI(TAG_EVENT, "Initializing binding manager and reviving existing bindings...");
			revive_existing_bindings();
		});
		break;
		
	case chip::DeviceLayer::DeviceEventType::kBindingsChangedViaCluster: {
		ESP_LOGI(TAG_EVENT, "🔄 Bindings changed via cluster");
		
		chip::app::Clusters::Binding::Table &table = chip::app::Clusters::Binding::Table::GetInstance();
			for (auto iter = table.begin(); iter != table.end(); ++iter) {
				if (iter->fabricIndex != event->BindingsChanged.fabricIndex) continue;
				if (iter->type != chip::app::Clusters::Binding::MATTER_UNICAST_BINDING) continue;
				
				auto entry = *iter;
				uint32_t target_cluster = entry.clusterId.has_value() ? entry.clusterId.value() : 0;
				
				// Check for contact sensor clusters
				// Note: If no cluster was specified (0), we accept it as a "bind to all" fallback
				bool is_contact_binding = false;
				if (target_cluster == chip::app::Clusters::BooleanState::Id) {
					is_contact_binding = true;
				} else if (target_cluster == chip::app::Clusters::OnOff::Id) {
					is_contact_binding = true;
				} else if (!entry.clusterId.has_value()) {
					// No cluster specified = accept as fallback for contact sensor
					is_contact_binding = true;
				}
				
				if (is_contact_binding) {
					ESP_LOGI(TAG_EVENT, "✅ Found contact binding. Node=0x%llX, Ep=%d, Cluster=0x%" PRIx32, 
							(unsigned long long)entry.nodeId, entry.remote, target_cluster);
					
					// Create a request handle to trigger the callback
					esp_matter::client::request_handle_t *req_handle = 
						chip::Platform::New<esp_matter::client::request_handle_t>();
					if (!req_handle) {
						ESP_LOGE(TAG_EVENT, "❌ Failed to allocate request handle");
						break;
					}
					
					// Build the attribute path for BooleanState Server (0x45) attribute StateValue (0x0000)
					req_handle->type = esp_matter::client::SUBSCRIBE_ATTR;
					req_handle->attribute_path.mEndpointId = entry.remote;
					req_handle->attribute_path.mClusterId = chip::app::Clusters::BooleanState::Id;
					req_handle->attribute_path.mAttributeId = 0x0000; // StateValue
					req_handle->request_data = nullptr;
					
					// Notify the binding manager that the bound cluster has changed
					// This will trigger our app_client_callback when session is ready
					esp_err_t err = esp_matter::client::cluster_update(
						0, 
						req_handle
					);
					if (err != ESP_OK) {
						ESP_LOGE(TAG_EVENT, "❌ Failed to notify bound cluster changed: %d", err);
						chip::Platform::Delete(req_handle);
					}
				}
			}
			break;
		}
		break;
    
	case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
		ESP_LOGI(TAG_EVENT, "🗑️ Fabric removed, clearing bindings");
		s_contact_sensor_bound = false;
		break;
	case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG_EVENT, "BLE deinitialized and memory reclaimed");
        break;
    default:
        ESP_LOGI(TAG_EVENT, "ℹ️ Unhandled event %d", static_cast<int>(event->Type));
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

static esp_err_t init_binding_cluster(esp_matter::node_t *node) {
	// Get root endpoint (always endpoint ID 0)
	esp_matter::endpoint_t *root_endpoint = esp_matter::endpoint::get_first(node);
	if (!root_endpoint) {
		ESP_LOGE(TAG, "❌ Failed to get root endpoint");
		return ESP_FAIL;
	}
	
	// Create Binding Server cluster on root endpoint
	esp_matter::cluster::common::config_t binding_config;
	esp_matter::cluster::binding::create(root_endpoint, 
										&binding_config, 
										esp_matter::CLUSTER_FLAG_SERVER);
	ESP_LOGI(TAG, "✅ Binding cluster added to root endpoint (ID=0)");
	
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
	
	err = init_binding_cluster(node);
	ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "failed to initialize binding cluster"));
	
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

	// assign the connection success callback for client interactions (e.g. subscriptions)
	err = esp_matter::client::set_request_callback(app_client_callback, app_client_group_callback, nullptr);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to set client callback");
	}

	esp_matter::client::binding_manager_init();
	err = esp_matter::start(app_event_cb, (intptr_t)0);
	ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "failed to start Matter, err:%d", err));
}
