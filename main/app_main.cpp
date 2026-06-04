#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#include <esp_matter.h>
#include <esp_matter_client.h>
#include <app/clusters/bindings/binding-table.h>
#include "debounce.h"

#include <app/clusters/boolean-state-server/boolean-state-cluster.h>
#include <app/clusters/window-covering-server/window-covering-server.h>
#include <app/data-model/Nullable.h>

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
#include "subscription_manager.h"
#include "close_duration.h"

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
static const char *TAG_HANDLER = "EVENT_HANDLER";

static uint16_t event_stage = 0;
uint16_t wc_endpoint_id = 99;
static esp_matter::endpoint_t* s_cover_endpoint = nullptr;

using namespace esp_matter::client;

static chip::app::Clusters::WindowCovering::MyWindowCoveringManager s_cover_delegate;
uint32_t s_closing_start_ms = 0;
bool s_is_closing_active = false;

uint32_t s_motor_start_ms = 0;
bool s_motor_active = false;
uint8_t s_last_known_position = 0;

// mutex lock for all resources going forward
static SemaphoreHandle_t data_model_mux = NULL;

// data model
esp_matter::node_t *data_node = nullptr;
esp_matter::endpoint_t *root_endpoint  = nullptr;
esp_matter::cluster_t *root_binding_cluster = nullptr;
// window covering endpoint
esp_matter::endpoint_t *wc_endpoint = nullptr;
esp_matter::cluster_t *wc_identify_cluster = nullptr;
esp_matter::cluster_t *wc_groups_cluster = nullptr;
esp_matter::cluster_t *wc_descriptor_cluster = nullptr;
// window covering attributes
esp_matter::cluster_t *wc_cluster_scratchbuilt = nullptr;
esp_matter::attribute_t *wc_cluster_type_attribute = nullptr;
esp_matter::attribute_t *wc_cluster_configstatus_attribute = nullptr;
esp_matter::attribute_t *wc_cluster_operationalstatus_attribute = nullptr;
esp_matter::attribute_t *wc_cluster_endproducttype_attribute = nullptr;
esp_matter::attribute_t *wc_cluster_mode = nullptr;
esp_matter::attribute_t *lift_percentage = nullptr;
esp_matter::attribute_t *lift_percentage_n100_target = nullptr;
esp_matter::attribute_t *lift_percentage_n100_current = nullptr;

// state tracking

bool bound_sensor_last_known_position_isknown = false;
bool bound_sensor_last_known_position_isclosed = false;
uint32_t closing_start_ms = 0;

bool animation_timer_engaged = false;

extern bool animation_timer_engaged;

static esp_err_t window_covering_command_openorclose_handler(const chip::app::ConcreteCommandPath &command_path, chip::TLV::TLVReader &tlv_data, void *opaque_ptr) {
	(void)tlv_data;
	(void)opaque_ptr;
	
	while (tlv_data.Next() == CHIP_NO_ERROR) {
		ESP_LOGW(TAG_HANDLER, "SKIPPING TVL_DATA");
		tlv_data.Skip();
	}
	
	// obtain the data model lock
	if (xSemaphoreTake(data_model_mux, pdMS_TO_TICKS(100)) != pdTRUE) {
		ESP_LOGE(TAG_BIND, "FAILED TO ACQUIRE DATA MODEL LOCK");
		return ESP_FAIL;
	}
	
	esp_err_t err_ret = ESP_OK;
	
	if (debounce_check() == false) {
		ESP_LOGW(TAG_HANDLER, "DEBOUNCER BLOCKING COMMAND. THE DEBOUNCER WILL BE RESET.");
		debounce_reset();
		err_ret = ESP_FAIL;
	} else {
		ESP_LOGI(TAG_HANDLER, "DEBOUNCER NO BLOCK");
	}

	motor_relay_toggle_async();

	if (bound_sensor_last_known_position_isknown == true) {
		// state must be known before we "animate" the movement in the data model.
		if (bound_sensor_last_known_position_isclosed == true) {
			// set to 'opening' state
			chip::app::Clusters::WindowCovering::OperationalStateSet(wc_endpoint_id, chip::app::Clusters::WindowCovering::OperationalStatus::kLift, chip::app::Clusters::WindowCovering::OperationalState::MovingUpOrOpen);
		} else {
			// set to 'closing' state
			chip::app::Clusters::WindowCovering::OperationalStateSet(wc_endpoint_id, chip::app::Clusters::WindowCovering::OperationalStatus::kLift, chip::app::Clusters::WindowCovering::OperationalState::MovingDownOrClose);
			closing_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
		}
		ESP_LOGI(TAG_HANDLER, "window covering operational state modified");
	} else {
		// take no action when the position is unknown
	}
	
	// release the data model lock
	xSemaphoreGive(data_model_mux);

	return err_ret;
}

static esp_err_t window_covering_command_handler(const chip::app::ConcreteCommandPath &command_path, chip::TLV::TLVReader &tlv_data, void *opaque_ptr) {
	(void)tlv_data;
	(void)opaque_ptr;
	
	while (tlv_data.Next() == CHIP_NO_ERROR) {
		ESP_LOGI(TAG, "SKIPPING TVL_DATA");
		tlv_data.Skip();
	}
	
	switch (command_path.mCommandId) {
		case chip::app::Clusters::WindowCovering::Commands::UpOrOpen::Id:
			ESP_LOGI(TAG, "⬆️ Up/Open Command Received");
			s_motor_active = true;
			s_is_closing_active = false;
			motor_relay_toggle_async();
			break;
	
		case chip::app::Clusters::WindowCovering::Commands::DownOrClose::Id:
			ESP_LOGI(TAG, "⬇️ Down/Close Command Received");
			s_motor_active = true;
			s_is_closing_active = true;
			s_closing_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
			motor_relay_toggle_async();
			break;
	
		case chip::app::Clusters::WindowCovering::Commands::StopMotion::Id:
			ESP_LOGI(TAG, "🛑 Stop Motion Command Received");
			s_motor_active = false;
			motor_relay_toggle_async(); // Your toggle likely stops the relay
			break;
	
		default:
			ESP_LOGW(TAG, "❓ Unknown Window Covering Command");
			return ESP_FAIL;
	}
	return ESP_OK;
}

static const char *TAG_ENDPOINT_INIT = "ENDPOINT_INIT";
static esp_err_t create_manual_window_covering_endpoint(esp_matter::node_t *node) {
	esp_err_t err = ESP_OK;
	
	// endpoint creation.
	wc_endpoint = esp_matter::endpoint::create(node, 0, nullptr);
	if (!wc_endpoint) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create new endpoint in node.");
		return ESP_FAIL;
	} else {
		wc_endpoint_id = esp_matter::endpoint::get_id(wc_endpoint);
		ESP_LOGI(TAG_ENDPOINT_INIT, "endpoint created (internal ID: %d)", esp_matter::endpoint::get_id(wc_endpoint));
	}
	
	// set the device type of the new endpoint.
	err = esp_matter::endpoint::add_device_type(wc_endpoint, ESP_MATTER_WINDOW_COVERING_DEVICE_TYPE_ID, ESP_MATTER_WINDOW_COVERING_DEVICE_TYPE_VERSION);
	if (err != ESP_OK) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to add device type: %d", err);
		return err;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "device type 0x%04X v%d added", ESP_MATTER_WINDOW_COVERING_DEVICE_TYPE_ID, ESP_MATTER_WINDOW_COVERING_DEVICE_TYPE_VERSION);
	}
	
	// create the required descriptor cluster.
	esp_matter::cluster::descriptor::config_t descriptor_config;
	wc_descriptor_cluster = esp_matter::cluster::descriptor::create(wc_endpoint, &descriptor_config, esp_matter::CLUSTER_FLAG_SERVER);
	if (!wc_descriptor_cluster) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create descriptor cluster");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "descriptor cluster created");
	}
	
	// create the required identify cluster.
	esp_matter::cluster::identify::config_t identify_config;
	identify_config.identify_time = 10;		// 10 seconds
	identify_config.identify_type = chip::to_underlying(chip::app::Clusters::Identify::IdentifyTypeEnum::kActuator);
	wc_identify_cluster = esp_matter::cluster::identify::create(wc_endpoint, &identify_config, esp_matter::CLUSTER_FLAG_SERVER);
	if (!wc_identify_cluster) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create identify cluster");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "identify cluster created (10s timeout)");
	}
	
	// create the required groups cluster.
	esp_matter::cluster::common::config_t groups_config;
	wc_groups_cluster = esp_matter::cluster::groups::create(wc_endpoint, &groups_config, esp_matter::CLUSTER_FLAG_SERVER);
	if (!wc_groups_cluster) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create groups cluster");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "groups cluster created");
	}
	
	// build the window covering cluster.
	wc_cluster_scratchbuilt = esp_matter::cluster::create(wc_endpoint, chip::app::Clusters::WindowCovering::Id, esp_matter::CLUSTER_FLAG_SERVER);
	if (!wc_cluster_scratchbuilt) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create window covering cluster");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "window covering cluster base created");
	}
	wc_cluster_type_attribute = esp_matter::cluster::window_covering::attribute::create_type(wc_cluster_scratchbuilt, 0);
	if (!wc_cluster_type_attribute) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create type attribute");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "type attribute created");
	}
	wc_cluster_configstatus_attribute = esp_matter::cluster::window_covering::attribute::create_config_status(wc_cluster_scratchbuilt, 0);
	if (!wc_cluster_configstatus_attribute) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create config status attribute");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "config status attribute created");
	}

	// operational status attribute
	wc_cluster_operationalstatus_attribute = esp_matter::cluster::window_covering::attribute::create_operational_status(wc_cluster_scratchbuilt, 0);
	if (!wc_cluster_operationalstatus_attribute) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create operational status attribute");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "operational status attribute created");
	}

	// end product type attribute
	wc_cluster_endproducttype_attribute = esp_matter::cluster::window_covering::attribute::create_end_product_type(wc_cluster_scratchbuilt, 0);
	if (!wc_cluster_endproducttype_attribute) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create end product type attribute");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "end product type attribute created");
	}

	// mode attribute
	wc_cluster_mode = esp_matter::cluster::window_covering::attribute::create_mode(wc_cluster_scratchbuilt, 0);
	if (!wc_cluster_mode) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create mode attribute");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "mode attribute created");
	}

	// lift percentage attribute
	lift_percentage = esp_matter::cluster::window_covering::attribute::create_current_position_lift_percentage(wc_cluster_scratchbuilt, 0);
	if (!lift_percentage) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create lift percentage attribute");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "lift percentage attribute created");
	}
	// lift percentage n100 target
	lift_percentage_n100_target = esp_matter::cluster::window_covering::attribute::create_target_position_lift_percent_100ths(wc_cluster_scratchbuilt, 0);
	if (!lift_percentage_n100_target) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create lift percentage n100 (target) attribute");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "lift percentage n100 (target) attribute created");
	}
	// lift percentage n100 current
	lift_percentage_n100_current = esp_matter::cluster::window_covering::attribute::create_current_position_lift_percent_100ths(wc_cluster_scratchbuilt, 0);
	if (!lift_percentage_n100_current) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create lift percentage n100 (current) attribute");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "lift percentage n100 (current) attribute created");
	}
	
	// cluster revision attribute
	esp_matter::cluster::global::attribute::create_cluster_revision(wc_cluster_scratchbuilt, 5);

	// feature map
	esp_matter::cluster::global::attribute::create_feature_map(wc_cluster_scratchbuilt, (uint32_t)chip::app::Clusters::WindowCovering::Feature::kLift);

	// up/open command registration
	esp_matter::command_t *wc_cluster_upopen_command = esp_matter::command::create(wc_cluster_scratchbuilt, (uint32_t)chip::app::Clusters::WindowCovering::Commands::UpOrOpen::Id, esp_matter::COMMAND_FLAG_ACCEPTED | esp_matter::COMMAND_FLAG_CUSTOM, window_covering_command_openorclose_handler);
	if (!wc_cluster_upopen_command) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create Up/Open command");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "up/open command created");
	}

	// stop command registration
	esp_matter::command_t *wc_cluster_stop_command = esp_matter::command::create(wc_cluster_scratchbuilt, (uint32_t)chip::app::Clusters::WindowCovering::Commands::StopMotion::Id, esp_matter::COMMAND_FLAG_ACCEPTED | esp_matter::COMMAND_FLAG_CUSTOM, window_covering_command_handler);
	if (!wc_cluster_stop_command) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create Stop command");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "stop command created");
	}

	// down/close command
	esp_matter::command_t *wc_cluster_downclose_command = esp_matter::command::create(wc_cluster_scratchbuilt, (uint32_t)chip::app::Clusters::WindowCovering::Commands::DownOrClose::Id, esp_matter::COMMAND_FLAG_ACCEPTED | esp_matter::COMMAND_FLAG_CUSTOM, window_covering_command_openorclose_handler);
	if (!wc_cluster_downclose_command) {
		ESP_LOGE(TAG_ENDPOINT_INIT, "failed to create Down/Close command");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG_ENDPOINT_INIT, "down/close command created");
	}
	
	ESP_LOGI(TAG_ENDPOINT_INIT, "window covering cluster fully constructed");
	return ESP_OK;
}


void set_window_covering_to_unknown(uint16_t endpoint_id) {
	nullable<uint8_t> nu8 = {};
	nullable<uint16_t> nu16 = {};
	esp_matter_attr_val_t attrval8 = esp_matter_nullable_uint8(nu8);
	esp_matter_attr_val_t attrval16 = esp_matter_nullable_uint16(nu16);
	
	uint16_t use_wc_endpoint = esp_matter::endpoint::get_id(wc_endpoint);
	uint32_t use_wc_clusterid = esp_matter::cluster::get_id(wc_cluster_scratchbuilt);
	uint32_t use_wc_attributeid = esp_matter::attribute::get_id(lift_percentage);
	esp_err_t err = esp_matter::attribute::report(use_wc_endpoint, use_wc_clusterid, use_wc_attributeid, &attrval8);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "there was an error setting unknown lift percentage (uint8_t) to null.");
	}
	use_wc_attributeid = esp_matter::attribute::get_id(lift_percentage_n100_target);
	err = esp_matter::attribute::report(use_wc_endpoint, use_wc_clusterid, use_wc_attributeid, &attrval16);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "there was an error setting unknown lift percentage n100 target (uint16_t) to null.");
	}
	use_wc_attributeid = esp_matter::attribute::get_id(lift_percentage_n100_current);
	err = esp_matter::attribute::report(use_wc_endpoint, use_wc_clusterid, use_wc_attributeid, &attrval16);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "there was an error setting unknown lift percentage n100 current (uint16_t) to null.");
	}
}

static esp_timer_handle_t s_retry_timer = NULL;

class BooleanStateReadCallback : public chip::app::ReadClient::Callback {
public:
	virtual void OnSubscriptionEstablished(chip::SubscriptionId aSubscriptionId) override {
		ESP_LOGI(TAG_BIND, "✅ Subscription established to remote contact sensor");
		subscription_manager_on_subscription_established();
		led_indicator_set_color(&led_indicator_subsystem, 0, 32, 0);
	}

	virtual void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath, chip::TLV::TLVReader *aReader, const chip::app::StatusIB &aStatus) override {
		if (aStatus.mStatus != chip::Protocols::InteractionModel::Status::Success) {
			ESP_LOGE(TAG_BIND, "⚠️ subscription update failed, status: 0x%" PRIx32, static_cast<uint32_t>(aStatus.mStatus));
			subscription_manager_on_subscription_failed();
			led_indicator_set_color(&led_indicator_subsystem, 255, 128, 0);
			return;
		}

		bool is_closed = false;
		
		CHIP_ERROR err_read = aReader->Get(is_closed);
		if (err_read != CHIP_NO_ERROR) {
			ESP_LOGE(TAG_BIND, "⚠️ failed to read attribute value: %s (TLV type: %d)", ErrorStr(err_read), aReader->GetType());
			return;
		}
		
		if (xSemaphoreTake(data_model_mux, pdMS_TO_TICKS(100)) != pdTRUE) {
			ESP_LOGE(TAG_BIND, "FAILED TO ACQUIRE DATA MODEL LOCK");
			return;
		}
		
		// update state variables
		if (bound_sensor_last_known_position_isknown == false && is_closed == true) {
			bound_sensor_last_known_position_isknown = true;
			bound_sensor_last_known_position_isclosed = is_closed;
			ESP_LOGI(TAG_BIND, "CLOSURE POSITION IS NOW KNOWN");
		} else if (bound_sensor_last_known_position_isknown == true && bound_sensor_last_known_position_isclosed != is_closed) {
			bound_sensor_last_known_position_isclosed = is_closed;
			ESP_LOGI(TAG_BIND, "internal state variables updated with new position.");
		}
		
		if (is_closed == true) {
			// closing specific logic here
			if (closing_start_ms != 0) {
				uint32_t close_end_ms = (uint32_t)(esp_timer_get_time() / 1000);
				if (close_duration_add(closing_start_ms, close_end_ms)) {
					ESP_LOGI(TAG_BIND, "✅ close duration recorded: %lu ms", close_end_ms - s_closing_start_ms);
				} else {
					ESP_LOGW(TAG_BIND, "⚠️ close duration outside bounds (5-60s), ignored");
				}
			}
// 			if (animation_timer_engaged == true) {
			chip::app::Clusters::WindowCovering::OperationalStateSet(wc_endpoint_id, chip::app::Clusters::WindowCovering::OperationalStatus::kLift, chip::app::Clusters::WindowCovering::OperationalState::Stall);
// 			}
			chip::app::Clusters::WindowCovering::NPercent100ths pos;
			pos.SetNonNull(WC_PERCENT100THS_MAX_CLOSED);
			chip::app::Clusters::WindowCovering::LiftPositionSet(wc_endpoint_id, pos);
			_datamodel_animation_timer_cancel();
		} else {
			// opening specific logic here
			chip::app::Clusters::WindowCovering::OperationalStateSet(wc_endpoint_id, chip::app::Clusters::WindowCovering::OperationalStatus::kLift, chip::app::Clusters::WindowCovering::OperationalState::Stall);
			chip::app::Clusters::WindowCovering::NPercent100ths pos;
			pos.SetNonNull(WC_PERCENT100THS_MIN_OPEN);
			chip::app::Clusters::WindowCovering::LiftPositionSet(wc_endpoint_id, pos);
			_datamodel_animation_timer_cancel();
		}
		
		xSemaphoreGive(data_model_mux);

		// BooleanState StateValue=true means the contact is Closed (magnet engaged)
		uint8_t percentage = is_closed ? 100 : 0; 
		ESP_LOGI(TAG_BIND, "📡 contact sensor changed position: %s (pos=%d%%)", is_closed ? "CLOSED" : "OPEN", percentage);

		// ✅ Update the "Basic Lift" percentage attribute (0% to 100%)
		percentage = is_closed ? 100 : 0;
	
		// 1. Update current position
		chip::app::Clusters::WindowCovering::NPercent100ths pos;
		pos.SetNonNull(is_closed ? WC_PERCENT100THS_MAX_CLOSED : WC_PERCENT100THS_MIN_OPEN);
		chip::app::Clusters::WindowCovering::LiftPositionSet(wc_endpoint_id, pos);
		
		// 3. Update last known position
		s_last_known_position = percentage;

		// ✅ Update operational state to Stall (not moving)
		chip::app::Clusters::WindowCovering::OperationalStateSet(wc_endpoint_id, chip::app::Clusters::WindowCovering::OperationalStatus::kLift, chip::app::Clusters::WindowCovering::OperationalState::Stall);
		ESP_LOGI(TAG_BIND, "✅ Window covering updated to %d%%", percentage);
	}

	virtual void OnError(CHIP_ERROR aError) override {
		ESP_LOGE(TAG_BIND, "❌ Subscription error: %s", ErrorStr(aError));
		subscription_manager_on_subscription_failed();
		led_indicator_set_color(&led_indicator_subsystem, 255, 128, 0);
	}

	virtual void OnDone(chip::app::ReadClient * apReadClient) override {
		ESP_LOGI(TAG_BIND, "ℹ️ Subscription ended");
		subscription_manager_on_subscription_failed();
		led_indicator_set_color(&led_indicator_subsystem, 255, 128, 0);
	}
};

static BooleanStateReadCallback s_callback_handler;

static const char *TAG_CLIENT = "CLIENT";

static void app_client_callback(esp_matter::client::peer_device_t *peer_device, esp_matter::client::request_handle_t *req_handle, void *priv_data) {
	ESP_LOGI(TAG_CLIENT, "✅🔥 app_client_callback FIRED! Starting subscription...");
	if (!peer_device || !req_handle) {
        ESP_LOGE(TAG_CLIENT, "❌ Peer or request handle is null");
        return;
    }
    
    uint16_t min_interval = 1;
    uint16_t max_interval = 5;
    bool keep_subscription = true;
    bool auto_resubscribe = false;

    esp_err_t err = esp_matter::client::interaction::subscribe::send_request(
        peer_device, 
        &req_handle->attribute_path, 
        1,
        nullptr, 
        0,
        min_interval, 
        max_interval, 
        keep_subscription, 
        auto_resubscribe, 
        s_callback_handler
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG_CLIENT, "❌ Failed to send subscription request: %d", err);
        subscription_manager_on_subscription_failed();
    } else {
		ESP_LOGI(TAG_CLIENT, "📤 Subscribe request sent");
    }
}

// Group callback (required by API signature even if unused)
static void app_client_group_callback(uint8_t fabric_index, esp_matter::client::request_handle_t *req_handle, void *priv_data) {
	(void)fabric_index; 
	(void)req_handle; 
	(void)priv_data;
}

static void start_contact_subscription(chip::NodeId node_id, chip::EndpointId endpoint_id, uint32_t cluster_id) {
	subscription_manager_start_or_revive(node_id, endpoint_id, cluster_id);
}

/* ------------------------------------------------------------------
 * Helper to revive subscriptions for existing bindings on startup
 * ------------------------------------------------------------------ */
static void revive_existing_bindings(void) {
    ESP_LOGI(TAG_BIND, "🔄 Reviving existing binding subscriptions from NVS...");
    
    chip::app::Clusters::Binding::Table &table = chip::app::Clusters::Binding::Table::GetInstance();
    size_t count = table.Size();
    ESP_LOGI(TAG_BIND, "🔄 There are %i entries stored in the nvs binding table storage.", count);
    
    for (size_t i = 0; i < count; i++) {
        chip::app::Clusters::Binding::TableEntry entry = table.GetAt(i);
        
        if (entry.type != chip::app::Clusters::Binding::MATTER_UNICAST_BINDING) continue;
        
        uint32_t target_cluster = entry.clusterId.has_value() ? entry.clusterId.value() : 0;
        
        bool is_contact_binding = (target_cluster == chip::app::Clusters::BooleanState::Id || target_cluster == chip::app::Clusters::OnOff::Id || !entry.clusterId.has_value());
        if (is_contact_binding) {
            ESP_LOGI(TAG_BIND, "   Reviving binding: Node=0x%llX, Ep=%d, Cluster=0x%" PRIx32, (unsigned long long)entry.nodeId, entry.remote, target_cluster);
            start_contact_subscription(entry.nodeId, entry.remote, target_cluster);
        }
    }
}

static void connection_success_callback(esp_matter::client::peer_device_t *peer_device, esp_matter::client::request_handle_t *req_handle, void *priv_data) {
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
		false,    // auto_resubscribe
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
	uint8_t status_bits;
	switch (event->Type) {
	case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
		ESP_LOGI(TAG_EVENT, "🌐 IP Address Changed");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
		ESP_LOGI(TAG_EVENT, "✅ commissioning complete");
		event_stage = 2;
		led_indicator_set_color(&led_indicator_subsystem, 0, 255, 0); // green
		
		break;

	case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
		ESP_LOGI(TAG_EVENT, "❌ commissioning failed, fail safe timer expired");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
		ESP_LOGI(TAG_EVENT, "🔧 commissioning session started");
		event_stage = 1;
		led_indicator_set_color(&led_indicator_subsystem, 0, 16, 16); // dim blue
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
		ESP_LOGI(TAG_EVENT, "🔒 commissioning session stopped");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
		ESP_LOGI(TAG_EVENT, "🪟 commissioning window opened");
		event_stage = 0;
		led_indicator_set_color(&led_indicator_subsystem, 0, 0, 128); // blue
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
		ESP_LOGI(TAG_EVENT, "🔒 commissioning window closed");
		if (event_stage == 0) {
			// led_indicator_set_color(&led_indicator_subsystem, 255, 0, 0); // red
		}
		break;

	case chip::DeviceLayer::DeviceEventType::kServerReady:
		ESP_LOGI(TAG_EVENT, "💡 matter stack initialized. scheduling an initialization of the binding subscriptions.");
		chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t arg) { (void)arg;
			ESP_LOGI(TAG_EVENT, "initializing binding manager and reviving existing bindings...");
			revive_existing_bindings();
		});
		led_indicator_set_color(&led_indicator_subsystem, 255, 0, 0);
		subscription_manager_start_watchdog();
		revive_existing_bindings();
		break;
		
		
	case chip::DeviceLayer::DeviceEventType::kBindingsChangedViaCluster: {
		ESP_LOGI(TAG_EVENT, "🔄 bindings changed via cluster");
		
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
					ESP_LOGI(TAG_EVENT, "✅ found contact binding. Node=0x%llX, Ep=%d, Cluster=0x%" PRIx32, (unsigned long long)entry.nodeId, entry.remote, target_cluster);
					start_contact_subscription(entry.nodeId, entry.remote, target_cluster);
				}
			}
			break;
		}
		break;
	case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
		ESP_LOGI(TAG_EVENT, "🗑️ fabric removed, clearing bindings");
		break;
	case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
		ESP_LOGI(TAG_EVENT, "BLE deinitialized and memory reclaimed");
		break;
	default:
		ESP_LOGI(TAG_EVENT, "ℹ️ unhandled event %d", static_cast<int>(event->Type));
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
	root_endpoint = esp_matter::endpoint::get_first(node);
	if (!root_endpoint) {
		ESP_LOGE(TAG, "❌ failed to acquire root endpoint from node.");
		return ESP_FAIL;
	}
	
	esp_matter::cluster::common::config_t binding_config;
	root_binding_cluster = esp_matter::cluster::binding::create(root_endpoint, &binding_config, esp_matter::CLUSTER_FLAG_SERVER);
	if (!root_binding_cluster) {
		ESP_LOGE(TAG, "❌ failed to create binding cluster on root endpoint.");
		return ESP_FAIL;
	} else {
		ESP_LOGI(TAG, "✅ binding cluster added to root endpoint");
		return ESP_OK;
	}
}

extern "C" void app_main() {
	data_model_mux = xSemaphoreCreateMutex();
	if (!data_model_mux) {
		ESP_LOGE(TAG, "❌ Failed to create data model mutex");
		abort();
	}
	
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
	data_node = esp_matter::node::create(&node_config, app_attribute_update_cb, app_identification_cb);
	ABORT_APP_ON_FAILURE(data_node != nullptr, ESP_LOGE(TAG, "failed to create Matter node"));
	ESP_LOGI(TAG, "NODE CREATED :: endpoint count %i", esp_matter::endpoint::get_count(data_node));
	err = init_binding_cluster(data_node);
	ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "failed to initialize binding cluster"));
		
	/* ================================================================
	Manually build the Window Covering endpoint from scratch
	================================================================ */
	err = create_manual_window_covering_endpoint(data_node);
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
    /* This should be called before esp_matter::start() */
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
