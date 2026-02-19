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
#include <app/clusters/window-covering-server/window-covering-server.h>
#include <app/clusters/boolean-state-configuration-server/boolean-state-configuration-server.h>
#include <esp_matter_console.h>
#include <esp_matter_providers.h>

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>
#include "wcman.h"
#include "driver/gpio.h"

#define LED_GPIO   GPIO_NUM_8
#define BLINK_MS   200

// pin layouts picked based on the esp32-h2 dev board.
#define GPIO_OUTPUT_COVER_OPEN			GPIO_NUM_2
#define GPIO_OUTPUT_COVER_CLOSE			GPIO_NUM_3
#define GPIO OUTPUT_COVER_STOP			GPIO_NUM_4
#define GPIO_INPUT_COVER_CLOSED			GPIO_NUM_10

#if CONFIG_SUBSCRIBE_TO_ON_OFF_SERVER_AFTER_BINDING
#include <app/util/binding-table.h>
#include <esp_matter_client.h>
#include <app/AttributePathParams.h>
#include <app/ConcreteAttributePath.h>
#include <lib/core/TLVReader.h>
#include <app/server/Server.h>
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

static chip::app::Clusters::WindowCovering::MyWindowCoveringManager *window_covering_manager = new chip::app::Clusters::WindowCovering::MyWindowCoveringManager();

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;

#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
dynamic_commissionable_data_provider g_dynamic_passcode_provider;
#endif

#if CONFIG_SUBSCRIBE_TO_ON_OFF_SERVER_AFTER_BINDING
static bool do_subscribe = true;
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

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg) {
    switch (event->Type) {
	case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
		ESP_LOGI("STATECHANGE", "interface IP Address Changed");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
		ESP_LOGI("STATECHANGE", "commissioning complete");
		break;

	case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
		ESP_LOGI("STATECHANGE", "commissioning failed, fail safe timer expired");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
		ESP_LOGI("STATECHANGE", "commissioning session started");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
		ESP_LOGI("STATECHANGE", "commissioning session stopped");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
		ESP_LOGI("STATECHANGE", "commissioning window opened");
		break;

	case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
		ESP_LOGI("STATECHANGE", "commissioning window closed");
		break;
	case chip::DeviceLayer::DeviceEventType::kServerReady:
		ESP_LOGI("STATECHANGE", "server ready");
		break;

	case chip::DeviceLayer::DeviceEventType::kBindingsChangedViaCluster: {
		ESP_LOGI("STATECHANGE", "binding entry changed");
#if CONFIG_SUBSCRIBE_TO_ON_OFF_SERVER_AFTER_BINDING
		if (do_subscribe) {
			for (const auto & binding : chip::BindingTable::GetInstance())
			{
				ESP_LOGI(
					TAG,
					"Read cached binding type=%d fabrixIndex=%d nodeId=0x" ChipLogFormatX64
					" groupId=%d local endpoint=%d remote endpoint=%d cluster=" ChipLogFormatMEI,
					binding.type, binding.fabricIndex, ChipLogValueX64(binding.nodeId), binding.groupId, binding.local,
					binding.remote, ChipLogValueMEI(binding.clusterId.value_or(0)));
				if (binding.type == MATTER_UNICAST_BINDING && event->BindingsChanged.fabricIndex == binding.fabricIndex)
				{
					ESP_LOGI(
						TAG,
						"Matched accessingFabricIndex with nodeId=0x" ChipLogFormatX64,
						ChipLogValueX64(binding.nodeId));

					uint32_t attribute_id = chip::app::Clusters::OnOff::Attributes::OnOff::Id;
					client::request_handle_t req_handle;
					req_handle.type = esp_matter::client::SUBSCRIBE_ATTR;
					req_handle.attribute_path = {binding.remote, binding.clusterId.value(), attribute_id};
					auto &server = chip::Server::GetInstance();
					client::connect(server.GetCASESessionManager(), binding.fabricIndex, binding.nodeId, &req_handle);
					break;
				}
			}
			do_subscribe = false;
		}
#endif
    }
    break;

    default:
		 ESP_LOGI("STATECHANGE", "unhandled event type: %d", event->Type);
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id, uint8_t effect_variant, void *priv_data) {
    ESP_LOGI(TAG, "identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data) {
    if (type == PRE_UPDATE) {
		gpio_set_level(LED_GPIO, 1);   // HIGH (ON)
		vTaskDelay(pdMS_TO_TICKS(BLINK_MS));
		ESP_LOGI(TAG, "toggling LED GPIO%d", LED_GPIO);
		gpio_set_level(LED_GPIO, 0);   // LOW (OFF)
    }

	ESP_LOGI(TAG, "attribute updated: endpoint_id=%d", endpoint_id);
    return ESP_OK;
}


static attribute_t *cover_currentPositionLiftPercent100ths;
static attribute_t *cover_targetPositionLiftPercent100ths;
static attribute_t *cover_operationalStatus;
static attribute_t *cover_configStatus;
static attribute_t *cover_mode;
static attribute_t *cover_featureMap;

static chip::EndpointId coverEPID = chip::kInvalidEndpointId;
void cover_set_endpoint_id(chip::EndpointId epId) { coverEPID = epId; }
void cover_update_state(bool isOpen, bool isClosed, bool movingOpen, bool movingClosed) {
	if (coverEPID == chip::kInvalidEndpointId) {
		ESP_LOGE(TAG, "cover_update_state called before cover_set_endpoint_id");
		return;
	}
	uint16_t targetPos = 0;
	if (isOpen) {
		targetPos = 1000;
	} else if (isClosed) {
		targetPos = 0;
	}

	// set CurrentPositionLiftPercent100ths attribute based on the state of the cover
	esp_matter_attr_val_t currentPosAttrVal = {
		.type = ESP_MATTER_VAL_TYPE_UINT16,
	};
	if (isOpen) {
		currentPosAttrVal.val.u16 = 1000;
	} else if (isClosed) {
		currentPosAttrVal.val.u16 = 0;
	} else if (movingOpen) {
		currentPosAttrVal.val.u16 = 500; // for demo purposes, we set it to 50% when it's moving
	} else if (movingClosed) {
		currentPosAttrVal.val.u16 = 500; // for demo purposes, we set it to 50% when it's moving
	} else {
		currentPosAttrVal.val.u16 = targetPos; // if the cover is in an unknown state, we set the current position to the target position to avoid confusion.
	}
	esp_err_t err = attribute::update(
		coverEPID,
        chip::app::Clusters::WindowCovering::Id,
        chip::app::Clusters::WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id,
        &currentPosAttrVal
	);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to update CurrentPositionLiftPercent100ths attribute");
	} else {
		ESP_LOGD(TAG, "Updated CurrentPositionLiftPercent100ths attribute");
	}

	// set TargetPositionLiftPercent100ths attribute to 0 when the cover is closed, and 1000 when the cover is open. This is to make sure that the target position is in sync with the current position when the cover is fully open or fully closed. When the cover is moving, we don't update the target position so that it can be controlled by the user or other controllers.
	uint16_t targetPosVal = 0xFFFF;
	if (movingOpen) {
		targetPosVal = 10000;
	} else if (movingClosed) {
		targetPosVal = 0;
	} else if (isOpen) {
		targetPosVal = 10000;
	} else if (isClosed) {
		targetPosVal = 0;
	}
	if (targetPosVal != 0xFFFF) {
		esp_matter_attr_val_t targetPosAttrVal = {
			.type = ESP_MATTER_VAL_TYPE_UINT16,
			.val = {
				.u16 = targetPosVal,
			}
		};
		err = attribute::update(coverEPID,
			chip::app::Clusters::WindowCovering::Id,
			chip::app::Clusters::WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id,
			&targetPosAttrVal
		);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Failed to update TargetPositionLiftPercent100ths attribute");
		} else {
			ESP_LOGD(TAG, "Updated TargetPositionLiftPercent100ths attribute");
		}
	}

	// set OperationalStatus attribute based on the state of the cover
	using namespace chip::app::Clusters::WindowCovering;
	uint8_t opStatus = 0;
	if (movingOpen) {
		opStatus = static_cast<uint8_t>(OperationalState::MovingUpOrOpen);
	} else if (movingClosed) {
		opStatus = static_cast<uint8_t>(OperationalState::MovingDownOrClose);
	} else {
		opStatus = static_cast<uint8_t>(OperationalState::Stall);
	}
	esp_matter_attr_val_t opStatusAttrVal = {
		.type = ESP_MATTER_VAL_TYPE_BITMAP8,
		.val = {
			.u8 = opStatus,
		}
	};
	err = attribute::update(coverEPID,
		chip::app::Clusters::WindowCovering::Id,
		chip::app::Clusters::WindowCovering::Attributes::OperationalStatus::Id,
		&opStatusAttrVal
	);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to update OperationalStatus attribute");
	} else {
		ESP_LOGD(TAG, "Updated OperationalStatus attribute");
	}
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

	node::config_t node_config;
	node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
	endpoint_t *rootEndpoint = endpoint::get_first(node); // initialize the id of the root endpoint before using it in the attribute update callback

	ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "failed to create Matter node"));

	#ifdef CONFIG_ENABLE_SNTP_TIME_SYNC
	endpoint_t *root_node_ep = endpoint::get_first(node);
	ABORT_APP_ON_FAILURE(root_node_ep != nullptr, ESP_LOGE(TAG, "Failed to find root node endpoint"));

	cluster::time_synchronization::config_t time_sync_cfg;
	static chip::app::Clusters::TimeSynchronization::DefaultTimeSyncDelegate time_sync_delegate;
	time_sync_cfg.delegate = &time_sync_delegate;
	cluster_t *time_sync_cluster = cluster::time_synchronization::create(root_node_ep, &time_sync_cfg, CLUSTER_FLAG_SERVER);
	ABORT_APP_ON_FAILURE(time_sync_cluster != nullptr, ESP_LOGE(TAG, "Failed to create time_sync_cluster"));

	cluster::time_synchronization::feature::time_zone::config_t tz_cfg;
	cluster::time_synchronization::feature::time_zone::add(time_sync_cluster, &tz_cfg);
	#endif

	window_covering_device::config_t window_covering_device_config(static_cast<uint8_t>(chip::app::Clusters::WindowCovering::EndProductType::kUnknown));
	window_covering_device_config.window_covering.feature_flags = (uint32_t)chip::app::Clusters::WindowCovering::Feature::kLift;
	window_covering_device_config.window_covering.type = (uint8_t)chip::app::Clusters::WindowCovering::Type::kUnknown;
	window_covering_device_config.window_covering.config_status = 0x01;
	window_covering_device_config.window_covering.operational_status = 0x00;
	window_covering_device_config.window_covering.mode = 0x00;

	endpoint_t *endpoint = window_covering_device::create(node, &window_covering_device_config, ENDPOINT_FLAG_NONE, NULL);
	cover_set_endpoint_id(endpoint::get_id(endpoint));
	cluster::binding::config_t common_config = {};
	cluster_t *bindingCluster = cluster::binding::create(rootEndpoint, &common_config, CLUSTER_FLAG_SERVER);

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
	// attribute::set_value(mode, 0x00);
	
	// attribute_t *featureMap = global::attribute::create_feature_map(windowCoveringCluser, (uint32_t)chip::app::Clusters::WindowCovering::Feature::kLift);
	/*
	attribute_t *openPositionLimit = cluster::window_covering::attribute::create_installed_open_limit_lift(windowCoveringCluser, 1024);
	attribute_t *closePositionLimit = cluster::window_covering::attribute::create_installed_closed_limit_lift(windowCoveringCluser, 0);
	attribute_t *currentPositionLiftPercent = cluster::window_covering::attribute::create_target_position_lift_percent_100ths(windowCoveringCluser, 5000);

	attribute_t *opStatus = cluster::window_covering::attribute::create_operational_status(windowCoveringCluser, (uint8_t)chip::app::Clusters::WindowCovering::OperationalStatus::kLift);
	attribute_t *configStatus = cluster::window_covering::attribute::create_config_status(windowCoveringCluser, ((uint8_t)chip::app::Clusters::WindowCovering::ConfigStatus::kOperational) | ((uint8_t)chip::app::Clusters::WindowCovering::ConfigStatus::kOnlineReserved));
	attribute_t *mode = cluster::window_covering::attribute::create_mode(windowCoveringCluser, 0x00);
	// attribute_t *featureMap = global::attribute::create_feature_map(windowCoveringCluser, (uint32_t)chip::app::Clusters::WindowCovering::Feature::kLift);
	*/

	ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "failed to create on off switch endpoint"));
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
	/* This should be called before esp_matter::start() */
	esp_matter::set_custom_commissionable_data_provider(&g_dynamic_passcode_provider);
#endif
	err = esp_matter::start(app_event_cb, NULL);
	ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "failed to start Matter, err:%d", err));
	gpio_config_t io_conf = {
		.pin_bit_mask = (1ULL << LED_GPIO),
		.mode         = GPIO_MODE_OUTPUT,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	gpio_config(&io_conf);
}
