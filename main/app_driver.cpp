/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "esp_matter_client.h"
#include <cstddef>
#include <cstdio>
#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include "bsp/esp-bsp.h"

#include <app_priv.h>
#include <app_reset.h>

#include <app/server/Server.h>
#include <lib/core/Optional.h>

#ifdef CONFIG_SUBSCRIBE_AFTER_BINDING
#include <app/AttributePathParams.h>
#include <app/ConcreteAttributePath.h>
#include <lib/core/TLVReader.h>
#endif

using chip::kInvalidClusterId;
static constexpr chip::CommandId kInvalidCommandId = 0xFFFF'FFFF;

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;

static const char *TAG = "app_driver";
extern uint16_t switch_endpoint_id;

#ifdef CONFIG_SUBSCRIBE_AFTER_BINDING
class MyReadClientCallback : public chip::app::ReadClient::Callback {
public:
    void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath,
                         chip::TLV::TLVReader *aReader,
                         const chip::app::StatusIB &aStatus) override {
        // Handle the attribute data
        if (aPath.mClusterId == chip::app::Clusters::BooleanState::Id) {
            if (aPath.mAttributeId == chip::app::Clusters::BooleanState::Attributes::StateValue::Id) {
                ESP_LOGI(TAG, "Received BooleanState attribute");
            }
        }
    }

    void OnEventData(const chip::app::EventHeader &aEventHeader, chip::TLV::TLVReader * apData,
                     const chip::app::StatusIB *aStatus) override {
        // Handle event data
    }

    void OnError(CHIP_ERROR aError) override {
        // Handle the error
        ESP_LOGI(TAG, "ReadClient Error: %s", ErrorStr(aError));
    }

    void OnDone(chip::app::ReadClient * apReadClient) override {
        // Cleanup after done
        ESP_LOGI(TAG, "ReadClient Done");
    }
};
MyReadClientCallback readClientCb;

void app_client_subscribe_command_callback(client::peer_device_t *peer_device, client::request_handle_t *req_handle,
                                               void *priv_data)
{
    uint16_t min_interval = 5;
    uint16_t max_interval = 10;
    bool keep_subscription = true;
    bool auto_resubscribe = true;
    chip::Platform::ScopedMemoryBufferWithSize<chip::app::AttributePathParams> attrb_path;
    attrb_path.Alloc(1);
    client::interaction::subscribe::send_request(peer_device, &req_handle->attribute_path, attrb_path.AllocatedSize(),
                                                &req_handle->event_path, 0, min_interval, max_interval, keep_subscription,
                                                auto_resubscribe, readClientCb);
}

#endif

static void send_command_success_callback(void *context, const ConcreteCommandPath &command_path,
                                          const chip::app::StatusIB &status, TLVReader *response_data)
{
    ESP_LOGI(TAG, "Send command success");
}

static void send_command_failure_callback(void *context, CHIP_ERROR error)
{
    ESP_LOGI(TAG, "Send command failure: err :%" CHIP_ERROR_FORMAT, error.Format());
}

void app_driver_client_invoke_command_callback(client::peer_device_t *peer_device, client::request_handle_t *req_handle,
                                               void *priv_data)
{
    if (req_handle->type == esp_matter::client::INVOKE_CMD) {
        char command_data_str[32];
        // on_off light switch should support on_off cluster and identify cluster commands sending.
        if (req_handle->command_path.mClusterId == OnOff::Id) {
            strcpy(command_data_str, "{}");
        } else if (req_handle->command_path.mClusterId == Identify::Id) {
            if (req_handle->command_path.mCommandId == Identify::Commands::Identify::Id) {
                if (((char *)req_handle->request_data)[0] != 1) {
                    ESP_LOGE(TAG, "Number of parameters error");
                    return;
                }
                snprintf(command_data_str, sizeof(command_data_str), "{\"0:U16\": %ld}",
                        strtoul((const char *)(req_handle->request_data) + 1, NULL, 16));
            } else {
                ESP_LOGE(TAG, "Unsupported command");
                return;
            }
        } else {
            ESP_LOGE(TAG, "Unsupported cluster");
            return;
        }
        client::interaction::invoke::send_request(NULL, peer_device, req_handle->command_path, command_data_str,
                                                  send_command_success_callback, send_command_failure_callback,
                                                  chip::NullOptional);
    }
    return;
}

void app_driver_client_callback(client::peer_device_t *peer_device, client::request_handle_t *req_handle,
                                               void *priv_data)
{
    if (req_handle->type == esp_matter::client::INVOKE_CMD) {
        app_driver_client_invoke_command_callback(peer_device, req_handle, priv_data);
#ifdef CONFIG_SUBSCRIBE_AFTER_BINDING
    } else if (req_handle->type == esp_matter::client::SUBSCRIBE_ATTR) {
        app_client_subscribe_command_callback(peer_device, req_handle, priv_data);
#endif
    }
    return;
}
void app_driver_client_group_invoke_command_callback(uint8_t fabric_index, client::request_handle_t *req_handle,
                                                     void *priv_data)
{
    if (req_handle->type != esp_matter::client::INVOKE_CMD) {
        return;
    }
    char command_data_str[32];
    // on_off light switch should support on_off cluster and identify cluster commands sending.
    if (req_handle->command_path.mClusterId == OnOff::Id) {
        strcpy(command_data_str, "{}");
    } else if (req_handle->command_path.mClusterId == Identify::Id) {
        if (req_handle->command_path.mCommandId == Identify::Commands::Identify::Id) {
            if (((char *)req_handle->request_data)[0] != 1) {
                ESP_LOGE(TAG, "Number of parameters error");
                return;
            }
            snprintf(command_data_str, sizeof(command_data_str), "{\"0:U16\": %ld}",
                    strtoul((const char *)(req_handle->request_data) + 1, NULL, 16));
        } else {
            ESP_LOGE(TAG, "Unsupported command");
            return;
        }
    } else {
        ESP_LOGE(TAG, "Unsupported cluster");
        return;
    }
    client::interaction::invoke::send_group_request(fabric_index, req_handle->command_path, command_data_str);
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Toggle button pressed");
    client::request_handle_t req_handle;
    req_handle.type = esp_matter::client::INVOKE_CMD;
    req_handle.command_path.mClusterId = OnOff::Id;
    req_handle.command_path.mCommandId = OnOff::Commands::Toggle::Id;

    lock::chip_stack_lock(portMAX_DELAY);
    client::cluster_update(switch_endpoint_id, &req_handle);
    lock::chip_stack_unlock();
}

app_driver_handle_t app_driver_switch_init()
{
    /* Initialize button */

    button_handle_t btns[BSP_BUTTON_NUM];
    ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
    ESP_ERROR_CHECK(iot_button_register_cb(btns[0], BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL));

    /* Other initializations */
#if CONFIG_ENABLE_CHIP_SHELL
    app_driver_register_commands();
#endif // CONFIG_ENABLE_CHIP_SHELL
    client::set_request_callback(app_driver_client_callback,
                                 app_driver_client_group_invoke_command_callback, NULL);

    return (app_driver_handle_t)btns[0];
}
