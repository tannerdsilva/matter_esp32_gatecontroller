#include "subscription_manager.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <app/clusters/bindings/binding-table.h>
#include <app/clusters/boolean-state-server/boolean-state-cluster.h>
#include <app/clusters/window-covering-server/window-covering-server.h>

#define TAG "SUB_MGR"

#define SUB_RETRY_MIN_MS      2000   // Minimum retry interval
#define SUB_RETRY_MAX_MS      30000  // Maximum retry interval (30s)
#define SUB_RETRY_BACKOFF_MULT 2     // Exponential backoff multiplier
#define SUB_MAX_RETRIES       10     // Max retries before giving up

// Global state
static subscription_binding_t s_current_binding = {
    .node_id = 0,
    .endpoint_id = 0,
    .cluster_id = 0,
    .state = SUBSCRIPTION_STATE_IDLE,
    .retry_count = 0,
    .last_failure_time = 0
};

/**
 * Calculate retry delay with exponential backoff + jitter
 */
static uint32_t calculate_retry_delay(int retry_count) {
    uint32_t delay = SUB_RETRY_MIN_MS;
    
    // Exponential backoff: 2s, 4s, 8s, 16s, ... up to 30s
    for (int i = 0; i < retry_count && i < 5; i++) {
        delay *= SUB_RETRY_BACKOFF_MULT;
        if (delay > SUB_RETRY_MAX_MS) {
            delay = SUB_RETRY_MAX_MS;
            break;
        }
    }
    
    // Add jitter ±25% to avoid thundering herd if multiple devices
    uint32_t jitter = delay / 4;
    delay += (esp_timer_get_time() % (jitter * 2)) - jitter;
    
    return delay;
}

void subscription_manager_start_or_revive(chip::NodeId node_id, chip::EndpointId endpoint_id, uint32_t cluster_id) {
    if (s_current_binding.state == SUBSCRIPTION_STATE_FAILED || s_current_binding.state == SUBSCRIPTION_STATE_REVIVING) {
        ESP_LOGI(TAG, "🔄 New binding arrived, cancelling pending retry");
    }
    
    s_current_binding.node_id = node_id;
    s_current_binding.endpoint_id = endpoint_id;
    s_current_binding.cluster_id = cluster_id;
    s_current_binding.state = SUBSCRIPTION_STATE_IDLE;
    s_current_binding.retry_count = 0;
    s_current_binding.last_failure_time = 0;
    
    ESP_LOGI(TAG, "📡 Binding added: Node=0x%llX, Ep=%d, Cluster=0x%" PRIx32, (unsigned long long)node_id, endpoint_id, cluster_id);
    
    esp_matter::client::request_handle_t *req_handle = chip::Platform::New<esp_matter::client::request_handle_t>();
    if (!req_handle) {
        ESP_LOGE(TAG, "❌ Failed to allocate request handle");
        return;
    }
    
    req_handle->type = esp_matter::client::SUBSCRIBE_ATTR;
    req_handle->attribute_path.mEndpointId = endpoint_id;
    req_handle->attribute_path.mClusterId = cluster_id;
    req_handle->attribute_path.mAttributeId = 0x0000;
    req_handle->request_data = nullptr;
    
    esp_err_t err = esp_matter::client::cluster_update(0, req_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to trigger cluster_update: %d", err);
        chip::Platform::Delete(req_handle);
        s_current_binding.state = SUBSCRIPTION_STATE_FAILED;
        s_current_binding.last_failure_time = esp_timer_get_time() / 1000;
    } else {
        s_current_binding.state = SUBSCRIPTION_STATE_CONNECTING;
        ESP_LOGI(TAG, "✅ Connection initiated successfully");
    }
}

/**
 * Initialize the subscription manager
 */
void subscription_manager_init(void) {
    ESP_LOGI(TAG, "🔧 Subscription manager initialized");
    s_current_binding.state = SUBSCRIPTION_STATE_IDLE;
}


/**
 * Called when a binding is removed (fabric removed, etc.)
 */
void subscription_manager_on_binding_removed(chip::NodeId node_id) {
    if (s_current_binding.node_id == node_id) {
        ESP_LOGI(TAG, "🗑️ Binding removed: Node=0x%" PRIx64, (uint64_t)node_id);
        s_current_binding.state = SUBSCRIPTION_STATE_IDLE;
        s_current_binding.retry_count = 0;
        s_current_binding.last_failure_time = 0;
    }
}

/**
 * Called from BooleanStateReadCallback::OnError() or OnDone()
 */
void subscription_manager_on_subscription_failed(void) {
    if (s_current_binding.state == SUBSCRIPTION_STATE_ESTABLISHED ||
        s_current_binding.state == SUBSCRIPTION_STATE_SUBSCRIBING ||
        s_current_binding.state == SUBSCRIPTION_STATE_CONNECTING) {
        
        ESP_LOGW(TAG, "⚠️ Subscription failed, scheduling retry (count=%d)", 
                 (int)(s_current_binding.retry_count + 1));
        
        s_current_binding.state = SUBSCRIPTION_STATE_FAILED;
        s_current_binding.last_failure_time = (uint32_t)(esp_timer_get_time() / 1000); // ms
    }
}

/**
 * Called from BooleanStateReadCallback::OnSubscriptionEstablished()
 */
void subscription_manager_on_subscription_established(void) {
    ESP_LOGI(TAG, "✅ Subscription established successfully");
    s_current_binding.state = SUBSCRIPTION_STATE_ESTABLISHED;
    s_current_binding.retry_count = 0; // Reset retry counter on success
}

/**
 * Retry pending subscriptions (call from timer/task or main loop)
 */
void subscription_manager_retry_pending(void) {
    if (s_current_binding.state != SUBSCRIPTION_STATE_FAILED) {
        return; // Nothing to retry
    }
    
    // Check if max retries exceeded
    if (s_current_binding.retry_count >= SUB_MAX_RETRIES) {
        ESP_LOGE(TAG, "❌ Max retries (%d) reached for Node=0x%" PRIx64, 
                 SUB_MAX_RETRIES, 
                 (uint64_t)s_current_binding.node_id);
        s_current_binding.state = SUBSCRIPTION_STATE_IDLE;
        return;
    }
    
    // Check if retry delay has passed
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000); // ms
    uint32_t elapsed = now - s_current_binding.last_failure_time;
    uint32_t delay = calculate_retry_delay(s_current_binding.retry_count);
    
    if (elapsed < delay) {
        ESP_LOGI(TAG, "⏳ Next retry in %" PRIu32 "ms (attempt %d/%d)", 
                 (uint32_t)(delay - elapsed), 
                 (int)(s_current_binding.retry_count + 1),
                 SUB_MAX_RETRIES);
        return; // Not yet time to retry
    }
    
    ESP_LOGI(TAG, "🔄 Attempting retry %d/%d for Node=0x%" PRIx64, 
             (int)(s_current_binding.retry_count + 1),
             SUB_MAX_RETRIES,
             (uint64_t)s_current_binding.node_id);
    
    subscription_manager_start_or_revive(s_current_binding.node_id, s_current_binding.endpoint_id, chip::app::Clusters::BooleanState::Id);
}

// ================================================================
// Watchdog task - runs periodically to retry failed subscriptions
// ================================================================
static void subscription_watchdog_task(void *pvParameters) {
    ESP_LOGI(TAG, "🕐 Subscription watchdog started");
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
        
        subscription_manager_retry_pending();
        
        // Also revive any bindings lost during network partition
        chip::app::Clusters::Binding::Table &table = 
            chip::app::Clusters::Binding::Table::GetInstance();
        
        for (size_t i = 0; i < table.Size(); i++) {
            chip::app::Clusters::Binding::TableEntry entry = table.GetAt(i);
            
            if (entry.type != chip::app::Clusters::Binding::MATTER_UNICAST_BINDING) {
                continue;
            }
            
            // Check if we have an active binding that's not tracked
            if (s_current_binding.node_id == 0 && 
                s_current_binding.state == SUBSCRIPTION_STATE_IDLE) {
                
                uint32_t target_cluster = entry.clusterId.has_value() ? 
                    entry.clusterId.value() : 0;
                
                // Match contact sensor clusters
                bool is_contact_binding = (target_cluster == chip::app::Clusters::BooleanState::Id ||
                                           target_cluster == chip::app::Clusters::OnOff::Id ||
                                           !entry.clusterId.has_value());
                
                if (is_contact_binding) {
                    ESP_LOGI(TAG, "🔍 Found untracked binding, reviving: Node=0x%" PRIx64, (uint64_t)entry.nodeId);
                    subscription_manager_start_or_revive(
                        entry.nodeId, 
                        entry.remote,
                        target_cluster);
                    break;
                }
            }
        }
    }
}

/**
 * Start the watchdog task (call once from app_main)
 */
void subscription_manager_start_watchdog(void) {
    xTaskCreatePinnedToCore(
        subscription_watchdog_task,
        "sub_watchdog",
        4096, // Stack size
        NULL,
        5,    // Priority
        NULL,
        1     // Core
    );
}
