#pragma once

#include <esp_err.h>
#include <esp_matter.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

#ifdef CONFIG_SUBSCRIBE_AFTER_BINDING
#include "bindings_cluster.h"
#endif

typedef void *app_driver_handle_t;

typedef enum {
	SUBSCRIPTION_STATE_IDLE,				// no binding active
	SUBSCRIPTION_STATE_CONNECTING,			// CASE session being established
	SUBSCRIPTION_STATE_SUBSCRIBING,			// subscribe request sent
	SUBSCRIPTION_STATE_ESTABLISHED,			// subscription active
	SUBSCRIPTION_STATE_FAILED,				// connection/sub failed, waiting for retry
	SUBSCRIPTION_STATE_REVIVING,			// reviving from NVS on startup
} subscription_state_t;

typedef struct {
	chip::NodeId node_id;
	chip::EndpointId endpoint_id;
	uint32_t cluster_id;
	subscription_state_t state;
	int retry_count;
	uint32_t last_failure_time;				// ms since boot
} subscription_binding_t;

// initialize the subscription manager
void subscription_manager_init(void);

// called when a binding is removed (fabric removed, etc.)
void subscription_manager_on_binding_removed(chip::NodeId node_id);

// called to start or revive a subscription.
void subscription_manager_start_or_revive(chip::NodeId node_id, chip::EndpointId endpoint_id, uint32_t cluster_id);
    
// call this from OnError/OnDone callbacks in BooleanStateReadCallback
void subscription_manager_on_subscription_failed(void);

// call this from OnSubscriptionEstablished
void subscription_manager_on_subscription_established(void);

// call periodically (or from a task) to retry failed subscriptions
void subscription_manager_retry_pending(void);

