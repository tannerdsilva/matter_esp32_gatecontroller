#pragma once

#ifdef CONFIG_SUBSCRIBE_AFTER_BINDING
#include "esp_matter_client.h"
#include <lib/core/TLVReader.h>
#include <app/AttributePathParams.h>
#include <app/ConcreteAttributePath.h>

// a simple structure that is used to keep the information of a remote subscription..
struct RemoteSubscription {
	// the node id that the subscription is associated with.
    chip::NodeId nodeId = chip::kUndefinedNodeId;
	// the matter fabric id that the subscription is associated with.
    chip::FabricIndex fabricIndex = chip::kUndefinedFabricIndex;
	// the endpoint id that the subscription is associated with.
    chip::EndpointId endpoint;
	// the cluster id that the subscription is associated with.
    chip::ClusterId  clusterId  = chip::kInvalidClusterId;

	// the memory that this pointer refers to is owned by the SDK, we just keep the pointer here for later use in the subscribe callback.
    chip::app::ReadClient *client = nullptr;   // owned by SDK, we just keep the pointer
};

static std::vector<RemoteSubscription> g_subscriptions;
static const uint16_t kMinIntervalSec = 5;
static const uint16_t kMaxIntervalSec = 10;

static RemoteSubscription* find_or_create_subscription(const EmberBindingTableEntry &binding);

#endif
// class MatterClientReadHandler : public chip::app::ReadClient::Callback {
// public:
// 	void OnSubscriptionEstablished(chip::SubscriptionId aSubscriptionId) override;
//     void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath, chip::TLV::TLVReader *aReader, const chip::app::StatusIB &aStatus) override;
//     void OnEventData(const chip::app::EventHeader &aEventHeader, chip::TLV::TLVReader * apData, const chip::app::StatusIB *aStatus) override;
//     void OnError(CHIP_ERROR aError) override;
//     void OnDone(chip::app::ReadClient * apReadClient) override;
// };
// #endif