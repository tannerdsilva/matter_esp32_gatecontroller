#pragma once

#include <map>

#include <esp_log.h>
#include <esp_matter_client.h>
#include <app/ReadClient.h>
#include <app/clusters/bindings/binding-table.h>
#include <app/AttributePathParams.h>
#include <lib/core/TLVReader.h>

// the primary structure of ownership for a subscription.
struct Subscription {
	uint16_t local_ep;
	uint16_t remote_ep;
	uint64_t remote_node_id;
	uint8_t fabric_index;
	chip::DeviceProxy *peer;
	std::unique_ptr<chip::app::ReadClient> read_client;
};

class SubscriptionManager {
	public:
		// access the global static instance of the manager.
		static SubscriptionManager &GetInstance() {
			static SubscriptionManager instance;
			return instance;
		}
		
		// install a binding into the subscription manager if it does not already exist. a graceful failure will occur (ESP_OK) if the binding already exists.
		esp_err_t AddBinding(const chip::app::Clusters::Binding::TableEntry &entry, std::map<uint64_t, std::unique_ptr<Subscription>> &new_sub_assemble);
		// called once after the entire table has been processed. any bindings that were not marked as "added" during the AddBinding phase will be removed during this phase.
		esp_err_t FinishAdditions(std::map<uint64_t, std::unique_ptr<Subscription>> &new_sub_assemble);

	private:
		// instance bullshit 
		SubscriptionManager() = default;
		~SubscriptionManager() = default;
		SubscriptionManager(const SubscriptionManager &) = delete;
		SubscriptionManager &operator=(const SubscriptionManager &) = delete;

		// stores the subscriptions for the bindings as they were known at the last time the bindings table was updated.
		std::map<uint64_t, std::unique_ptr<Subscription>> m_subs;

		// only to be used while the manager still owns the old map.
		Subscription *Find(const chip::app::Clusters::Binding::TableEntry &key);

		// keys - - - - - 

		// fabric values to hash
		uint64_t MakeKey(uint8_t fabric, uint64_t node, uint16_t ep);
		// hash to fabric values so it is safe to start the subscription before the unique_ptr is moved into the manager/ so it is safe to start the subscription before the unique_ptr is moved into the manager's map.'s map.
		void ReverseHash(uint64_t key, uint8_t &fabric, uint64_t &node, uint16_t &ep);
	
		// subscriptions - - - - -

		// initiates a subscription
		esp_err_t StartSubscription(Subscription *sub);
		// aborts and deletes a subscription.
		void AbortAndDelete(Subscription *sub);
};