#include "ACLAssist.hpp"

static chip::Access::Privilege GetRequiredPrivilege(chip::ClusterId clusterId) {
	for (const auto &e : kClusterPrivilegeMap) {
		if (e.cluster == clusterId) {
			return e.privilege;
		}
	}
	return chip::Access::Privilege::kView;
}

static CHIP_ERROR __acl_entry_already_exists(chip::FabricIndex fabricIndex, chip::NodeId nodeId, chip::EndpointId endpointId, chip::ClusterId clusterId, bool &doesExist) {
	doesExist = false;
	chip::Access::AccessControl::EntryIterator it;
	CHIP_ERROR err = chip::Access::GetAccessControl().Entries(fabricIndex, it);
	if (err != CHIP_NO_ERROR) {
		return err;   // be conservative – assume it does NOT exist
	}

	chip::Access::AccessControl::Entry entry;
	while ((err = it.Next(entry)) == CHIP_NO_ERROR) {
		// ----- subjects -------------------------------------------------
		size_t subjCnt = 0;
		entry.GetSubjectCount(subjCnt);
		for (size_t i = 0; i < subjCnt; ++i) {
			chip::NodeId storedNode = chip::kUndefinedNodeId;
			entry.GetSubject(i, storedNode);
			if (storedNode != nodeId) continue;
			// ----- targets ----------------------------------------------
			size_t tgtCnt = 0;
			entry.GetTargetCount(tgtCnt);
			for (size_t t = 0; t < tgtCnt; ++t) {
				chip::Access::AccessControl::Entry::Target tgt;
				entry.GetTarget(t, tgt);
				if ((tgt.flags & chip::Access::AccessControl::Entry::Target::kCluster) && (tgt.flags & chip::Access::AccessControl::Entry::Target::kEndpoint) && tgt.cluster == clusterId && tgt.endpoint == endpointId) {
						chip::Access::Privilege priv;
						if ((entry.GetPrivilege(priv) == CHIP_NO_ERROR) && (priv == GetRequiredPrivilege(clusterId))) {
							doesExist = true;
							return CHIP_NO_ERROR;
						}
				}
				
			}
		}
	}
	doesExist = false;
	return CHIP_NO_ERROR;
}

static CHIP_ERROR create_acl_entry(chip::FabricIndex fabricIndex, chip::NodeId nodeId, chip::EndpointId endpointId, chip::ClusterId clusterId, chip::Access::Privilege privilege) {
	// validate there is room in the acl for a new entry on this fabric.
	size_t maxEntries = 0;
	CHIP_ERROR err = chip::Access::GetAccessControl().GetMaxEntriesPerFabric(maxEntries);
	if (err != CHIP_NO_ERROR) {
		return err;
	}
	size_t used = 0;
	err = chip::Access::GetAccessControl().GetEntryCount(fabricIndex, used);
	if (err != CHIP_NO_ERROR) {
		return err;
	}
	if (used >= maxEntries) {
		return CHIP_ERROR_TOO_MANY_PEER_NODES;
	}

	// initial entry assemblly
	chip::Access::AccessControl::Entry entry;
	err = chip::Access::GetAccessControl().PrepareEntry(entry);
	if (err != CHIP_NO_ERROR) {
		return err;
	}
	err = entry.SetAuthMode(chip::Access::AuthMode::kCase);
	if (err != CHIP_NO_ERROR) {
		return err;
	}
	err = entry.SetFabricIndex(fabricIndex);
	if (err != CHIP_NO_ERROR) {
		return err;
	}
	err = entry.SetPrivilege(privilege);
	if (err != CHIP_NO_ERROR) {
		return err;
	}
	// handle the subject (remote node)
	size_t subjIdx = 0;
	err = entry.AddSubject(&subjIdx, nodeId);
	if (err != CHIP_NO_ERROR) {
		return err;
	}

	// handle the target (endpoint + cluster)
	chip::Access::AccessControl::Entry::Target tgt;
	tgt.flags = chip::Access::AccessControl::Entry::Target::kEndpoint | chip::Access::AccessControl::Entry::Target::kCluster;
	tgt.endpoint = endpointId;
	tgt.cluster = clusterId;

	size_t tgtIdx = 0;
	err = entry.AddTarget(&tgtIdx, tgt);
	if (err != CHIP_NO_ERROR) {
		return err;
	}

	// store it
	size_t newIdx = 0;
	err = chip::Access::GetAccessControl().CreateEntry(nullptr, fabricIndex, &newIdx, entry);
	if (err != CHIP_NO_ERROR) {
		return err;
	}
	return CHIP_NO_ERROR;
}

static CHIP_ERROR __ensure_acl_bindings(void) {
	// walk the stored bindings and make sure we have ACL entries for them.
	for (const auto &binding : chip::BindingTable::GetInstance()) {
		if (binding.type != MATTER_UNICAST_BINDING) {
			continue;
		}
		if (!binding.clusterId.has_value()) {
			continue;
		}
		const chip::ClusterId clusterId = binding.clusterId.value();
		const chip::Access::Privilege requiredPriv = GetRequiredPrivilege(clusterId);
		const ACLCacheKey key{
			.fabricIndex = binding.fabricIndex,
			.nodeId      = binding.nodeId,
			.endpointId  = binding.remote,   // remote endpoint on *our* device
			.clusterId   = clusterId,
		};
		if (g_aclCache.find(key) != g_aclCache.end()) {
			continue;
		}

		// ask acl storage if the entry already exists.
		bool alreadyExists = false;
		CHIP_ERROR err = __acl_entry_already_exists(binding.fabricIndex, binding.nodeId, binding.remote, clusterId, alreadyExists);
		if (err != CHIP_NO_ERROR) {
			return err;   // be conservative – if we can’t check, assume it does NOT exist
		}

		if (alreadyExists) {
			g_aclCache.insert(key);
			continue;
		}

		// create the entry. create function already checks the per fabric limit.
		err = create_acl_entry(binding.fabricIndex,
							binding.nodeId,
							binding.remote,
							clusterId,
							requiredPriv);
		if (err == CHIP_NO_ERROR) {
			// created acl entry
			// remember it for the rest of the session.
			g_aclCache.insert(key);
		} else if (err == CHIP_ERROR_TOO_MANY_PEER_NODES) {
			return err;
		} else {
			return err;
		}
	}
	// no problems here.
	return CHIP_NO_ERROR;
}
