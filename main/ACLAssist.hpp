#pragma once

#include <access/AccessControl.h>
#include <access/SubjectDescriptor.h>
#include <access/Privilege.h>
#include <access/AuthMode.h>
#include <app/util/binding-table.h>
#include <platform/CHIPDeviceLayer.h>
#include <set>
#include <lib/core/CHIPError.h>

// a static mapping of cluster IDs to the privileges we want to grant for that cluster.  this is used when we create ACL entries for bindings.
static const struct {
    chip::ClusterId   cluster;
    chip::Access::Privilege privilege;
} kClusterPrivilegeMap[] = {
    { chip::app::Clusters::BooleanState::Id, chip::Access::Privilege::kView },
    { chip::app::Clusters::OnOff::Id, chip::Access::Privilege::kOperate },
    { chip::app::Clusters::LevelControl::Id, chip::Access::Privilege::kOperate },
};

// convenience function to return the statically configured privilege for a given cluster ID, or kView if the cluster ID is not found in the map.
static chip::Access::Privilege GetRequiredPrivilege(chip::ClusterId clusterId);

struct ACLCacheKey {
    chip::FabricIndex fabricIndex;
    chip::NodeId      nodeId;
    chip::EndpointId  endpointId;
    chip::ClusterId   clusterId;

    bool operator<(const ACLCacheKey &rhs) const {
        return std::tie(fabricIndex, nodeId, endpointId, clusterId) < std::tie(rhs.fabricIndex, rhs.nodeId, rhs.endpointId, rhs.clusterId);
    }
};

static std::set<ACLCacheKey> g_aclCache;

static CHIP_ERROR __acl_entry_already_exists(chip::FabricIndex fabricIndex, chip::NodeId nodeId, chip::EndpointId endpointId, chip::ClusterId clusterId, bool &doesExist);
static CHIP_ERROR __ensure_acl_bindings(void);