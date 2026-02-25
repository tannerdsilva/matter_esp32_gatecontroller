#pragma once

#include <access/AccessControl.h>
#include <access/SubjectDescriptor.h>
#include <access/Privilege.h>
#include <access/AuthMode.h>
#include <app/util/binding-table.h>
#include <platform/CHIPDeviceLayer.h>

// a static mapping of cluster IDs to the privileges we want to grant for that cluster.  this is used when we create ACL entries for bindings.
static const struct {
    chip::ClusterId   cluster;
    chip::Access::Privilege privilege;
} kClusterPrivilegeMap[] = {
    { chip::app::Clusters::BooleanState::Id, chip::Access::Privilege::kView },
    { chip::app::Clusters::OnOff::Id, chip::Access::Privilege::kOperate },
    { chip::app::Clusters::LevelControl::Id, chip::Access::Privilege::kOperate },
};
