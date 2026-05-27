#pragma once

#include <esp_matter.h>
#include <esp_matter_core.h>
#include <app/clusters/closure-control-server/closure-control-cluster-delegate.h>

namespace chip {
namespace app {
namespace Clusters {
namespace ClosureControl {

class MyClosureDelegate : public DelegateBase {
public:
    Protocols::InteractionModel::Status HandleStopCommand() override;
    Protocols::InteractionModel::Status HandleMoveToCommand(
        const Optional<TargetPositionEnum> & position,
        const Optional<bool> & latch,
        const Optional<Globals::ThreeLevelAutoEnum> & speed) override;
    Protocols::InteractionModel::Status HandleCalibrateCommand() override;
    bool IsReadyToMove() override;
    ElapsedS GetCalibrationCountdownTime() override;
    ElapsedS GetMovingCountdownTime() override;
    ElapsedS GetWaitingForMotionCountdownTime() override;
};

} // namespace ClosureControl
} // namespace Clusters
} // namespace app
} // namespace chip
