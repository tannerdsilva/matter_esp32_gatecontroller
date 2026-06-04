#include "closure_control.h"
#include <esp_log.h>
#include <app_priv.h>

static const char *TAG = "CLOSURE_DELEGATE";

extern void motor_relay_toggle(void);

namespace chip {
namespace app {
namespace Clusters {
namespace ClosureControl {

using namespace chip::app::DataModel;

Protocols::InteractionModel::Status MyClosureDelegate::HandleStopCommand() {
    ESP_LOGI(TAG, "HandleStopCommand");
    motor_relay_toggle();  // Or skip for latching relay hardware
    return Protocols::InteractionModel::Status::Success;
}

Protocols::InteractionModel::Status MyClosureDelegate::HandleMoveToCommand(
    const Optional<TargetPositionEnum> & position,
    const Optional<bool> & latch,
    const Optional<Globals::ThreeLevelAutoEnum> & speed) {
    
    // FIX: position.Value() returns the enum directly. No second .Value() needed.
    ESP_LOGI(TAG, "HandleMoveToCommand - Position: %s (Value=%d)", 
             position.HasValue() ? "Provided" : "Not Provided",
             position.HasValue() ? static_cast<int>(position.Value()) : -1);
             
    if (latch.HasValue()) {
        ESP_LOGI(TAG, "  Latch requested: %s", latch.Value() ? "YES" : "NO");
    }
    
    motor_relay_toggle();
    return Protocols::InteractionModel::Status::Success;
}

Protocols::InteractionModel::Status MyClosureDelegate::HandleCalibrateCommand() {
    ESP_LOGI(TAG, "HandleCalibrateCommand");
    // Calibration: pulse relay to find limits
    return Protocols::InteractionModel::Status::Success;
}

bool MyClosureDelegate::IsReadyToMove() {
    return true;  // Gate is always ready
}

ElapsedS MyClosureDelegate::GetCalibrationCountdownTime() { return ElapsedS(0); }
ElapsedS MyClosureDelegate::GetMovingCountdownTime() { return ElapsedS(0); }
ElapsedS MyClosureDelegate::GetWaitingForMotionCountdownTime() { return ElapsedS(0); }

} // namespace ClosureControl
} // namespace Clusters
} // namespace app
} // namespace chip
