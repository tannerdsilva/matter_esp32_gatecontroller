#include "wcman.h"
#include "app/util/attribute-storage.h"
#include "esp_log.h"

static const char *TAG = "WC_MANAGER";

/* --------------------------------------------------------------------- */
using namespace chip;
using namespace chip::app::Clusters::WindowCovering;

/* --------------------------------------------------------------------- */
CHIP_ERROR MyWindowCoveringManager::HandleMovement(WindowCoveringType type) {
    ESP_LOGE(TAG, "HANDLEMOVEMENT REQUEST");
    return CHIP_NO_ERROR;
}

/* --------------------------------------------------------------------- */
CHIP_ERROR MyWindowCoveringManager::HandleStopMotion() {
    ESP_LOGE(TAG, "HANDLESTOPMOTION REQUEST");
    return CHIP_NO_ERROR;
}
