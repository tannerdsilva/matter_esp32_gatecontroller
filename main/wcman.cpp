#include "wcman.h"
#include <esp_log.h>

static const char *TAG = "WC_MANAGER";

/* ------------------------------------------------------------------ */
/*  Extern declarations for motor relay (defined in app_driver.cpp)    */
/* ------------------------------------------------------------------ */
extern void motor_relay_toggle(void);

using namespace chip;
using namespace chip::app::Clusters::WindowCovering;

/* ------------------------------------------------------------------ */
CHIP_ERROR MyWindowCoveringManager::HandleMovement(WindowCoveringType type) {
    ESP_LOGI(TAG, "HandleMovement(%s)", 
             type == WindowCoveringType::Lift ? "LIFT" : "TILT");
    
    if (type != WindowCoveringType::Lift) {
        return CHIP_NO_ERROR;  // Only Lift supported — gate only
    }
    
    // Pulse the relay — the hardware interprets this as a move command
    motor_relay_toggle();
    
    return CHIP_NO_ERROR;
}

/* ------------------------------------------------------------------ */
CHIP_ERROR MyWindowCoveringManager::HandleStopMotion(void) {
    ESP_LOGI(TAG, "HandleStopMotion");
    
    // Pulse the relay to stop (latching relay hardware handles direction)
    motor_relay_toggle();
    
    return CHIP_NO_ERROR;
}
