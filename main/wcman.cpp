#include "wcman.h"
#include <esp_log.h>
#include <esp_timer.h>

static const char *TAG = "WC_MANAGER";

extern uint16_t switch_endpoint_id;

/* ------------------------------------------------------------------ */
/*  Extern declarations for motor relay (defined in app_driver.cpp)    */
/* ------------------------------------------------------------------ */
extern void motor_relay_toggle(void);
extern uint16_t switch_endpoint_id;
extern uint32_t s_closing_start_ms;
extern bool s_is_closing_active;


using namespace chip;
using namespace chip::app::Clusters::WindowCovering;

/* ------------------------------------------------------------------ */
CHIP_ERROR MyWindowCoveringManager::HandleMovement(WindowCoveringType type) {
    ESP_LOGI(TAG, "HandleMovement(%s)", 
             type == WindowCoveringType::Lift ? "LIFT" : "TILT");
    
    if (type != WindowCoveringType::Lift) {
        return CHIP_NO_ERROR;  // Only Lift supported — gate only
    }
 
	OperationalState current_op = OperationalStateGet(switch_endpoint_id, OperationalStatus::kLift);
    bool is_opening = (current_op == OperationalState::MovingUpOrOpen);
    
    ESP_LOGI(TAG, "🔄 Direction: %s", is_opening ? "OPENING" : "CLOSING");
	
	if (!is_opening) {
        s_closing_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
        s_is_closing_active = true;
        ESP_LOGI(TAG, "⏱️ Closing timer started at %lu ms", s_closing_start_ms);
    }
    
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
