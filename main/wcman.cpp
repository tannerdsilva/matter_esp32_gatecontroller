#include "wcman.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_matter_attribute_utils.h>

static const char *TAG = "WC_MANAGER";

/* ------------------------------------------------------------------ */
/*  Extern declarations for global state (defined in app_main.cpp)     */
/* ------------------------------------------------------------------ */
extern uint16_t wc_endpoint_id;
extern uint32_t s_closing_start_ms;
extern bool s_is_closing_active;
extern uint32_t s_motor_start_ms;
extern bool s_motor_active;
extern uint8_t s_last_known_position;

/* ------------------------------------------------------------------ */
/*  Extern declarations for motor relay (defined in app_driver.cpp)    */
/* ------------------------------------------------------------------ */
extern void motor_relay_toggle_async(void);
extern void motor_relay_stop_immediate(void);

using namespace chip;
using namespace chip::app::Clusters::WindowCovering;

CHIP_ERROR MyWindowCoveringManager::HandleMovement(WindowCoveringType type) {
    ESP_LOGI(TAG, "🔥 HandleMovement called with type=%d", (int)type);
    
    if (type != WindowCoveringType::Lift) {
        return CHIP_NO_ERROR;  
    }

    s_motor_active = true;
    s_motor_start_ms = (uint32_t)(esp_timer_get_time() / 1000);

    bool is_opening = (s_last_known_position == 0);
    
    ESP_LOGI(TAG, "🔄 Direction: %s (from pos=%d%%)", is_opening ? "OPENING" : "CLOSING", s_last_known_position);

    OperationalState newState = is_opening ? OperationalState::MovingUpOrOpen : OperationalState::MovingDownOrClose;

    OperationalStateSet(wc_endpoint_id, OperationalStatus::kLift, newState);
    ESP_LOGI(TAG, "✅ Operational status updated to %d", chip::to_underlying(newState));

    motor_relay_toggle_async();

    return CHIP_NO_ERROR;
}

CHIP_ERROR MyWindowCoveringManager::HandleStopMotion(void) {
    ESP_LOGI(TAG, "HandleStopMotion");
    
    motor_relay_toggle_async();
    
    // Update operational status back to Stall via proper API
    OperationalStateSet(wc_endpoint_id, OperationalStatus::kLift, 
                        OperationalState::Stall);
    
    return CHIP_NO_ERROR;
}
