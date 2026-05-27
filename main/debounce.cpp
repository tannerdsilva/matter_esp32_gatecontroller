#include "debounce.h"
#include <esp_timer.h>

// Fixed threshold: 500ms (adjust as needed)
#define DEBOUNCE_THRESHOLD_MS 500

static uint32_t s_last_time_ms = 0;
static bool s_initialized = false;

static uint32_t get_current_time_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000); // Convert µs → ms
}

void debounce_init(void) {
    s_last_time_ms = get_current_time_ms();
    s_initialized = true;
}

bool debounce_check(void) {
    if (!s_initialized) {
        debounce_init();
    }

    uint32_t now = get_current_time_ms();
    
    // Handle uint32_t wraparound safely
    uint32_t elapsed = (now >= s_last_time_ms) ? (now - s_last_time_ms) : (UINT32_MAX - s_last_time_ms + now);
    
    if (elapsed >= DEBOUNCE_THRESHOLD_MS) {
        s_last_time_ms = now;
        return true;
    }

    return false;
}

void debounce_reset(void) {
    s_last_time_ms = get_current_time_ms();
}
