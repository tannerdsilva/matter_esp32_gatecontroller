#include "close_duration.h"
#include <vector>
#include <algorithm>
#include <cstdint>

// Enforced bounds for valid close durations (in milliseconds)
static const uint32_t MIN_DURATION_MS = 5000;
static const uint32_t MAX_DURATION_MS = 60000;

static std::vector<uint32_t> s_valid_durations;

static inline uint32_t calculate_duration(uint32_t start_ms, uint32_t end_ms) {
    return (end_ms >= start_ms) ? (end_ms - start_ms) : (UINT32_MAX - start_ms + end_ms);
}

bool close_duration_add(uint32_t start_ms, uint32_t end_ms) {
    uint32_t duration_ms = calculate_duration(start_ms, end_ms);

    // Enforce min/max bounds silently
    if (duration_ms >= MIN_DURATION_MS && duration_ms <= MAX_DURATION_MS) {
        // Find correct insertion point to keep the vector sorted
        auto it = std::upper_bound(s_valid_durations.begin(), s_valid_durations.end(), duration_ms);
        s_valid_durations.insert(it, duration_ms);
        return true;
    }

    return false;
}

float close_duration_get_median_ms(void) {
    if (s_valid_durations.empty()) {
        return 0.0f;
    }

    size_t n = s_valid_durations.size();
    if (n % 2 != 0) {
        // Odd sample count: exact middle element
        return static_cast<float>(s_valid_durations[n / 2]);
    } else {
        // Even sample count: average of the two middle elements
        float mid1 = static_cast<float>(s_valid_durations[(n / 2) - 1]);
        float mid2 = static_cast<float>(s_valid_durations[n / 2]);
        return (mid1 + mid2) / 2.0f;
    }
}
