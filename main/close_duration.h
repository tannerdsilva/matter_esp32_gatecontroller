#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * Add a new close duration measurement.
 * @param start_ms  Timestamp when closing began (e.g., motor toggle time)
 * @param end_ms    Timestamp when closed was detected
 * @return true if measurement was within bounds and accepted, false otherwise
 */
bool close_duration_add(uint32_t start_ms, uint32_t end_ms);

/**
 * Returns the current rolling median of accepted measurements.
 * Returns 0.0f until at least one valid measurement is recorded.
 */
float close_duration_get_median_ms(void);
