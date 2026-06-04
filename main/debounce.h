#pragma once

#include <stdint.h>

/* Check if enough time has elapsed since the last check or reset.
 * Returns true if threshold has passed, and resets the timer.
 * Returns false if threshold has not yet been reached. */
bool debounce_check(void);

/* Unconditionally set the stored timestamp to current time.
 * Useful for "resetting" the debounce window on demand. */
void debounce_reset(void);
