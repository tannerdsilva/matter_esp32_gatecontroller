#pragma once
#include "driver/gpio.h"
#include "led_strip.h"
#include "led_strip_types.h"
#include <atomic>
#include <mutex>

/// @brief structure for managing the led indicator subsystem, which includes the led strip handle and the current color state of the indicator.
struct led_indicator_subsystem {
	led_strip_handle_t indicator;
	std::atomic<uint8_t> r;
	std::atomic<uint8_t> g;
	std::atomic<uint8_t> b;
	std::mutex mutex;
};
typedef struct led_indicator_subsystem led_indicator_subsystem_t;

int led_indicator_init(led_indicator_subsystem_t *subsystem, const gpio_num_t gpio_num);
int led_indicator_set_color(led_indicator_subsystem_t *subsystem, const uint8_t r, const uint8_t g, const uint8_t b);
int led_indicator_delta_color(led_indicator_subsystem_t *subsystem, const int8_t dr, const int8_t dg, const int8_t db);
int led_indicator_deinit(led_indicator_subsystem_t *subsystem);