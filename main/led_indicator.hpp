#pragma once
#include "driver/gpio.h"
#include "led_strip.h"
#include "led_strip_types.h"
#include <atomic>
#include <mutex>

struct led_indicator_subsystem {
	led_strip_handle_t indicator;
	std::atomic<uint8_t> r;
	std::atomic<uint8_t> g;
	std::atomic<uint8_t> b;
	std::mutex mutex;
};
typedef struct led_indicator_subsystem led_indicator_subsystem_t;

int led_indicator_init(led_indicator_subsystem_t *subsystem, gpio_num_t gpio_num);
int led_indicator_set_color(led_indicator_subsystem_t *subsystem, uint8_t r, uint8_t g, uint8_t b);
int led_indicator_delta_color(led_indicator_subsystem_t *subsystem, int8_t dr, int8_t dg, int8_t db);
int led_indicator_deinit(led_indicator_subsystem_t *subsystem);
