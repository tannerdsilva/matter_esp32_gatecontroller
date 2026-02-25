#pragma once
#include "driver/gpio.h"
#include "led_strip.h"
#include "led_strip_types.h"

struct led_indicator_subsystem {
	led_strip_handle_t indicator;
};
typedef struct led_indicator_subsystem led_indicator_subsystem_t;

int led_indicator_init(led_indicator_subsystem_t *subsystem, gpio_num_t gpio_num);
int led_indicator_set_color(led_indicator_subsystem_t *subsystem, uint8_t r, uint8_t g, uint8_t b);
int led_indicator_deinit(led_indicator_subsystem_t *subsystem);
