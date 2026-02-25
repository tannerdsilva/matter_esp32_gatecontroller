#include "led_indicator.hpp"
// rmt
#include "led_strip_rmt.h"

int led_indicator_init(led_indicator_subsystem_t *subsystem, gpio_num_t gpio_num) {
	// set up the led indicator
	led_strip_config_t strip_config = {
		.strip_gpio_num = gpio_num,
		.max_leds = 1,
		.led_pixel_format = LED_PIXEL_FORMAT_GRB,
		.led_model = LED_MODEL_WS2812
	};
	led_strip_rmt_config_t rmt_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 10000000, // 10MHz resolution (0.1us per tick)
		.mem_block_symbols = 64,
		.flags = {
			.with_dma = 0,
		}
	};
	led_strip_new_rmt_device(&strip_config, &rmt_config, &subsystem->indicator);
	led_strip_set_pixel(subsystem->indicator, 0, 0, 0, 0); // set no color initially
	led_strip_refresh(subsystem->indicator);
	return 0;
}
int led_indicator_set_color(led_indicator_subsystem_t *subsystem, uint8_t r, uint8_t g, uint8_t b) {
	led_strip_set_pixel(subsystem->indicator, 0, r, g, b);
	led_strip_refresh(subsystem->indicator);
	return 0;
}
int led_indicator_deinit(led_indicator_subsystem_t *subsystem) {
	led_strip_clear(subsystem->indicator);
	led_strip_refresh(subsystem->indicator);
	return 0;
}
