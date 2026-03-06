#include "led_indicator.hpp"
#include "led_strip_rmt.h"
#include <atomic>
#include <mutex>

int led_indicator_init(led_indicator_subsystem_t *subsystem, gpio_num_t gpio_num) {
	// initialize the led indicator subsystem safely
	if (subsystem == nullptr) return -1;
	subsystem->r.store(0, std::memory_order_release);
	subsystem->g.store(0, std::memory_order_release);
	subsystem->b.store(0, std::memory_order_release);
	led_strip_config_t strip_config = {
		.strip_gpio_num = gpio_num,
		.max_leds = 1,
		.led_pixel_format = LED_PIXEL_FORMAT_GRB,
		.led_model = LED_MODEL_WS2812
	};
	led_strip_rmt_config_t rmt_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 10000000,
		.mem_block_symbols = 64,
		.flags = {
			.with_dma = 0,
		}
	};
	led_strip_new_rmt_device(&strip_config, &rmt_config, &subsystem->indicator);
	led_strip_set_pixel(subsystem->indicator, 0, 0, 0, 0);
	led_strip_refresh(subsystem->indicator);
	return 0;
}
int led_indicator_set_color(led_indicator_subsystem_t *subsystem, uint8_t r, uint8_t g, uint8_t b) {
	// set absolute color value with mutex protection
	if (subsystem == nullptr) return -1;
	if (subsystem->indicator == nullptr) return -1;
	std::lock_guard<std::mutex> lock(subsystem->mutex);
	subsystem->r.store(r, std::memory_order_release);
	subsystem->g.store(g, std::memory_order_release);
	subsystem->b.store(b, std::memory_order_release);
	led_strip_set_pixel(subsystem->indicator, 0, r, g, b);
	led_strip_refresh(subsystem->indicator);
	return 0;
}
int led_indicator_delta_color(led_indicator_subsystem_t *subsystem, int8_t dr, int8_t dg, int8_t db) {
	// apply atomic delta changes with saturation arithmetic
	if (subsystem == nullptr) return -1;
	if (subsystem->indicator == nullptr) return -1;
	std::lock_guard<std::mutex> lock(subsystem->mutex);
	uint8_t current_r = subsystem->r.load(std::memory_order_acquire);
	uint8_t current_g = subsystem->g.load(std::memory_order_acquire);
	uint8_t current_b = subsystem->b.load(std::memory_order_acquire);
	int16_t new_r = (int16_t)current_r + dr;
	int16_t new_g = (int16_t)current_g + dg;
	int16_t new_b = (int16_t)current_b + db;
	if (new_r < 0) new_r = 0;
	if (new_r > 255) new_r = 255;
	if (new_g < 0) new_g = 0;
	if (new_g > 255) new_g = 255;
	if (new_b < 0) new_b = 0;
	if (new_b > 255) new_b = 255;
	subsystem->r.store((uint8_t)new_r, std::memory_order_release);
	subsystem->g.store((uint8_t)new_g, std::memory_order_release);
	subsystem->b.store((uint8_t)new_b, std::memory_order_release);
	led_strip_set_pixel(subsystem->indicator, 0, (uint8_t)new_r, (uint8_t)new_g, (uint8_t)new_b);
	led_strip_refresh(subsystem->indicator);
	return 0;
}
int led_indicator_deinit(led_indicator_subsystem_t *subsystem) {
	// safely turn off led and release resources
	if (subsystem == nullptr) return -1;
	if (subsystem->indicator == nullptr) return -1;
	std::lock_guard<std::mutex> lock(subsystem->mutex);
	led_strip_clear(subsystem->indicator);
	led_strip_refresh(subsystem->indicator);
	return 0;
}
