#include <cstdint>

#define BLACK_RGB urgb_u32(0, 0, 0)
#define RED_RGB urgb_u32(255, 0, 0)
#define GREEN_RGB urgb_u32(0, 255, 0)
#define BLUE_RGB urgb_u32(0, 0, 255)
#define WHITE_RGB urgb_u32(255, 255, 255)
#define AMBER_RGB urgb_u32(255, 90, 0)
#define TURQUOISE_RGB urgb_u32(0, 56, 255)
#define YELLOW_RGB urgb_u32(255, 255, 0)

enum ws2812_effect_t {
	SOLID,
	BLINK_FAST,
	BLINK_SLOW,
	BREATHE_FAST,
	BREATHE_SLOW,
};

uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);

uint32_t urgbw_u32(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

void ws2812_set_brightness(float brightness);

void ws2812_set(uint32_t pixel_grb, ws2812_effect_t effect = SOLID);

void ws2812_flash(uint32_t pixel_grb, uint32_t timeout_ms);

void ws2812_update();

void ws2812_init();
