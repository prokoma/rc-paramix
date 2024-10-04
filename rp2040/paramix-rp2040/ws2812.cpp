/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ws2812.h"
#include "ws2812.pio.h"
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdlib.h>

using std::abs;

/**
 * NOTE:
 *  Take into consideration if your WS2812 is a RGB or RGBW variant.
 *
 *  If it is RGBW, you need to set IS_RGBW to true and provide 4 bytes per
 *  pixel (Red, Green, Blue, White) and use urgbw_u32().
 *
 *  If it is RGB, set IS_RGBW to false and provide 3 bytes per pixel (Red,
 *  Green, Blue) and use urgb_u32().
 *
 *  When RGBW is used with urgb_u32(), the White channel will be ignored (off).
 *
 */
#define IS_RGBW false

#define WS2812_PIN PICO_DEFAULT_WS2812_PIN

static inline void put_pixel(uint32_t pixel_grb) {
  pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

uint32_t urgbw_u32(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | ((uint32_t)(w) << 24) |
         (uint32_t)(b);
}

static uint32_t set_brt(uint32_t rgbw, float brt) {
  uint8_t r = (uint8_t)(rgbw >> 8);
  uint8_t g = (uint8_t)(rgbw >> 16);
  uint8_t b = (uint8_t)rgbw;
  uint8_t w = (uint8_t)(rgbw >> 24);
  return urgbw_u32(r * brt, g * brt, b * brt, w * brt);
}

static absolute_time_t g_last_update = nil_time;

static uint64_t g_flash_color = 0;
static absolute_time_t g_flash_until = nil_time;

static float g_brightness = 1.0f;

static uint32_t g_color = 0;
static uint32_t g_effect = SOLID;

#define PERIOD_FAST 500
#define PERIOD_SLOW 1500

void ws2812_update() {
  absolute_time_t now = get_absolute_time();
  uint32_t now_ms = to_ms_since_boot(now);
  int64_t us_since_last_update = absolute_time_diff_us(g_last_update, now);

  if (us_since_last_update >= 10000) {
    uint32_t color = g_color;
    float brightness = g_brightness;
    switch (g_effect) {
    case SOLID:
      break;
    case BLINK_FAST:
      if (now_ms % PERIOD_FAST > PERIOD_FAST / 2)
        brightness = 0.0f;
      break;
    case BLINK_SLOW:
      if (now_ms % PERIOD_SLOW > PERIOD_SLOW / 2)
        brightness = 0.0f;
      break;
    case BREATHE_FAST:
      brightness *= abs((int)(now_ms % PERIOD_FAST) - PERIOD_FAST / 2) / ((float)(PERIOD_FAST / 2));
      break;
    case BREATHE_SLOW:
      brightness *= abs((int)(now_ms % PERIOD_SLOW) - PERIOD_SLOW / 2) / ((float)(PERIOD_SLOW / 2));
      break;
    }
    if(absolute_time_diff_us(now, g_flash_until) > 0) {
      color = g_flash_color;
      brightness = g_brightness;
    }
    color = set_brt(color, brightness);
    put_pixel(color);
    g_last_update = now;
  }
}

void ws2812_set_brightness(float brightness) {
  g_brightness = brightness;
}

void ws2812_set(uint32_t pixel_grb, ws2812_effect_t effect) {
  g_color = pixel_grb;
  g_effect = effect;
}

void ws2812_flash(uint32_t pixel_grb, uint32_t timeout_ms) {
  g_flash_color = pixel_grb;
  g_flash_until = make_timeout_time_ms(timeout_ms);
}

void ws2812_init() {
  // todo get free sm
  PIO pio = pio0;
  int sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);

  ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
}
