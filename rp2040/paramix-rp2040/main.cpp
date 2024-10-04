#include <cassert>
#include <cstdint>
#include <hardware/clocks.h>
#include <hardware/flash.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/regs/addressmap.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
#include <ios>
#include <pico/stdio.h>
#include <pico/time.h>

#include "ws2812.h"

#define IN_AIL 9
#define IN_ELE 10

#define OUT_LEFT 11
#define OUT_RIGHT 13

#define CENTER_DEADZONE_US 3

// Utility functions

template <typename T> static T max(T a, T b) { return a > b ? a : b; }

template <typename T> static T min(T a, T b) { return a < b ? a : b; }

template <typename T> static T abs_diff(T a, T b) {
  return max(a, b) - min(a, b);
}

static int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
                   int32_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Servo constants

#define SERVO_PULSE_MIN 1000
#define SERVO_PULSE_MAX 2000
#define SERVO_PULSE_MID ((SERVO_PULSE_MIN + SERVO_PULSE_MAX) / 2)
#define SERVO_PULSE_14 1250
#define SERVO_PULSE_34 1750
#define SERVO_PULSE_LEN (SERVO_PULSE_MAX - SERVO_PULSE_MIN)

static uint32_t servo_pulse_len_reverse(uint32_t us) {
  return SERVO_PULSE_LEN - us;
}

static bool is_valid_pulse_len(uint32_t len) { return len > 750 && len < 2250; }

static bool servo_near(uint32_t a, uint32_t b) { return abs_diff(a, b) < 125; }

// Calibration data management

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t servo_offset_us;
  uint32_t ail_center_us;
  uint32_t ail_rate_us;
  uint32_t ele_center_us;
  uint32_t ele_rate_us;
} mixer_cal_data_t;

#define CAL_DATA_MAGIC ((uint32_t)42)
#define CAL_DATA_VERSION 2
#define CAL_DATA_OFFS (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE) // last sector
static mixer_cal_data_t g_cal_data = {
    // defaults
    .servo_offset_us = 0,
    .ail_center_us = SERVO_PULSE_MID,
    .ail_rate_us = SERVO_PULSE_LEN / 4,
    .ele_center_us = SERVO_PULSE_MID,
    .ele_rate_us = SERVO_PULSE_LEN / 4,
};

static bool cal_read() {
  mixer_cal_data_t *flash_data = (mixer_cal_data_t *)(XIP_BASE + CAL_DATA_OFFS);

  if (flash_data->magic == CAL_DATA_MAGIC &&
      flash_data->version == CAL_DATA_VERSION) {
    g_cal_data = *flash_data;
    return true;
  }
  return false;
}
static void cal_write() {
  g_cal_data.magic = CAL_DATA_MAGIC;
  g_cal_data.version = CAL_DATA_VERSION;

  uint ints = save_and_disable_interrupts();
  flash_range_erase(CAL_DATA_OFFS, FLASH_SECTOR_SIZE);
  flash_range_program(CAL_DATA_OFFS, (uint8_t *)&g_cal_data,
                      sizeof(g_cal_data));
  restore_interrupts(ints);
}

volatile uint64_t ail_pulse_start = 0;
volatile uint32_t ail_us = 0;

volatile uint64_t ele_pulse_start = 0;
volatile uint32_t ele_us = 0;

static void rc_irq_handler() {
  uint64_t now = to_us_since_boot(get_absolute_time());
  uint32_t ail_events = gpio_get_irq_event_mask(IN_AIL);
  gpio_acknowledge_irq(IN_AIL, ail_events);
  if (ail_events & GPIO_IRQ_EDGE_RISE) {
    ail_pulse_start = now;
  }
  if (ail_events & GPIO_IRQ_EDGE_FALL) {
    ail_us = now - ail_pulse_start;
  }
  uint32_t ele_events = gpio_get_irq_event_mask(IN_ELE);
  gpio_acknowledge_irq(IN_ELE, ele_events);
  if (ele_events & GPIO_IRQ_EDGE_RISE) {
    ele_pulse_start = now;
  }
  if (ele_events & GPIO_IRQ_EDGE_FALL) {
    ele_us = now - ele_pulse_start;
  }
}

static void rc_in_init(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_IN);
  gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_add_raw_irq_handler(gpio, rc_irq_handler);
}

static void servo_out_init(uint gpio) {
  pwm_set_clkdiv_int_frac(pwm_gpio_to_slice_num(gpio), 100, 0);
  pwm_set_wrap(pwm_gpio_to_slice_num(gpio), 20000);
  pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), pwm_gpio_to_channel(gpio), 0);
  pwm_set_counter(pwm_gpio_to_slice_num(gpio), 0);
  gpio_set_function(gpio, GPIO_FUNC_PWM);
}

static void write_servos(uint32_t left_us, uint32_t right_us) {
  pwm_set_chan_level(pwm_gpio_to_slice_num(OUT_LEFT),
                     pwm_gpio_to_channel(OUT_LEFT), left_us);
  pwm_set_chan_level(pwm_gpio_to_slice_num(OUT_RIGHT),
                     pwm_gpio_to_channel(OUT_RIGHT), right_us);
}

enum cal_state_t {
  CAL_DONE,
  CAL_ENTER,
  CAL_SAVE,
  CAL_CENTER,
  CAL_CENTER_LOOP,
  CAL_RATE,
  CAL_RATE_LOOP,
  CAL_SERVO_OFFSET,
  CAL_SERVO_OFFSET_LOOP,
};

static cal_state_t g_cal_state = CAL_DONE;
static uint64_t g_cal_us;
static uint64_t g_cal_count = 0;
static uint64_t g_cal_ail_center_us_sum = 0;
static uint64_t g_cal_ele_center_us_sum = 0;

static cal_state_t cal_update(cal_state_t state, uint64_t now, uint32_t ail_us,
                              uint32_t ele_us) {
  switch (state) {
  case CAL_ENTER:
    return CAL_CENTER;
  case CAL_SAVE:
    ws2812_flash(GREEN_RGB, 500);
    cal_write();
    return CAL_DONE;
  case CAL_DONE:
    return CAL_DONE;

  case CAL_CENTER:
    ws2812_set(TURQUOISE_RGB);
    g_cal_us = now;
    g_cal_count = g_cal_ail_center_us_sum = g_cal_ele_center_us_sum = 0;
    return CAL_CENTER_LOOP;
  case CAL_CENTER_LOOP:
    if (servo_near(ail_us, SERVO_PULSE_MID) &&
        servo_near(ele_us, SERVO_PULSE_MID)) {
      if (now - g_cal_us >
          500 * 1000) { // compute avg after 500 ms (for 250 ms)
        g_cal_data.ail_center_us = g_cal_ail_center_us_sum / g_cal_count;
        g_cal_data.ele_center_us = g_cal_ele_center_us_sum / g_cal_count;
        return CAL_RATE;
      } else if (now - g_cal_us > 250 * 1000) { // wait 250 ms for the stick to
                                                // be really in the center
        g_cal_count++;
        g_cal_ail_center_us_sum += ail_us;
        g_cal_ele_center_us_sum += ele_us;
      }
    } else {
      g_cal_us = now;
      g_cal_count = g_cal_ail_center_us_sum = g_cal_ele_center_us_sum = 0;
    }
    return CAL_CENTER_LOOP;

  case CAL_RATE:
    if (servo_near(ele_us, SERVO_PULSE_MID)) { // wait for ele center
      ws2812_set(TURQUOISE_RGB, BREATHE_FAST);
      g_cal_data.ail_rate_us = g_cal_data.ele_rate_us =
          SERVO_PULSE_LEN / 4; // reset to default (small) rates
      return CAL_RATE_LOOP;
    }
    return CAL_RATE;
  case CAL_RATE_LOOP:
    g_cal_data.ail_rate_us = max(g_cal_data.ail_rate_us,
                                 abs_diff(ail_us, (uint32_t)SERVO_PULSE_MID));
    g_cal_data.ele_rate_us = max(g_cal_data.ele_rate_us,
                                 abs_diff(ele_us, (uint32_t)SERVO_PULSE_MID));
    if (ele_us < SERVO_PULSE_14) { // ele up -> save
      return CAL_SERVO_OFFSET;
    }
    return CAL_RATE_LOOP;

  case CAL_SERVO_OFFSET:
    if (servo_near(ele_us, SERVO_PULSE_MID)) { // wait for ele center
      ws2812_set(TURQUOISE_RGB, BREATHE_SLOW);
      g_cal_us = now;
      return CAL_SERVO_OFFSET_LOOP;
    }
    return CAL_SERVO_OFFSET;
  case CAL_SERVO_OFFSET_LOOP:
    if (ail_us > SERVO_PULSE_34) {
      if (now - g_cal_us > 200 * 1000) {
        g_cal_us = now;
        g_cal_data.servo_offset_us += 2;
      }
    } else if (ail_us < SERVO_PULSE_14) {
      if (now - g_cal_us > 200 * 1000) {
        g_cal_us = now;
        g_cal_data.servo_offset_us -= 2;
      }
    } else if (ele_us > SERVO_PULSE_34) { // ele down -> reset
      g_cal_data.servo_offset_us = 0;
    } else if (ele_us < SERVO_PULSE_14) { // ele up -> save
      return CAL_SAVE;
    }

    // preview initial servo position
    write_servos(SERVO_PULSE_MIN + g_cal_data.servo_offset_us,
                 SERVO_PULSE_MAX + g_cal_data.servo_offset_us);
    return CAL_SERVO_OFFSET_LOOP;
  }

  return state; // unreachable
}

static void normal_update(uint64_t now, uint32_t ail_us, uint32_t ele_us) {
  // if sticks are near center, treat them as exactly in the center to stop
  // servo jitter
  if (abs_diff(ail_us, g_cal_data.ail_center_us) <= CENTER_DEADZONE_US)
    ail_us = g_cal_data.ail_center_us;
  if (abs_diff(ele_us, g_cal_data.ele_center_us) <= CENTER_DEADZONE_US)
    ele_us = g_cal_data.ele_center_us;

  int32_t ail_contrib = map(
    ail_us,
    g_cal_data.ail_center_us - g_cal_data.ail_rate_us,
    g_cal_data.ail_center_us + g_cal_data.ail_rate_us,
    -SERVO_PULSE_LEN,
    SERVO_PULSE_LEN
  );
  int32_t ele_contrib = map(
    ele_us,
    g_cal_data.ele_center_us,
    g_cal_data.ele_center_us + g_cal_data.ele_rate_us,
    0,
    SERVO_PULSE_LEN
  );

  uint32_t left_us  = SERVO_PULSE_MIN + max((int32_t)0, max(ele_contrib, ail_contrib)) + g_cal_data.servo_offset_us;
  uint32_t right_us = SERVO_PULSE_MIN + servo_pulse_len_reverse(max((int32_t)0, max(ele_contrib, -ail_contrib))) + g_cal_data.servo_offset_us;

  ws2812_set(BLACK_RGB);
  write_servos(left_us, right_us);
}

static void update(uint64_t now, uint32_t ail_us, uint32_t ele_us,
                   bool first_update) {
  if (first_update && ele_us < SERVO_PULSE_MIN + 250) {
    g_cal_state = CAL_ENTER;
  }

  if (g_cal_state != CAL_DONE) {
    g_cal_state = cal_update(g_cal_state, now, ail_us, ele_us);
  } else {
    normal_update(now, ail_us, ele_us);
  }
}

int main() {
  set_sys_clock_khz(100000, true);

  watchdog_enable(500, true);

  // init IN_AIL & IN_ELE
  rc_in_init(IN_AIL);
  rc_in_init(IN_ELE);

  // enable pin change interrupts
  irq_set_enabled(IO_IRQ_BANK0, true);

  // init OUT_LEFT & OUT_RIGHT
  servo_out_init(OUT_LEFT);
  servo_out_init(OUT_RIGHT);

  // start OUT_LEFT and 4 ms later OUT_RIGHT to reduce current spikes
  pwm_set_enabled(pwm_gpio_to_slice_num(OUT_LEFT), true);
  sleep_us(4000);
  pwm_set_enabled(pwm_gpio_to_slice_num(OUT_RIGHT), true);

  // init LED
  ws2812_init();
  ws2812_set_brightness(0.5);
  ws2812_set(RED_RGB);

  // read calibration data from flash
  cal_read();

  bool first_update = true;
  while (true) {
    if (is_valid_pulse_len(ail_us) && is_valid_pulse_len(ele_us)) {
      uint64_t now = to_us_since_boot(get_absolute_time());
      update(now, ail_us, ele_us, first_update);
      first_update = false;
    } else {
      ws2812_set(RED_RGB);
    }
    ws2812_update();
    watchdog_update();
    sleep_ms(1);
  }
}
