#include <EEPROM.h>
#include <avr/wdt.h>

// I/O pins
#define IN_AIL PINB0
#define IN_ELE PINB2
#define OUT_LEFT PINB3
#define OUT_RIGHT PINB4
#define OUT_LED PINB1

// F_CPU == 16000000L // 16 MHz
// one clock tick = 1/16 us
// prescaler = 64
// one timer tick = 1/16*64 = 4 us

#define SERVO_PULSE_MIN 1000
#define SERVO_PULSE_MAX 2000
#define SERVO_PULSE_WIDTH (SERVO_PULSE_MAX - SERVO_PULSE_MIN)
#define SERVO_PULSE_WIDTH_REVERSE(x) (SERVO_PULSE_WIDTH - min(SERVO_PULSE_WIDTH, (x)))

// === LED ===
inline void ledOff() {
  bitClear(PORTB, OUT_LED);
}
inline void ledOn() {
  bitSet(PORTB, OUT_LED);
}
void ledDelay(uint16_t ms) {
  while (ms >= 100) {
    wdt_reset();
    delay(100);
    ms -= 100;
  }
  if (ms) {
    wdt_reset();
    delay(ms);
  }
  wdt_reset();
}
void ledBlinkFast() {
  ledOn();
  ledDelay(200);
  ledOff();
  ledDelay(500);
}
void ledBlinkSlow() {
  ledOn();
  ledDelay(800);
  ledOff();
  ledDelay(500);
}

// === Ticks ===
#define TIMER0_PRESCALER 64
#define TIMER1_PRESCALER (TIMER0_PRESCALER * 256)
#define TICKS_500US (F_CPU / TIMER0_PRESCALER / 2000)
#define TICKS_250US (TICKS_500US / 2)
#define TICKS_1000US (TICKS_500US * 2)
#define TICKS_1500US (TICKS_500US * 3)
#define TICKS_2000US (TICKS_500US * 4)

#if TIMER0_PRESCALER == 64
#define TIMER0_PRESCALER_BITS (bit(CS01) | bit(CS00))
#elif TIMER0_PRESCALER == 8
#define TIMER0_PRESCALER_BITS (bit(CS01))
#else
#error "Unsupported TIMER0_PRESCALER"
#endif

#if TIMER1_PRESCALER == 16384
#define TIMER1_PRESCALER_BITS (bit(CS13) | bit(CS12) | bit(CS11) | bit(CS10))
#elif TIMER1_PRESCALER == 2048
#define TIMER1_PRESCALER_BITS (bit(CS13) | bit(CS12))
#else
#error "Unsupported TIMER1_PRESCALER"
#endif

inline uint16_t ticks() {
  uint8_t h1 = TCNT1;
  uint8_t l1 = TCNT0;
  uint8_t h2 = TCNT1;
  uint8_t l2 = TCNT0;
  return h1 == h2 ? ((h1 << 8) | l1) : ((h2 << 8) | l2);  // h1 == h2 -> no overflows
}
void ticksInit() {
  TCCR0A = 0;
  TCCR0B = TIMER0_PRESCALER_BITS;  // set timer0 prescaler
  TCCR1 = TIMER1_PRESCALER_BITS;   // set timer1 prescaler
  GTCCR = bit(PSR1) | bit(PSR0);   // sync timer0 and timer1 prescalers
  TCNT0 = 0;                       // reset timer0
  TCNT1 = 0;                       // reset timer1
}
/** Returns (a - b) accounting for overflow. */
inline uint16_t ticksDiff(uint16_t a, uint16_t b) {
  if (a >= b)
    return a - b;
  else
    return a + ((~b) + 1);  // a + (0x10000 - b)
}

// === RC Input ===
#define WAIT_AIL_ON 0
#define WAIT_AIL_OFF 1
#define WAIT_ELE_ON 2
#define WAIT_ELE_OFF 3

// === Calibration ===
uint16_t ailCenterPulseLen = TICKS_1500US;
uint16_t ailRatePulseLen = TICKS_500US;
uint16_t eleCenterPulseLen = TICKS_1500US;
uint16_t eleRatePulseLen = TICKS_500US;
int servoOffsetUs = 0;

#define EEPROM_VERSION 0x43

void calRead() {
  if (EEPROM.read(0) != EEPROM_VERSION)
    return;
  EEPROM.get(1, ailCenterPulseLen);
  EEPROM.get(3, ailRatePulseLen);
  EEPROM.get(5, eleCenterPulseLen);
  EEPROM.get(7, eleRatePulseLen);
  EEPROM.get(9, servoOffsetUs);
}
void calWrite() {
  EEPROM.put(1, ailCenterPulseLen);
  EEPROM.put(3, ailRatePulseLen);
  EEPROM.put(5, eleCenterPulseLen);
  EEPROM.put(7, eleRatePulseLen);
  EEPROM.put(9, servoOffsetUs);

  EEPROM.update(0, EEPROM_VERSION);
}

// === Main ===

inline void writeServos(uint16_t leftUs, uint16_t rightUs) {
  PORTB |= bit(OUT_LEFT);
  delayMicroseconds(leftUs);
  PORTB &= ~bit(OUT_LEFT);
  PORTB |= bit(OUT_RIGHT);
  delayMicroseconds(rightUs);
  PORTB &= ~bit(OUT_RIGHT);
}

uint16_t diffU16(uint16_t a, uint16_t b) {
  if (a >= b)
    return a - b;
  return b - a;
}

bool pulseLenClose(uint16_t a, uint16_t b) {
  return diffU16(a, b) * 4 < TICKS_500US;
}

#define CAL_DONE 0
#define CAL_CENTER 1
#define CAL_CENTER_LOOP 2
#define CAL_RATE 3
#define CAL_RATE_LOOP 4
#define CAL_SERVO_OFFSET_WAIT 6
#define CAL_SERVO_OFFSET 7
#define CAL_SERVO_OFFSET_LOOP 8

uint8_t calState = CAL_DONE;
uint16_t calTicks = 0;
bool firstUpdate = true;

void setup() {
  cli();                                                 // disable interrupts, we don't use them
  PORTB = 0;                                             // all pins off
  DDRB = bit(OUT_LEFT) | bit(OUT_RIGHT) | bit(OUT_LED);  // configure OUT_LEFT, OUT_RIGHT and OUT_LED as outputs, other pins as inputs
  calRead();
  wdt_enable(WDTO_500MS);
  ticksInit();
}

void loop() {
  uint8_t inFlags;
  uint16_t ailOnTick, ailPulseLen;
  uint16_t eleOnTick, elePulseLen;
  uint16_t leftOnUs, rightOnUs;
  int ailServoUs, eleServoUs;

  // reset watchdog
  wdt_reset();

  inFlags = bit(WAIT_AIL_ON) | bit(WAIT_ELE_ON);

  while(PINB & (bit(IN_AIL) | bit(IN_ELE))); // wait for both off

  while (inFlags) {
    if (inFlags & bit(WAIT_AIL_OFF)) {
      if (!(PINB & bit(IN_AIL))) {  // IN_AIL off
        ailPulseLen = ticksDiff(ticks(), ailOnTick);
        inFlags &= ~(bit(WAIT_AIL_OFF) | bit(WAIT_AIL_ON));
      }
    }
    if (inFlags & bit(WAIT_ELE_OFF)) {
      if (!(PINB & bit(IN_ELE))) {  // IN_ELE off
        elePulseLen = ticksDiff(ticks(), eleOnTick);
        inFlags &= ~(bit(WAIT_ELE_OFF) | bit(WAIT_ELE_ON));
      }
    }
    if (inFlags & bit(WAIT_ELE_ON)) {
      if (PINB & (1 << IN_ELE)) {  // IN_ELE on
        eleOnTick = ticks();
        inFlags ^= (bit(WAIT_ELE_ON) | bit(WAIT_ELE_OFF));
      }
    }
    if (inFlags & bit(WAIT_AIL_ON)) {
      if (PINB & bit(IN_AIL)) {  // IN_AIL on
        ailOnTick = ticks();
        inFlags ^= (bit(WAIT_AIL_ON) | bit(WAIT_AIL_OFF));  // clear WAIT_AIL_ON, set WAIT_AIL_OFF
      }
    }
  }

  if (ailPulseLen < TICKS_1000US - TICKS_250US || ailPulseLen > TICKS_2000US + TICKS_250US)
    return;

  uint16_t now = ticks();

  ailServoUs = map(ailPulseLen, ailCenterPulseLen - ailRatePulseLen, ailCenterPulseLen + ailRatePulseLen, -SERVO_PULSE_WIDTH, SERVO_PULSE_WIDTH);
  eleServoUs = map(elePulseLen, eleCenterPulseLen, eleCenterPulseLen + eleRatePulseLen, 0, SERVO_PULSE_WIDTH);
  leftOnUs = SERVO_PULSE_MIN + max(0, max(eleServoUs, ailServoUs)) + servoOffsetUs;
  rightOnUs = SERVO_PULSE_MIN + SERVO_PULSE_WIDTH_REVERSE(max(0, max(eleServoUs, -ailServoUs))) + servoOffsetUs;

  if (firstUpdate && elePulseLen < TICKS_1000US + TICKS_250US) {
    calState = CAL_CENTER;
    ledBlinkFast();
  }
  firstUpdate = false;

  if (calState != CAL_DONE) {
    switch (calState) {
      case CAL_CENTER:
        calTicks = 0;
        calState = CAL_CENTER_LOOP;
        break;

      case CAL_CENTER_LOOP:
        // calibrate stick center (ailCenterPulseLen, eleCenterPulseLen)
        if (pulseLenClose(ailPulseLen, TICKS_1500US) && pulseLenClose(elePulseLen, TICKS_1500US)) {                        // both sticks close to center
          if (calTicks && diffU16(ailPulseLen, ailCenterPulseLen) < 16 && diffU16(elePulseLen, eleCenterPulseLen) < 16) {  // sticks are steady
            if (ticksDiff(now, calTicks) > TICKS_2000US * 100) {                                                            // for at least 200 ms
              ailCenterPulseLen = ailPulseLen;
              eleCenterPulseLen = elePulseLen;
              calWrite();
              ledBlinkFast();
              calState = CAL_RATE;
            }
          } else {
            calTicks = now;
            ailCenterPulseLen = ailPulseLen;
            eleCenterPulseLen = elePulseLen;
          }
        } else {
          calTicks = 0;
        }
        delayMicroseconds(1000);
        return;
        break;

      case CAL_RATE:
        ailRatePulseLen = TICKS_250US;
        eleRatePulseLen = TICKS_250US;
        calState = CAL_RATE_LOOP;
        break;

      case CAL_RATE_LOOP:
        if (diffU16(ailPulseLen, ailCenterPulseLen) > ailRatePulseLen)
          ailRatePulseLen = diffU16(ailPulseLen, ailCenterPulseLen);
        if (diffU16(elePulseLen, eleCenterPulseLen) > eleRatePulseLen)
          eleRatePulseLen = diffU16(elePulseLen, eleCenterPulseLen);

        if (elePulseLen < TICKS_1000US + TICKS_250US) {
          calWrite();
          ledBlinkFast();
          calState = CAL_SERVO_OFFSET_WAIT;
        }
        delayMicroseconds(1000);
        return;
        break;

      case CAL_SERVO_OFFSET_WAIT:
        if (pulseLenClose(ailPulseLen, TICKS_1500US) && pulseLenClose(elePulseLen, TICKS_1500US)) {  // both sticks close to center
          calState = CAL_SERVO_OFFSET;
        }
        break;

      case CAL_SERVO_OFFSET:
        calTicks = now;
        calState = CAL_SERVO_OFFSET_LOOP;
        break;

      case CAL_SERVO_OFFSET_LOOP:
        if (ailPulseLen > TICKS_1500US + TICKS_250US) {  // > 3/4 -> inc
          if (ticksDiff(now, calTicks) > TICKS_2000US * 100) {
            calTicks = now;
            servoOffsetUs += 4;
          }
        } else if (ailPulseLen < TICKS_1000US + TICKS_250US) {  // < 1/4 -> dec
          if (ticksDiff(now, calTicks) > TICKS_2000US * 100) {
            calTicks = now;
            servoOffsetUs -= 4;
          }
        } else if (elePulseLen > TICKS_1500US + TICKS_250US) {
          servoOffsetUs = 0;
        } else if (elePulseLen < TICKS_1000US + TICKS_250US) {
          calWrite();
          ledBlinkFast();
          calState = CAL_DONE;
        }

        // move servos to initial position
        leftOnUs = SERVO_PULSE_MIN + servoOffsetUs;
        rightOnUs = SERVO_PULSE_MAX + servoOffsetUs;
        break;
    }
  }

  writeServos(leftOnUs, rightOnUs);
}
