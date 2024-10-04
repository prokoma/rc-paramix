#include <EEPROM.h>
#include <avr/wdt.h>

// timer0 = ticks()
// timer1 = RC Input

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

#define TICKS_1000US (TICKS_500US * 2)
#define TICKS_1500US (TICKS_500US * 3)
#define TICKS_2000US (TICKS_500US * 4)

#define TICKS_250US (TICKS_500US / 2)
#define TICKS_4000US (TICKS_500US * 8)

#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000
#define SERVO_PULSE_WIDTH (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
#define SERVO_PULSE_REVERSE(x) (SERVO_PULSE_WIDTH - min(SERVO_PULSE_WIDTH, (x)))

// === LED ===
inline void ledOff() {
  PORTB &= ~(1 << OUT_LED);
}
inline void ledOn() {
  PORTB |= (1 << OUT_LED);
}
void ledInit() {
  ledOff();
  DDRB |= (1 << OUT_LED);  // enable LED pin
}
void ledDelay(uint16_t ms) {
  while(ms >= 100) {
    wdt_reset();
    delay(100);
    ms -= 100;
  }
  if(ms) {
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
#define TICKS_PRESCALER 64
#if TICKS_PRESCALER == 64
#define TICKS_PRESCALER_BITS ((1 << CS01) | (1 << CS00))
#elif TICKS_PRESCALER == 8
#define TICKS_PRESCALER_BITS ((1 << CS01))
#else
#error "Invalid prescaler"
#endif
#define TICKS_500US (F_CPU / TICKS_PRESCALER / 2000)

volatile uint8_t ticksHiByte = 0;  // overflows every 64 * 256 * 256 = 4194304 ticks = ~254200 us

ISR(TIMER0_OVF_vect) {
  // called every 64 * 256 = 16384 ticks = ~992 us
  ticksHiByte++;
}
inline uint16_t ticksIsr() {
  uint8_t _ticksHiByte = ticksHiByte;  // copy to register
  uint16_t res = (_ticksHiByte << 8) | TCNT0;
  if (TIFR & (1 << TOV0)) {                   // there is a pending overflow interrupt
    res = ((_ticksHiByte + 1) << 8) | TCNT0;  // read TCNT0 again
  }
  return res;
}
uint16_t ticks() {
  uint8_t oldSREG = SREG;
  cli();
  uint16_t res = ticksIsr();
  SREG = oldSREG;
  return res;
}
void ticksInit() {
  cli();
  TCCR0A = 0;
  TCCR0B = TICKS_PRESCALER_BITS;  // set timer0 prescaler
  TCNT0 = 0;                      // reset timer0
  TIMSK |= (1 << TOIE0);          // enable timer0 overflow interrupt
  TIFR |= (1 << TOV0);            // clear timer0 overflow interrupt flag
}
inline uint16_t ticksDiff(uint16_t a, uint16_t b) {
  if (a >= b)
    return a - b;
  else
    return a + ((~b) + 1);  // a + (0x10000 - b)
}

// === RC Input ===
volatile uint8_t inFlags = 0;
#define AIL_ON 0
#define ELE_ON 1

volatile uint16_t ailOnTick = 0;
volatile uint16_t ailPulseLen = 0;
volatile uint16_t eleOnTick = 0;
volatile uint16_t elePulseLen = 0;

ISR(PCINT0_vect) {
  // called when IN_AIL or IN_ELE change state, interrupts are automatically disabled
  uint16_t now = ticksIsr();
  if (inFlags & (1 << AIL_ON)) {
    if (!(PINB & (1 << IN_AIL))) {  // IN_AIL turned off
      ailPulseLen = ticksDiff(now, ailOnTick);
      inFlags &= ~(1 << AIL_ON);
    }
  } else {
    if (PINB & (1 << IN_AIL)) {  // IN_AIL turned on
      ailOnTick = now;
      ailPulseLen = 0;
      inFlags |= (1 << AIL_ON);
    }
  }
  if (inFlags & (1 << ELE_ON)) {
    if (!(PINB & (1 << IN_ELE))) {  // IN_ELE turned off
      elePulseLen = ticksDiff(now, eleOnTick);
      inFlags &= ~(1 << ELE_ON);
    }
  } else {
    if (PINB & (1 << IN_ELE)) {  // IN_ELE turned on
      eleOnTick = now;
      elePulseLen = 0;
      inFlags |= (1 << ELE_ON);
    }
  }
}
void rcInInit() {
  cli();
  PCMSK = (1 << IN_AIL) | (1 << IN_ELE);  // configure pin change interrupts for IN_AIL and IN_ELE
  GIMSK = (1 << PCIE);                    // enable pin change interrupts, disable INT0
  GIFR |= (1 << PCIF);                    // clear pin change interrupt flag
}

// === Calibration ===
uint16_t ailCenterPulseLen = TICKS_1500US;
uint16_t ailRatePulseLen = TICKS_500US;
uint16_t eleCenterPulseLen = TICKS_1500US;
uint16_t eleRatePulseLen = TICKS_500US;
int servoOffsetUs = 0;

void calRead() {
  if (EEPROM.read(0) != 0x43)
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

  EEPROM.update(0, 0x42);
}

// === Watchdog ===
void watchDogInit() {
  wdt_enable(WDTO_500MS);  
}

// === Main ===

void writeServo(uint16_t leftUs, uint16_t rightUs) {
  cli();
  PORTB |= (1 << OUT_LEFT);
  delayMicroseconds(leftUs);
  PORTB &= ~(1 << OUT_LEFT);
  PORTB |= (1 << OUT_RIGHT);
  delayMicroseconds(rightUs);
  PORTB &= ~(1 << OUT_RIGHT);

  ailPulseLen = 0;  // reset lengths so we can detect lost signal
  elePulseLen = 0;
  sei();

  delayMicroseconds(16000 - leftUs - rightUs);
}

bool pulseLenClose(int a, int b) {
  return abs(a - b) * 4 < TICKS_500US;
}

bool syncWithRx(uint16_t _ailOnTick, uint16_t _ailPulseLen, uint16_t _eleOnTick, uint16_t _elePulseLen, uint16_t _now) {
  // sync with the RC receiver - wait after both pulses are timed

  if (_ailPulseLen == 0 || _elePulseLen == 0) {  // no signal or in the middle of a pulse
    delayMicroseconds(1000);
    return false;
  }

  int ailOnAge = ticksDiff(_now, _ailOnTick);
  int eleOnAge = ticksDiff(_now, _eleOnTick);

  if (abs(ailOnAge - eleOnAge) > TICKS_1000US * 10) {  // pulses are from different cycles
    delayMicroseconds(1000);
    return false;
  }

  return true;
}

#define CAL_WAIT_FOR_CENTER1 0
#define CAL_WAIT_FOR_CENTER2 1
#define CAL_WAIT_FOR_CENTER3 2
#define CAL_WAIT_FOR_CENTER4 3
#define CAL_WAIT_FOR_CENTER5 4
#define CAL_SAVE_CENTER 5
#define CAL_SERVO_OFFSET 6
#define CAL_SAVE_SERVO_OFFSET 7
#define CAL_DONE 8

uint8_t calState = CAL_DONE;
uint16_t calTicks = 0;
bool firstUpdate = true;

uint16_t lastServoWrite = 0;

void setup() {
  PORTB = 0;                                  // all pins off
  DDRB = (1 << OUT_LEFT) | (1 << OUT_RIGHT);  // configure OUT_LEFT and OUT_RIGHT as outputs, other pins as inputs
  ledInit();
  calRead();
  watchDogInit();
  cli();
  ticksInit();
  rcInInit();
  sei();  // enable interrupts
}

void loop() {
  uint16_t _ailOnTick, _ailPulseLen;
  uint16_t _eleOnTick, _elePulseLen;
  uint16_t _now;
  uint16_t leftOnUs, rightOnUs;
  int ailServoUs, eleServoUs;

  // reset watchdog
  wdt_reset();

  // copy timing data into local variables - split critical sections so we don't lose precision
  cli();
  _now = ticksIsr();
  sei();
  asm("nop");
  cli();
  _ailOnTick = ailOnTick;
  _ailPulseLen = ailPulseLen;
  sei();
  asm("nop");
  cli();
  _eleOnTick = eleOnTick;
  _elePulseLen = elePulseLen;
  sei();

  if (!syncWithRx(_ailOnTick, _ailPulseLen, _eleOnTick, _elePulseLen, _now)) {
    ledOn();
    return;
  }

  ledOff();

  ailServoUs = map(((int)_ailPulseLen) - ((int)ailCenterPulseLen) + TICKS_1500US, TICKS_1000US, TICKS_2000US, -SERVO_PULSE_WIDTH, SERVO_PULSE_WIDTH);
  eleServoUs = map(((int)_elePulseLen) - ((int)eleCenterPulseLen) + TICKS_1500US, TICKS_1500US, TICKS_2000US, 0, SERVO_PULSE_WIDTH);
  leftOnUs = MIN_PULSE_WIDTH + max(0, max(eleServoUs, ailServoUs)) + servoOffsetUs;
  rightOnUs = MIN_PULSE_WIDTH + SERVO_PULSE_REVERSE(max(0, max(eleServoUs, -ailServoUs))) + servoOffsetUs;

  if (firstUpdate && _elePulseLen < TICKS_1000US + TICKS_250US) {
    calState = CAL_WAIT_FOR_CENTER1;
    ledBlinkFast();
  }
  firstUpdate = false;

  if (calState != CAL_DONE) {
    // avr-gcc supports max 7 case statements in switch()
    if (calState <= CAL_WAIT_FOR_CENTER5) {
      if (pulseLenClose(_ailPulseLen, TICKS_1500US) && pulseLenClose(_elePulseLen, TICKS_1500US)) {
        if (!calTicks) {
          calTicks = _now;
        } else if (ticksDiff(_now, calTicks) > TICKS_2000US * 100) {
          calTicks = _now;
          calState++;
        }
      } else {
        calTicks = 0;
        calState = CAL_WAIT_FOR_CENTER1;
      }
      return;
    }

    switch (calState) {
      case CAL_SAVE_CENTER:
        ailCenterPulseLen = _ailPulseLen;
        eleCenterPulseLen = _elePulseLen;
        calWrite();
        ledBlinkFast();
        calTicks = _now;
        calState = CAL_SERVO_OFFSET;
        break;

      case CAL_SERVO_OFFSET:
        if (_ailPulseLen > TICKS_1500US + TICKS_250US) {  // > 3/4 -> inc
          if (ticksDiff(_now, calTicks) > TICKS_2000US * 100) {
            calTicks = _now;
            servoOffsetUs += 4;
          }
        } else if (_ailPulseLen < TICKS_1000US + TICKS_250US) {  // < 1/4 -> dec
          if (ticksDiff(_now, calTicks) > TICKS_2000US * 100) {
            calTicks = _now;
            servoOffsetUs -= 4;
          }
        } else if (_elePulseLen < TICKS_1000US + TICKS_250US) {
          calState = CAL_SAVE_SERVO_OFFSET;
        } else if (_elePulseLen > TICKS_1500US + TICKS_250US) {
          servoOffsetUs = 0;
        }

        // move servos to initial position
        leftOnUs = MIN_PULSE_WIDTH + servoOffsetUs;
        rightOnUs = MAX_PULSE_WIDTH + servoOffsetUs;
        break;

      case CAL_SAVE_SERVO_OFFSET:
        calWrite();
        ledBlinkFast();
        calState = CAL_DONE;
        break;
    }
  }

  writeServo(leftOnUs, rightOnUs);
}
