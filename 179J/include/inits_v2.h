// nano_hw.h  (ATmega328P / Arduino Nano, bare-metal; no Arduino core)
// Uses only <avr/io.h> and <avr/interrupt.h>.
//
// FINAL PIN MAP:
// Actuator #1 (BTS7960-style):
//   D4 = R_EN, D7 = L_EN, D5 = RPWM (PWM), D6 = LPWM (PWM)
//   -> Timer0 provides PWM on D5(OC0B) and D6(OC0A)
//
// Actuator #2 (BTS7960-style):
//   D9  = R_EN (digital only), D12 = L_EN (digital only)
//   D3  = RPWM (PWM), D11 = LPWM (PWM)
//   -> Timer2 provides PWM on D3(OC2B) and D11(OC2A)
//
// Scheduler:
//   Exact 1.000 ms tick using Timer1 CTC:
//     Prescaler /64, OCR1A=249 => 250kHz clock, 250 counts = 1ms
//
// Notes:
// - Timer0 PWM frequency with /64 prescale is ~976 Hz (fast PWM 8-bit).
// - Timer2 PWM frequency with /64 prescale is ~976 Hz (fast PWM 8-bit).
// - All EN pins are just GPIO outputs (no PWM).
//
// Changes vs your previous header:
// 1) Timer0 init is "non-clobbering": we clear only the bits we use.
// 2) Timer1 init clears pending OCF1A flag before enabling interrupt.
// 3) Timer1 tick counter + ISR moved OUT of the header into a .c file in the
//    recommended layout below (multi-file safe). For single-file projects,
//    you can keep the "NANO_HW_IMPLEMENTATION" block below in exactly one TU.

#pragma once
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BIT(n) (1u << (n))

// =========================================================
// PIN DEFINITIONS
// =========================================================

// ----------------- Actuator 1 (PORTD) -----------------
// D4 = PD4
#define ACT1_REN_DDR   DDRD
#define ACT1_REN_PORT  PORTD
#define ACT1_REN_PIN   PD4

// D5 = PD5 (OC0B)
#define ACT1_RPWM_DDR  DDRD
#define ACT1_RPWM_PIN  PD5

// D6 = PD6 (OC0A)
#define ACT1_LPWM_DDR  DDRD
#define ACT1_LPWM_PIN  PD6

// D7 = PD7
#define ACT1_LEN_DDR   DDRD
#define ACT1_LEN_PORT  PORTD
#define ACT1_LEN_PIN   PD7

// ----------------- Actuator 2 -----------------
// D9  = PB1 (digital enable)
#define ACT2_REN_DDR   DDRB
#define ACT2_REN_PORT  PORTB
#define ACT2_REN_PIN   PB1

// D3  = PD3 (OC2B) RPWM
#define ACT2_RPWM_DDR  DDRD
#define ACT2_RPWM_PIN  PD3

// D11 = PB3 (OC2A) LPWM
#define ACT2_LPWM_DDR  DDRB
#define ACT2_LPWM_PIN  PB3

// D12 = PB4 (digital enable)
#define ACT2_LEN_DDR   DDRB
#define ACT2_LEN_PORT  PORTB
#define ACT2_LEN_PIN   PB4

// =========================================================
// PWM: TIMER0 for Actuator 1 (D5/D6)
// =========================================================
static inline void pwm_timer0_init(void) {
  // PWM pins output
  ACT1_RPWM_DDR |= BIT(ACT1_RPWM_PIN); // D5 OC0B
  ACT1_LPWM_DDR |= BIT(ACT1_LPWM_PIN); // D6 OC0A

  // --- Timer0 mode bits we care about ---
  // Fast PWM: WGM01=1, WGM00=1 (in TCCR0A)
  // Non-inverting OC0A/OC0B: COM0A1=1, COM0B1=1 (in TCCR0A)
  //
  // Non-clobbering approach:
  // - clear the relevant bitfields
  // - set the desired bits
  TCCR0A &= (uint8_t)~(BIT(WGM01) | BIT(WGM00) | BIT(COM0A1) | BIT(COM0A0) | BIT(COM0B1) | BIT(COM0B0));
  TCCR0A |= (uint8_t)(BIT(WGM01) | BIT(WGM00) | BIT(COM0A1) | BIT(COM0B1));

  // Prescaler bits are in TCCR0B: CS02:0
  TCCR0B &= (uint8_t)~(BIT(CS02) | BIT(CS01) | BIT(CS00));
  TCCR0B |= (uint8_t)(BIT(CS01) | BIT(CS00)); // /64

  OCR0A = 0; // D6 duty (OC0A)
  OCR0B = 0; // D5 duty (OC0B)
}

// =========================================================
// PWM: TIMER2 for Actuator 2 (D3/D11)  (OC2B + OC2A)
// =========================================================
static inline void pwm_timer2_init_dual(void) {
  // PWM pins output
  ACT2_RPWM_DDR |= BIT(ACT2_RPWM_PIN); // D3  OC2B
  ACT2_LPWM_DDR |= BIT(ACT2_LPWM_PIN); // D11 OC2A

  // Timer2: Fast PWM 8-bit, non-inverting on OC2A/OC2B
  // WGM21=1,WGM20=1; COM2A1=1; COM2B1=1
  TCCR2A &= (uint8_t)~(BIT(WGM21) | BIT(WGM20) | BIT(COM2A1) | BIT(COM2A0) | BIT(COM2B1) | BIT(COM2B0));
  TCCR2A |= (uint8_t)(BIT(WGM21) | BIT(WGM20) | BIT(COM2A1) | BIT(COM2B1));

  // Prescaler bits CS22:0 in TCCR2B
  TCCR2B &= (uint8_t)~(BIT(CS22) | BIT(CS21) | BIT(CS20));
  TCCR2B |= (uint8_t)(BIT(CS22)); // /64

  OCR2A = 0; // D11 duty
  OCR2B = 0; // D3  duty
}

// =========================================================
// BTS7960-STYLE ACTUATOR CONTROL (2 PWM + 2 EN)
// speed in [-255..255]
// =========================================================

static inline void actuator1_init(void) {
  // EN pins outputs
  ACT1_REN_DDR |= BIT(ACT1_REN_PIN);
  ACT1_LEN_DDR |= BIT(ACT1_LEN_PIN);

  // Enable HIGH
  ACT1_REN_PORT |= BIT(ACT1_REN_PIN);
  ACT1_LEN_PORT |= BIT(ACT1_LEN_PIN);

  pwm_timer0_init();
}

static inline void actuator2_init(void) {
  // EN pins outputs
  ACT2_REN_DDR |= BIT(ACT2_REN_PIN);
  ACT2_LEN_DDR |= BIT(ACT2_LEN_PIN);

  // Enable HIGH
  ACT2_REN_PORT |= BIT(ACT2_REN_PIN);
  ACT2_LEN_PORT |= BIT(ACT2_LEN_PIN);

  pwm_timer2_init_dual();
}

static inline void actuator1_set_speed(int16_t speed) {
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    OCR0B = (uint8_t)speed; // D5 RPWM (OC0B)
    OCR0A = 0;              // D6 LPWM (OC0A)
  } else if (speed < 0) {
    OCR0B = 0;
    OCR0A = (uint8_t)(-speed);
  } else {
    OCR0B = 0;
    OCR0A = 0;
  }
}

static inline void actuator2_set_speed(int16_t speed) {
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    OCR2B = (uint8_t)speed; // D3  RPWM (OC2B)
    OCR2A = 0;              // D11 LPWM (OC2A)
  } else if (speed < 0) {
    OCR2B = 0;
    OCR2A = (uint8_t)(-speed);
  } else {
    OCR2B = 0;
    OCR2A = 0;
  }
}

static inline void actuator1_enable(uint8_t en) {
  if (en) {
    ACT1_REN_PORT |= BIT(ACT1_REN_PIN);
    ACT1_LEN_PORT |= BIT(ACT1_LEN_PIN);
  } else {
    ACT1_REN_PORT &= (uint8_t)~BIT(ACT1_REN_PIN);
    ACT1_LEN_PORT &= (uint8_t)~BIT(ACT1_LEN_PIN);
    actuator1_set_speed(0);
  }
}

static inline void actuator2_enable(uint8_t en) {
  if (en) {
    ACT2_REN_PORT |= BIT(ACT2_REN_PIN);
    ACT2_LEN_PORT |= BIT(ACT2_LEN_PIN);
  } else {
    ACT2_REN_PORT &= (uint8_t)~BIT(ACT2_REN_PIN);
    ACT2_LEN_PORT &= (uint8_t)~BIT(ACT2_LEN_PIN);
    actuator2_set_speed(0);
  }
}

// =========================================================
// TIMER1 1ms TICK (DECLARATIONS)
// =========================================================
//
// Multi-file safe design:
// - In exactly ONE .c file, before including this header, define:
//     #define NANO_HW_IMPLEMENTATION
// - That .c file will contain the tick storage + ISR definition.
// - Everywhere else, you just include this header normally.

void sched_init_1ms(void);
uint32_t sched_ms(void);
void sched_delay_ms(uint16_t ms);

// =========================================================
// TOP-LEVEL INIT
// =========================================================
static inline void nano_init(void) {
  actuator1_init();
  actuator2_init();
  sched_init_1ms();
}

// =========================================================
// IMPLEMENTATION BLOCK (include in ONE .c file only)
// =========================================================

static volatile uint32_t g_ms_ticks = 0;

ISR(TIMER1_COMPA_vect) {
  g_ms_ticks++;
}

void sched_init_1ms(void) {
  uint8_t s = SREG;
  cli();

  g_ms_ticks = 0;

  // Stop and clear
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // CTC mode: WGM12=1
  TCCR1B |= BIT(WGM12);

  // TOP for 1ms @ 16MHz/64 = 250kHz => 250 counts => OCR1A=249
  OCR1A = 249;

  // Clear pending compare flag to avoid spurious first tick
  TIFR1 |= BIT(OCF1A);

  // Enable compare match A interrupt
  TIMSK1 |= BIT(OCIE1A);

  // Start timer with prescaler /64
  TCCR1B |= (uint8_t)(BIT(CS11) | BIT(CS10));

  SREG = s;
  sei();
}

uint32_t sched_ms(void) {
  uint32_t t;
  uint8_t s = SREG;
  cli();
  t = g_ms_ticks;
  SREG = s;
  return t;
}

void sched_delay_ms(uint16_t ms) {
  uint32_t start = sched_ms();
  while ((uint32_t)(sched_ms() - start) < ms) {;}
}