// nano_hw.h  (ATmega328P / Arduino Nano, bare-metal; no Arduino core)
// Uses only <avr/io.h> and <avr/interrupt.h>.
//
// FINAL PIN MAP (per your latest change):
// Actuator #1 (BTS7960-style):
//   D4 = R_EN, D7 = L_EN, D5 = RPWM (PWM), D6 = LPWM (PWM)
//   -> Timer0 provides PWM on D5(OC0B) and D6(OC0A)
//
// Actuator #2 (BTS7960-style) UPDATED to free Timer1:
//   D9  = R_EN (digital only), D12 = L_EN (digital only)
//   D3  = RPWM (PWM), D11 = LPWM (PWM)
//   -> Timer2 provides PWM on D3(OC2B) and D11(OC2A)
//
// Sensors:
//   Two BH1750 on I2C: A4(SDA), A5(SCL) shared
//     BH1750 #1 ADDR=GND -> 0x23
//     BH1750 #2 ADDR=VCC -> 0x5C
//
// Scheduler:
//   Exact 1.000 ms tick using Timer1 CTC:
//     Prescaler /64, OCR1A=249 => 250kHz clock, 250 counts = 1ms
//
// Notes:
// - Timer0 PWM frequency with /64 prescale is ~976 Hz (fast PWM 8-bit).
// - Timer2 PWM frequency with /64 prescale is ~976 Hz (fast PWM 8-bit).
// - All EN pins are just GPIO outputs (no PWM).
// - If you need different PWM freq, change prescalers carefully.

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

// ----------------- Actuator 2 (UPDATED) -----------------
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

// ----------------- I2C pins (PORTC) -----------------
// A4 = PC4 (SDA), A5 = PC5 (SCL)
#define I2C_SDA_PIN    PC4
#define I2C_SCL_PIN    PC5

// =========================================================
// PWM: TIMER0 for Actuator 1 (D5/D6)
// =========================================================
static inline void pwm_timer0_init(void) {
  // PWM pins output
  ACT1_RPWM_DDR |= BIT(ACT1_RPWM_PIN); // D5 OC0B
  ACT1_LPWM_DDR |= BIT(ACT1_LPWM_PIN); // D6 OC0A

  // Timer0: Fast PWM 8-bit, non-inverting on OC0A/OC0B
  TCCR0A = BIT(WGM01) | BIT(WGM00) | BIT(COM0A1) | BIT(COM0B1);
  TCCR0B = BIT(CS01) | BIT(CS00); // prescaler /64

  OCR0A = 0;
  OCR0B = 0;
}

// =========================================================
// PWM: TIMER2 for Actuator 2 (D3/D11)  (OC2B + OC2A)
// =========================================================
static inline void pwm_timer2_init_dual(void) {
  // PWM pins output
  ACT2_RPWM_DDR |= BIT(ACT2_RPWM_PIN); // D3  OC2B
  ACT2_LPWM_DDR |= BIT(ACT2_LPWM_PIN); // D11 OC2A

  // Timer2: Fast PWM 8-bit, non-inverting on OC2A/OC2B
  TCCR2A = BIT(WGM21) | BIT(WGM20) | BIT(COM2A1) | BIT(COM2B1);
  TCCR2B = BIT(CS22); // prescaler /64

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
// I2C (TWI) MASTER (minimal)
// =========================================================

static inline void twi_init_100khz(void) {
  // Let TWI control the pins; disable internal pullups (external pullups expected)
  DDRC  &= (uint8_t)~(BIT(I2C_SDA_PIN) | BIT(I2C_SCL_PIN));
  PORTC &= (uint8_t)~(BIT(I2C_SDA_PIN) | BIT(I2C_SCL_PIN));

  TWSR = 0x00; // prescaler = 1
  TWBR = 72;   // ~100kHz @ 16MHz
  TWCR = BIT(TWEN);
}

static inline uint8_t twi_start(uint8_t addr7, uint8_t read) {
  TWCR = BIT(TWINT) | BIT(TWSTA) | BIT(TWEN);
  while (!(TWCR & BIT(TWINT))) {;}

  TWDR = (uint8_t)((addr7 << 1) | (read ? 1 : 0));
  TWCR = BIT(TWINT) | BIT(TWEN);
  while (!(TWCR & BIT(TWINT))) {;}

  uint8_t status = (uint8_t)(TWSR & 0xF8);
  return (status == (read ? 0x40 : 0x18)); // SLA+R/W ACK
}

static inline void twi_stop(void) {
  TWCR = BIT(TWINT) | BIT(TWEN) | BIT(TWSTO);
}

static inline uint8_t twi_write(uint8_t data) {
  TWDR = data;
  TWCR = BIT(TWINT) | BIT(TWEN);
  while (!(TWCR & BIT(TWINT))) {;}
  return ((TWSR & 0xF8) == 0x28); // data ACK
}

static inline uint8_t twi_read_ack(void) {
  TWCR = BIT(TWINT) | BIT(TWEN) | BIT(TWEA);
  while (!(TWCR & BIT(TWINT))) {;}
  return TWDR;
}

static inline uint8_t twi_read_nack(void) {
  TWCR = BIT(TWINT) | BIT(TWEN);
  while (!(TWCR & BIT(TWINT))) {;}
  return TWDR;
}

// =========================================================
// BH1750 (two sensors, different addresses)
// =========================================================

#define BH1750_ADDR_LOW   0x23
#define BH1750_ADDR_HIGH  0x5C

#define BH1750_CMD_POWER_ON    0x01
#define BH1750_CMD_RESET       0x07
#define BH1750_CMD_CONT_HIRES  0x10  // continuous, 1lx resolution

static inline uint8_t bh1750_write_cmd(uint8_t addr7, uint8_t cmd) {
  if (!twi_start(addr7, 0)) return 0;
  uint8_t ok = twi_write(cmd);
  twi_stop();
  return ok;
}

static inline void bh1750_init(uint8_t addr7) {
  (void)bh1750_write_cmd(addr7, BH1750_CMD_POWER_ON);
  (void)bh1750_write_cmd(addr7, BH1750_CMD_RESET);
  (void)bh1750_write_cmd(addr7, BH1750_CMD_CONT_HIRES);
}

// Raw measurement; lux ≈ raw / 1.2
static inline uint16_t bh1750_read_raw(uint8_t addr7) {
  if (!twi_start(addr7, 1)) return 0;
  uint8_t hi = twi_read_ack();
  uint8_t lo = twi_read_nack();
  twi_stop();
  return (uint16_t)((hi << 8) | lo);
}

static inline uint16_t bh1750_raw_to_lux_uint(uint16_t raw) {
  return (uint16_t)(((uint32_t)raw * 10u) / 12u); // (raw/1.2)
}

static inline uint16_t bh1750_1_read_lux_uint(void) {
  return bh1750_raw_to_lux_uint(bh1750_read_raw(BH1750_ADDR_LOW));
}

static inline uint16_t bh1750_2_read_lux_uint(void) {
  return bh1750_raw_to_lux_uint(bh1750_read_raw(BH1750_ADDR_HIGH));
}

// =========================================================
// EXACT 1ms SCHEDULER TICK (Timer1 CTC)
// =========================================================
//
// F_CPU=16MHz, prescaler /64 => 250 kHz timer clock
// Need 1ms => 250 counts. OCR1A = 249 gives 250 counts (0..249).
//
// ISR fires every 1ms exactly.

static volatile uint32_t g_ms_ticks = 0;

ISR(TIMER1_COMPA_vect) {
  g_ms_ticks++;
}

static inline void sched_init_1ms(void) {
  uint8_t s = SREG;
  cli();

  g_ms_ticks = 0;

  // Timer1: CTC mode (WGM12=1), TOP=OCR1A
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= BIT(WGM12);

  // Compare value for 1ms
  OCR1A = 249;

  // Enable compare match A interrupt
  TIMSK1 |= BIT(OCIE1A);

  // Start timer with prescaler /64 (CS11=1, CS10=1)
  TCCR1B |= BIT(CS11) | BIT(CS10);

  SREG = s;
  sei();
}

static inline uint32_t sched_ms(void) {
  uint32_t t;
  uint8_t s = SREG;
  cli();
  t = g_ms_ticks;
  SREG = s;
  return t;
}

// Cooperative delay (busy-wait). Prefer using sched_ms() checks in a scheduler.
static inline void sched_delay_ms(uint16_t ms) {
  uint32_t start = sched_ms();
  while ((uint32_t)(sched_ms() - start) < ms) {;}
}

// =========================================================
// TOP-LEVEL INIT
// =========================================================
static inline void nano_init(void) {
  // Actuators (PWM + EN)
  actuator1_init();
  actuator2_init();

  // I2C + BH1750s
  twi_init_100khz();
  bh1750_init(BH1750_ADDR_LOW);
  bh1750_init(BH1750_ADDR_HIGH);

  // Exact 1ms scheduler tick
  sched_init_1ms();
}






