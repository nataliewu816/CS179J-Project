// nano_hw.h  (ATmega328P / Arduino Nano, bare-metal; no Arduino core)
// Uses only <avr/io.h>.
// - Actuator #1 (BTS7960-style): D4, D5(PWM), D6(PWM), D7
// - Actuator #2 (BTS7960-style): D9, D10(PWM), D11(PWM), D12
// - Sensors: TWO BH1750 light sensors on the same I2C bus (A4/A5)
//     BH1750 #1: ADDR = GND -> 0x23
//     BH1750 #2: ADDR = VCC -> 0x5C
//
// Notes:
// - This header assumes F_CPU = 16MHz (Nano default).
// - Timer usage:
//     Timer0: Actuator #1 PWM on D5(OC0B) and D6(OC0A)
//     Timer1: Actuator #2 RPWM on D10(OC1B) (Fast PWM 8-bit, mode 5)
//     Timer2: Actuator #2 LPWM on D11(OC2A)
// - I2C (TWI) uses A4/A5 only (PC4/PC5), shared by both BH1750s.


//just call "nano_init()" at start of main, this does the rest
#pragma once
#include <avr/io.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define BIT(n) (1u << (n))

// =========================================================
// PIN DEFINITIONS
// =========================================================

// ---- Actuator 1 (PORTD) ----
#define ACT1_REN_DDR   DDRD
#define ACT1_REN_PORT  PORTD
#define ACT1_REN_PIN   PD4

#define ACT1_RPWM_DDR  DDRD
#define ACT1_RPWM_PIN  PD5   // D5  = OC0B

#define ACT1_LPWM_DDR  DDRD
#define ACT1_LPWM_PIN  PD6   // D6  = OC0A

#define ACT1_LEN_DDR   DDRD
#define ACT1_LEN_PORT  PORTD
#define ACT1_LEN_PIN   PD7

// ---- Actuator 2 (PORTB) ----
#define ACT2_REN_DDR   DDRB
#define ACT2_REN_PORT  PORTB
#define ACT2_REN_PIN   PB1   // D9

#define ACT2_RPWM_DDR  DDRB
#define ACT2_RPWM_PIN  PB2   // D10 = OC1B

#define ACT2_LPWM_DDR  DDRB
#define ACT2_LPWM_PIN  PB3   // D11 = OC2A

#define ACT2_LEN_DDR   DDRB
#define ACT2_LEN_PORT  PORTB
#define ACT2_LEN_PIN   PB4   // D12

// ---- I2C pins (PORTC) ----
#define I2C_SDA_PIN    PC4    // A4
#define I2C_SCL_PIN    PC5    // A5

// =========================================================
// PWM INITIALIZATION
// =========================================================

// Actuator #1: Timer0 Fast PWM on OC0A(D6) + OC0B(D5), prescaler /64
static inline void pwm_timer0_init(void) {
  ACT1_RPWM_DDR |= BIT(ACT1_RPWM_PIN); // D5
  ACT1_LPWM_DDR |= BIT(ACT1_LPWM_PIN); // D6

  // Fast PWM, non-inverting on both outputs
  TCCR0A = BIT(WGM01) | BIT(WGM00) | BIT(COM0A1) | BIT(COM0B1);
  TCCR0B = BIT(CS01) | BIT(CS00); // /64

  OCR0A = 0; // D6 duty
  OCR0B = 0; // D5 duty
}

// Actuator #2 RPWM: Timer1 OC1B(D10) Fast PWM 8-bit (mode 5), prescaler /64
static inline void pwm_timer1_init_for_oc1b_8bit(void) {
  ACT2_RPWM_DDR |= BIT(ACT2_RPWM_PIN); // D10

  // Mode 5: Fast PWM 8-bit => WGM10=1, WGM12=1
  // Non-inverting on OC1B => COM1B1=1
  TCCR1A = BIT(WGM10) | BIT(COM1B1);
  TCCR1B = BIT(WGM12) | BIT(CS11) | BIT(CS10); // /64

  OCR1B = 0;
}

// Actuator #2 LPWM: Timer2 OC2A(D11) Fast PWM, prescaler /64
static inline void pwm_timer2_init_for_oc2a(void) {
  ACT2_LPWM_DDR |= BIT(ACT2_LPWM_PIN); // D11

  // Fast PWM, non-inverting on OC2A
  TCCR2A = BIT(WGM21) | BIT(WGM20) | BIT(COM2A1);
  TCCR2B = BIT(CS22); // /64

  OCR2A = 0;
}

// =========================================================
// BTS7960-STYLE ACTUATOR CONTROL (2 PWM + 2 EN)
// =========================================================
//
// Convention:
//   speed in [-255..255]
//   speed > 0: RPWM active, LPWM=0
//   speed < 0: LPWM active, RPWM=0
//   speed = 0: both PWM=0
//
// EN pins must be HIGH for motion.

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

  pwm_timer1_init_for_oc1b_8bit(); // D10
  pwm_timer2_init_for_oc2a();      // D11
}

static inline void actuator1_set_speed(int16_t speed) {
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    OCR0B = (uint8_t)speed; // D5  = RPWM (OC0B)
    OCR0A = 0;              // D6  = LPWM (OC0A)
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
    OCR1B = (uint8_t)speed; // D10 = RPWM (OC1B)
    OCR2A = 0;              // D11 = LPWM (OC2A)
  } else if (speed < 0) {
    OCR1B = 0;
    OCR2A = (uint8_t)(-speed);
  } else {
    OCR1B = 0;
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
//
// 100kHz @ 16MHz: TWBR=72, prescaler=1

static inline void twi_init_100khz(void) {
  // Ensure SDA/SCL are inputs (TWI takes over). Optional: disable pull-ups.
  DDRC &= (uint8_t)~(BIT(I2C_SDA_PIN) | BIT(I2C_SCL_PIN));
  PORTC &= (uint8_t)~(BIT(I2C_SDA_PIN) | BIT(I2C_SCL_PIN)); // no internal pullups

  TWSR = 0x00; // prescaler = 1
  TWBR = 72;   // ~100kHz
  TWCR = BIT(TWEN);
}

static inline uint8_t twi_start(uint8_t addr7, uint8_t read) {
  // START
  TWCR = BIT(TWINT) | BIT(TWSTA) | BIT(TWEN);
  while (!(TWCR & BIT(TWINT))) {;}

  // SLA+R/W
  TWDR = (uint8_t)((addr7 << 1) | (read ? 1 : 0));
  TWCR = BIT(TWINT) | BIT(TWEN);
  while (!(TWCR & BIT(TWINT))) {;}

  uint8_t status = (uint8_t)(TWSR & 0xF8);
  // 0x18: SLA+W ACK, 0x40: SLA+R ACK
  return (status == (read ? 0x40 : 0x18));
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
// BH1750 (two sensors)
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

// Integer lux approximation: lux ≈ raw / 1.2  => (raw*10)/12
static inline uint16_t bh1750_raw_to_lux_uint(uint16_t raw) {
  return (uint16_t)(((uint32_t)raw * 10u) / 12u);
}

static inline uint16_t bh1750_1_read_lux_uint(void) {
  return bh1750_raw_to_lux_uint(bh1750_read_raw(BH1750_ADDR_LOW));
}

static inline uint16_t bh1750_2_read_lux_uint(void) {
  return bh1750_raw_to_lux_uint(bh1750_read_raw(BH1750_ADDR_HIGH));
}

// =========================================================
// TOP-LEVEL PROJECT INIT
// =========================================================
static inline void nano_init(void) {
  // Actuators
  actuator1_init();
  actuator2_init();

  // I2C + BH1750s
  twi_init_100khz();
  bh1750_init(BH1750_ADDR_LOW);
  bh1750_init(BH1750_ADDR_HIGH);
}