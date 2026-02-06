#pragma once
#include <avr/io.h>
#include <stdint.h>
#include "printer.h" 


#define I2C_SDA PC4
#define I2C_SCL PC5

// --- 1. I2C SETUP ---
static inline void i2c_init() { 
    DDRC &= ~((1<<I2C_SDA)|(1<<I2C_SCL));
    PORTC &= ~((1<<I2C_SDA)|(1<<I2C_SCL));
    TWSR = 0; TWBR = 72; TWCR = (1<<TWEN); 
}

static inline void i2c_start() { TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); while (!(TWCR & (1<<TWINT))); }
static inline void i2c_stop() { TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); }
static inline void i2c_write(uint8_t d) { TWDR = d; TWCR = (1<<TWINT)|(1<<TWEN); while (!(TWCR & (1<<TWINT))); }
static inline uint8_t i2c_read(int ack) { TWCR = (1<<TWINT)|(1<<TWEN)|(ack?(1<<TWEA):0); while (!(TWCR & (1<<TWINT))); return TWDR; }

// Reading part of it and calucated
static inline uint16_t get_lux_raw(uint8_t addr) {
   i2c_start(); 
    i2c_write(addr << 1); // Write Addr
    i2c_write(0x10);      // CMD_HIGH_RES
    i2c_stop(); 
    
    // Step B: Read Data
    i2c_start(); 
    i2c_write((addr << 1) | 1); // Read Addr
    uint8_t hi = i2c_read(1);   // ACK
    uint8_t lo = i2c_read(0);   // NACK
    i2c_stop(); 
    
    uint16_t raw = (hi << 8) | lo;
    return (uint16_t)(((uint32_t)raw * 10) / 12);
}




enum SensorStates { Start, ReadValues, Analyze_Noise, Analyze_Shadow, Analyze_Direct } sensor_state;

uint16_t L = 0;
uint16_t R = 0;
uint16_t diff = 0;

static inline void Sensor_Tick() {
    
    // --- State Transitions ---
    switch (sensor_state) {
        case Start:
            sensor_state = ReadValues;
            break;

        case ReadValues:
            L = get_lux_raw(0x23);
            R = get_lux_raw(0x5C);
            
            if (L > R) diff = L - R;
            else diff = R - L;

            if (diff < 25) sensor_state = Analyze_Noise;
            else if (diff < 75) sensor_state = Analyze_Shadow;
            else sensor_state = Analyze_Direct;
            break;

        case Analyze_Noise:
            sensor_state = ReadValues; 
            break;

        case Analyze_Shadow:
            sensor_state = ReadValues;
            break;

        case Analyze_Direct:
            sensor_state = ReadValues;
            break;

        default:
            sensor_state = Start;
            break;
    }

    // State Actions 
    switch (sensor_state) {
        case Analyze_Noise:
            serialPrint("[Noise]  L:"); serialPrintNum(L);
            serialPrint(" R:"); serialPrintNum(R);
            serialPrint(" D:"); serialPrintNum(diff);
            serialPrint("\n");
            break;

        case Analyze_Shadow:
            serialPrint("[Shadow] L:"); serialPrintNum(L);
            serialPrint(" R:"); serialPrintNum(R);
            serialPrint(" D:"); serialPrintNum(diff);
            serialPrint("\n");
            break;

        case Analyze_Direct:
            serialPrint("[DIRECT] L:"); serialPrintNum(L);
            serialPrint(" R:"); serialPrintNum(R);
            serialPrint(" D:"); serialPrintNum(diff);
            serialPrint("\n");
            break;
    }
}