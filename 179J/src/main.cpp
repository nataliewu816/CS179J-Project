

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// for the reading
#include "../include/inits_v2.h"
#include "../include/printer.h"
#include "../include/driver.h"

#define NANO_HW_IMPLEMENTATION


void actuTest1(){
  nano_init();

  actuator1_enable(1);
  actuator2_enable(1);

  while (1) {
    actuator1_set_speed(120);   // try 120 first for reliable movement
    sched_delay_ms(2000);

    actuator1_set_speed(0);
    sched_delay_ms(500);

    actuator1_set_speed(-120);
    sched_delay_ms(2000);

    actuator1_set_speed(0);
    sched_delay_ms(1000);
  }
}

int main(void) {
  nano_init();

  serialBegin(); // Printer
  i2c_init();     // Sensors
  sensor_state = Start; // State Machine


  // Make sure enables are on
  // actuator1_enable(1);
  // actuator2_enable(1);

  while (1) {

    Sensor_Tick(); 
    // need for the reading of it 
    sched_delay_ms(200);

    /*
    // State 1: D3 & D5 ON, D6 & D11 OFF
    OCR0B = 255; // D5
    OCR2B = 255; // D3
    OCR0A = 0;   // D6
    OCR2A = 0;   // D11
    sched_delay_ms(1000);

    // State 2: D6 & D11 ON, D3 & D5 OFF
    OCR0B = 0;
    OCR2B = 0;
    OCR0A = 255;
    OCR2A = 255;
    sched_delay_ms(1000);
    */
  }
}
