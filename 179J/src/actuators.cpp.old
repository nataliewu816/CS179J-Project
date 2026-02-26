

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "../include/inits_v2.h"

#define NANO_HW_IMPLEMENTATION

int main(void) {
  nano_init();
  actuator1_set_speed(0);
  actuator2_set_speed(0);
  actuator1_enable(1);
  actuator2_enable(0);
 

  while (1) {
    // actuator1_set_speed(120);   // try 120 first for reliable movement
    // sched_delay_ms(2000);

    // actuator1_set_speed(0);
    // sched_delay_ms(500);

    // actuator1_set_speed(-120);
    // sched_delay_ms(2000);

    // actuator1_set_speed(0);
    // sched_delay_ms(1000);
  }
}
