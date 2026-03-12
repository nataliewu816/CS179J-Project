

#include <avr/io.h>
#include <stdint.h>

#include "inits_v2.h"
#include "printer.h"
#include "driver.h"
#include "motorLimits.h"


int main(void) {
  nano_init();

  serialBegin(); // Printer
  i2c_init();     // Sensors
  sensor_state = Start; // State Machine

  //serialPrint("I Work \n");
  actuator1_enable(1);
  actuator2_enable(1);
  ActuatorControls motionCtl;
  motionCtl.homeMotors();
  while (1) {
    Sensor_Tick(); 
    sched_delay_ms(200);
     if(diff > 50){
      if(L > R){
        //serialPrint("Left is Brighter Right going up\n");
        if(motionCtl.A2POS >= 0){
          motionCtl.angle(1);
        }
        
      } else {
        //serialPrint("Right is Brighter, Left going up\n");
        if(motionCtl.A1POS >= 0){
          motionCtl.angle(-1);
        }
      }
    }
    else
    
    sched_delay_ms(50);

  }
}
