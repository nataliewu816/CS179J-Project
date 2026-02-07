#pragma once
#include "inits_v2.h"

//a global value needs to be defined to know where either actuator is in height

//this means there has to be a homing sequence for the motors to verify their existing positions

//how does the actuator behave when maxed out - does it break itself or just stop moving?

//50mm total freedom

//10mm is 1800ms at 120 speed
//negative speed is UP
//180ms = 1mm


//this will post both actuators to the top.  Initalize your values to either 0 or 50
//for actuator position following this function
void homeMotors(){
    actuator1_set_speed(-120);
    actuator2_set_speed(-120);
    sched_delay_ms(20000);
    actuator1_set_speed(0);
    actuator2_set_speed(0);
}