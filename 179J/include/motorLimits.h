#pragma once
#include "inits_v2.h"

//a global value needs to be defined to know where either actuator is in height

//this means there has to be a homing sequence for the motors to verify their existing positions

//how does the actuator behave when maxed out - does it break itself or just stop moving?

//50mm total freedom

//10mm is MMDELAY0ms at 120 speed
//negative speed is UP
//MMDELAYms = 1mm
//120 for 70
//60 for 140
inline constexpr int MMDELAY = 35;
inline constexpr int ACTSPEED = 61;
inline constexpr int STEPS = 10;
inline constexpr int MAXHEIGHT = 50;
inline constexpr int MINHEIGHT = 0;
//ActuatorTracker must be declared in main.cpp to use functions - it will track
//the positions of actuators at all times
struct ActuatorControls{

int A1POS = -1;
int A2POS = -1;

//this will post both actuators to the top and must be called after initializing
//the tracker structure
    void homeMotors(){
        actuator1_set_speed(-120);
        actuator2_set_speed(-120);
        sched_delay_ms(10000);
        actuator1_set_speed(0);
        actuator2_set_speed(0);
        A1POS = 50;
        A2POS = 50;
        //TODO add code to run actuators back down to the threshhold level
    }
    //move actuator 2 up by X mm (max 50)
    void A1UP(int mm){
        if(A1POS == MAXHEIGHT || mm <= 0){
            return;
        }
        if((A1POS + mm) > MAXHEIGHT){
            mm = MAXHEIGHT - A1POS;
        }
        for(int i=0;i<STEPS;i++){
            actuator1_set_speed(-ACTSPEED);
            sched_delay_ms(mm*MMDELAY);
            actuator1_set_speed(0);
        }
        A1POS = A1POS + mm;
    }
    //move actuator 2 up by X mm (max 50)
    void A2UP(int mm){
        if(A2POS == MAXHEIGHT || mm <= 0){
            return;
        }
        if((A2POS + mm) > MAXHEIGHT){
            mm = MAXHEIGHT - A2POS;
        }
        actuator2_set_speed(-ACTSPEED);
        sched_delay_ms(mm*MMDELAY);
        actuator2_set_speed(0);
        A2POS = A2POS + mm;
    }
    //move actuator 1 down by X mm (max 50)
    void A1DN(int mm){
        if(A1POS == MINHEIGHT || mm <= 0){
            return;
        }
        if((A1POS - mm) < MINHEIGHT){
            mm = A1POS - MINHEIGHT;
        }
        for(int i=0;i<STEPS;i++){
            actuator1_set_speed(ACTSPEED);
            sched_delay_ms(mm*MMDELAY);
            actuator1_set_speed(0);
        }
        A1POS = A1POS - mm;
    }
    //move actuator 2 down by X mm (max 50)
    void A2DN(int mm){
        if(A2POS == MINHEIGHT || mm <= 0){
            return;
        }
        if((A2POS - mm) < MINHEIGHT){
            mm = A2POS - MINHEIGHT;
        }
        actuator2_set_speed(ACTSPEED);
        sched_delay_ms(mm*MMDELAY);
        actuator2_set_speed(0);
        A2POS = A2POS - mm;
    }
};