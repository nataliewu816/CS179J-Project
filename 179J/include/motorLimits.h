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
        
        // Move back down to safe starting position 
        A1DN(40);  
        A2DN(40);  
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
        for(int i=0;i<STEPS;i++){
            actuator2_set_speed(-ACTSPEED);
            sched_delay_ms(mm*MMDELAY);
            actuator2_set_speed(0);
        }
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
        for(int i=0;i<STEPS;i++){
            actuator2_set_speed(ACTSPEED);
            sched_delay_ms(mm*MMDELAY);
            actuator2_set_speed(0);
        }
        A2POS = A2POS - mm;
    }
    //angle function - 0 degrees is flat pointing up, max
    //angle is 3.3 degrees with 50mm x 34" right triangle
    void angle(int degrees){
        //run the A#UP/DN functions here
        if(degrees == 0){
            //run both actuators to bottom limit
            A1DN(MINHEIGHT);
            A2DN(MINHEIGHT);
        }
        else if(degrees > 0){
            //run actuator X to top and Y to bottom
            A1UP(MAXHEIGHT);
            A2DN(MINHEIGHT);
        }
        else{
            //run actuator X to bottom and Y to top
            A2UP(MAXHEIGHT);
            A1DN(MINHEIGHT);
        }
    }



};