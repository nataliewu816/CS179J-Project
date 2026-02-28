#pragma once
#include "inits_v2.h"
#include "math.h"
#include "printer.h"
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
inline constexpr int ACTGAP = 610;
inline constexpr int THTOZ = 10;
//ActuatorTracker must be declared in main.cpp to use functions - it will track
//the positions of actuators and the panel side at all times
struct ActuatorControls{

int A1POS = -1;
int A2POS = -1;
int SIDE = 0;
//this will post both actuators to the top and must be called after initializing
//the tracker structure
    //call this for reporting each line item without error callouts
    void spawnReport(){
        serialBegin();
        serialPrint("A1POS = ");serialPrintNum(A1POS);serialPrint("\n");
        serialPrint("A2POS = ");serialPrintNum(A2POS);serialPrint("\n");
        serialPrint("SIDE = ");serialPrintNum(SIDE);serialPrint("\n");
    }
    //call this for just callouts when something is wrong
    void debugger(){
        serialBegin();
        if(A1POS < MINHEIGHT){
            serialPrint("A1 LOST CALIBRATION ON THE BOTTOM");
        }
        if(A1POS > MAXHEIGHT){
            serialPrint("A1 LOST CALIBRATION ON THE TOP");
        }
        if(A2POS < MINHEIGHT){
            serialPrint("A2 LOST CALIBRATION ON THE BOTTOM");
        }
        if(A2POS > MAXHEIGHT){
            serialPrint("A2 LOST CALIBRATION ON THE TOP");
        }
        if(SIDE == 1 && (A1POS <= MINHEIGHT || A2POS > MINHEIGHT)){
            serialPrint("SIDE CORRUPTION - SIDE 1 WITH BAD POS - EXPECT A2MIN A1VAR");
            serialPrint("A1POS = ");serialPrintNum(A1POS);serialPrint("\n");
            serialPrint("A2POS = ");serialPrintNum(A2POS);serialPrint("\n");
        }
        if(SIDE == -1 && (A2POS <= MINHEIGHT || A1POS > MINHEIGHT)){
            serialPrint("SIDE CORRUPTION - SIDE 2 WITH BAD POS - EXPECT A1MIN A2VAR");
            serialPrint("A1POS = ");serialPrintNum(A1POS);serialPrint("\n");
            serialPrint("A2POS = ");serialPrintNum(A2POS);serialPrint("\n");
        }
    }
    void homeMotors(){
        //turn on actuators and set to max height
        actuator1_set_speed(-120);
        actuator2_set_speed(-120);
        sched_delay_ms(10000);
        actuator1_set_speed(0);
        actuator2_set_speed(0);
        A1POS = MAXHEIGHT;
        A2POS = MAXHEIGHT;
        //verify internal functions and return to lowest point
        A1DN(MAXHEIGHT);
        A2DN(MAXHEIGHT);
        SIDE = 0;
    }
    //move actuator 1 up by X mm (max 50)
    void A1UP(int mm){
        if(A1POS == MAXHEIGHT || mm <= 0){
            return;
        }
        if((A1POS + mm) > MAXHEIGHT){
            mm = MAXHEIGHT - A1POS;
        }
        int calcSteps = STEPS*mm;
        for(int i=0;i<calcSteps;i++){
            actuator1_set_speed(-ACTSPEED);
            sched_delay_ms(MMDELAY);
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
        int calcSteps = STEPS*mm;
        for(int i=0;i<calcSteps;i++){
            actuator2_set_speed(-ACTSPEED);
            sched_delay_ms(MMDELAY);
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
        int calcSteps = STEPS*mm;
        for(int i=0;i<calcSteps;i++){
            actuator1_set_speed(ACTSPEED);
            sched_delay_ms(MMDELAY);
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
        int calcSteps = STEPS*mm;
        for(int i=0;i<calcSteps;i++){
            actuator2_set_speed(ACTSPEED);
            sched_delay_ms(MMDELAY);
            actuator2_set_speed(0);
        }
        A2POS = A2POS - mm;
    }
    //angle function - 0 degrees is flat pointing up, max
    //angle is 1.65 degrees with 50mm x 34" right triangle

    //this should be stepped by the sensor readings
    //for example, the sensor should read "L greater than R"
    //feed a small value into the function, let it run
    //then check if it needs to run again (stepping)
    //angle maximum to either side is 4.65
    void angle(float degrees){
        //determine height to move based on degrees
        float height = fabs(ACTGAP * tan(degrees * M_PI/180.0));
        //run the A#UP/DN functions here
        if(degrees == 0){
            //the function was fed "Do not move"
            return;
        }
        else if(degrees > 0){//light is coming from the side with A1 up
            //run actuator X to top and Y to bottom
            if(SIDE == 0){
                hardAngle(2);
            }
            else if(SIDE == 1){
                //move more to the side it's already on
                A1UP(height);
                if(A1POS > MAXHEIGHT-THTOZ){
                    hardAngle(1);
                }
            }
            else{//side is opposite
                //step to the other side
                A1DN(height);
                //A1 is close to the bottom already, lets bottom it out
                if(A1POS < MINHEIGHT+THTOZ){
                    hardAngle(0);
                }
            }
            
        }
        else{//light is coming from the side with A2 UP
            //run actuator X to top and Y to bottom
            if(SIDE == 0){
                hardAngle(-2);
            }
            else if(SIDE == -1){
                //move more to the side it's already on
                A2UP(height);
                if(A2POS > MAXHEIGHT-THTOZ){
                    hardAngle(-1);
                }
            }
            else{//side is opposite
                //step to the other side
                A2DN(height);
                //A1 is close to the bottom already, lets bottom it out
                if(A2POS < MINHEIGHT+THTOZ){
                    hardAngle(0);
                }
            }
        }
    }

    //need a function that will check if A1POS is less than MIN and reset the value AND position
    //how to determine when it finds out it is out of calibration

    //Give -1,1,or 0 to move actuator to one side
    //-1 will move A2 up or down first
    //1 will move A1 up or down first
    //0 is center, 2s are half angle
    void hardAngle(int degrees){
        //determine height to move based on degrees
        //float height = fabs(ACTGAP * tan(degrees * M_PI/180));
        //run the A#UP/DN functions here
        if(degrees == 0){
            //run both actuators to bottom limit
            A1DN(MAXHEIGHT);
            A2DN(MAXHEIGHT);
            SIDE = 0;
            A1POS = MINHEIGHT;
            A2POS = MINHEIGHT;
        }
        else if(degrees == 1){
            //run actuator X to top and Y to bottom
            A1UP(MAXHEIGHT);
            A2DN(MAXHEIGHT);
            SIDE = 1;
            A1POS = MAXHEIGHT;
            A2POS = MINHEIGHT;
        }
        else if(degrees == 2){
            //run actuator X to top and Y to bottom
            A1UP(MAXHEIGHT/2);
            A2DN(MAXHEIGHT);
            SIDE = 1;
            A2POS = MINHEIGHT;

        }
        else if(degrees == -2){
            //run actuator X to top and Y to bottom
            A2UP(MAXHEIGHT/2);
            A1DN(MAXHEIGHT);
            SIDE = -1;
            A1POS = MINHEIGHT;
        }
        else{
            //run actuator X to bottom and Y to top
            A2UP(MAXHEIGHT);
            A1DN(MAXHEIGHT);
            SIDE = -1;
            A1POS = MINHEIGHT;
            A2POS = MAXHEIGHT;
        }
    }



};