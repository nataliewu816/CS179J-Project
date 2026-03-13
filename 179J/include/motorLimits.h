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
inline constexpr int ACT1SPEED = 140;
inline constexpr int ACT2SPEED = 130;
//TODO - steps or MMDELAY adjust
inline constexpr int STEPS = 5;
inline constexpr int MAXHEIGHT = 50;
inline constexpr int MINHEIGHT = 0;
inline constexpr int ACTGAP = 400;
inline constexpr int THTOZ = 1;
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
    void ATOP(){
        actuator1_set_speed(-2*ACT1SPEED);
        actuator2_set_speed(-2*ACT2SPEED);
        sched_delay_ms(5000);
        actuator1_set_speed(0);
        actuator2_set_speed(0);
        A1POS = MAXHEIGHT;
        A2POS = MAXHEIGHT;
    }
    void ABOT(){
        actuator1_set_speed(2*ACT1SPEED);
        actuator2_set_speed(2*ACT2SPEED);
        sched_delay_ms(5000);
        actuator1_set_speed(0);
        actuator2_set_speed(0);
        A1POS = MINHEIGHT;
        A2POS = MINHEIGHT;
        SIDE = 0;
    }
    void homeMotors(){
        //turn on actuators and set to max height
        ATOP();
        sched_de1ay_ms(1000);
        ABOT();
    }
    void calibrationSequence(){
        homeMotors();
        hardAngle(1);
        hardAngle(0);
        hardAngle(-1);
        hardAngle(0);
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

    //INTEGRATION FUNCTION: this should be stepped by the sensor readings
    //enter "1" or "-1" in a while loop comparing sensors for proper use
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
    //do not call this function in integrated code
    void hardAngle(int degrees){
        //determine height to move based on degrees
        //float height = fabs(ACTGAP * tan(degrees * M_PI/180));
        //run the A#UP/DN functions here
        if(degrees == 0){
            //run both actuators to bottom limit
            dualMotor(MAXHEIGHT,2,1,2,1);
        }
        else if(degrees == 1){
            dualMotor(MAXHEIGHT,-1,1,1,1);
        }
        else if(degrees == 2){
            dualMotor(MAXHEIGHT,-2,2,2,1);
        }
        else if(degrees == -2){
            dualMotor(MAXHEIGHT,2,1,-2,2);
        }
        else if(degrees == -1){
            dualMotor(MAXHEIGHT,1,1,-1,1);
        }
    }
    //do not call for integration, replaces code in hardAngle
    //mm, A1neg, A1div, A2neg, A2div
    //universal motor function - set A1neg or A2neg to 0 to cancel movement
    //set to -1 to go up, set to 1 to go down, set higher than 1 for faster
    //set A1div and A2div to slow a particular motor speed
    //feed mm the desired distance - give it small amounts unless doing hard angles
    void dualMotor(int mm, int A1neg, int A1div, int A2neg, int A2div){
        //we don't put the panel all the way up, only all the way down
        if(A1neg < 0 && A2neg < 0){
            return;
        }
        if(A1div == 0 || A2div == 0 || mm == 0){
            return;
        }
        int calcSteps = STEPS*mm;
        if(A1neg < -1 && A2neg > 1){
            calcSteps = calcSteps / A2neg;
        }
        else if(A2neg < -1 && A1neg > 1){
            calcSteps = calcSteps / A1neg;
        }
        else if(A1neg > 1 && A2neg > 1){
            calcSteps = calcSteps / A1neg;
        }
        for(int i=0;i<calcSteps;i++){
                actuator1_set_speed(A1neg * (ACT1SPEED/A1div));
                actuator2_set_speed(A2neg * (ACT2SPEED/A2div));
                sched_delay_ms(MMDELAY);
                actuator1_set_speed(0);
                actuator2_set_speed(0);
            }
            if(A1neg < 0 && A2neg >= 0){
                SIDE = 1;
                A2POS = MINHEIGHT;
                A1POS = MAXHEIGHT/A1div;
            }
            else if(A2neg < 0 && A1neg >= 0){
                SIDE = -1;
                A1POS = MINHEIGHT;
                A2POS = MAXHEIGHT/A2div;
            }
            else{
                SIDE = 0;
                A1POS = MINHEIGHT;
                A2POS = MINHEIGHT;
            }
            
    }


};
