#pragma once
#include <avr/io.h>
#include <stdint.h>
#include "inits_v2.h"
#include "printer.h"  


const int actuatorSpeed = 120;
const int mmToMs = 180;
const int maxMM = 50;

// the position of the actuators A1 is left and A2 is right
int posA1 = 0;
int posA2 = 0; 

void homeMotors(){

    serialPrint("Homing the Motors Moving Up... Please Wait\n");

    // If we have the actual actuators then it will be this for now it is commented it out
    // actuator1_set_speed(-ACTUATOR_SPEED);
    // actuator2_set_speed(-ACTUATOR_SPEED);

    // DONT FORGET TO CHANGE IT TO BACK TO 20 SECONDSSSSSSS
    // it is supposed to be 20s for the actual actuatorsbut  
    // its 2 since i am testing it out on termial
    sched_delay_ms(2000); 

    // When to stop
    // actuator1_set_speed(0);
    // actuator2_set_speed(0);

    // the actuator to max positon
    posA1 = maxMM;
    posA2 = maxMM;

    serialPrint("Homing Complete\n");
}

void A1Up(int mm){

    // make sure it doesnt go over 50 mm and cap it
    if((posA1 + mm) > maxMM){
        mm = maxMM - posA1;
    }

    if(mm > 0){
        // for now we are not test da actual actuators yet
        // actuator2_set_speed(-ACTUATOR_SPEED);

        serialPrint("A1 Moving UP by "); 
        serialPrintNum(mm); 
        serialPrint("mm\n");
        
        sched_delay_ms(mm * mmToMs);


        // actuator2_set_speed(0);
        // update pos
        posA1 += mm;
        
        // testing
        serialPrint("A1 Stopped. New Pos: ");
        serialPrintNum(posA1);
        serialPrint("mm\n");
    } else {
        serialPrint("A1 already at MAX height\n");
    }
}

void A2Up(int mm){

    // make sure it doesnt go over 50 mm and cap it
    if((posA2 + mm) > maxMM){
        mm = maxMM - posA2;
    }

    if(mm > 0){
        // for now we are not test da actual actuators yet
        // actuator2_set_speed(-ACTUATOR_SPEED);

        serialPrint("A2 Moving UP by "); 
        serialPrintNum(mm); 
        serialPrint("mm\n");
        
        sched_delay_ms(mm * mmToMs);


        // actuator2_set_speed(0);
        // update pos
        posA2 += mm;
        
        // testing
        serialPrint("A2 Stopped. New Pos: ");
        serialPrintNum(posA2);
        serialPrint("mm\n");
    } else {
        serialPrint("A2 already at MAX height\n");
    }
}

void A1Down(int mm){

    // make sure it doesnt go under 0 mm and cap it
    if((posA1 - mm) < 0){
        mm = posA1;
    }

    if(mm > 0){
        // for now we are not test da actual actuators yet
        // actuator2_set_speed(-ACTUATOR_SPEED);

        serialPrint("A1 Moving Down by "); 
        serialPrintNum(mm); 
        serialPrint("mm\n");
        
        sched_delay_ms(mm * mmToMs);


        // actuator2_set_speed(0);
        // update pos
        posA1 -= mm;
        
        // testing
        serialPrint("A1 Stopped. New Pos: ");
        serialPrintNum(posA1);
        serialPrint("mm\n");
    } else {
        serialPrint("A1 already at da BOTTOM");
    }
}

void A2Down(int mm){

    // make sure it doesnt go under 0 mm and cap it
    if((posA2 - mm) < 0){
        mm = posA2;
    }

    if(mm > 0){
        // for now we are not test da actual actuators yet
        // actuator2_set_speed(-ACTUATOR_SPEED);

        serialPrint("A2 Moving Down by "); 
        serialPrintNum(mm); 
        serialPrint("mm\n");
        
        sched_delay_ms(mm * mmToMs);


        // actuator2_set_speed(0);
        // update pos
        posA2 -= mm;
        
        // testing
        serialPrint("A2 Stopped. New Pos: ");
        serialPrintNum(posA2);
        serialPrint("mm\n");
    } else {
        serialPrint("A2 already at da BOTTOM");
    }
}