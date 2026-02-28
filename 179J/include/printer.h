#pragma once
#include <avr/io.h>
#include <stdlib.h> 


static inline void serialBegin(){

    // Setting the baud speed to 9600 and the 103 is what is calculated for that

    UBRR0H = 0;
    UBRR0L = 103;
    // Turning on the transmitter for to get data 
    UCSR0B = ( 1 << TXEN0);
 
    // Set text format to standard 8-bit mode for the computer can understand tho
    // MUST BE NEED TO GET READING FROM IT
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 

}

static inline void serialPrint(const char* text) {
    // Loop through every letter in the message
    while(*text) { 
        // Wait and fCheck if the hardware is empty
        // UDRE0 = "Data Register Empty"
        while( !(UCSR0A & (1 << UDRE0)) ); 

        // Send: Put the current letter into the "Outbox" (UDR0)
        UDR0 = *text++; 
    } 
}

static inline void serialPrintNum(uint16_t number) { 
    char buffer[10]; //  hold the text
    utoa(number, buffer, 10); // it need to be utoa not itoa
    serialPrint(buffer); // Print the text
}
