/**

Author: Tyler Bibus

*/
/*
#include <stdio.h>
#include "lcd.h"
#include "cyBot_Scan.h"
#include "Timer.h"
#include "cyBot_uart.h"
#include <string.h>
#include <inc/tm4c123gh6pm.h>
#include <stdlib.h>

#define ESCAPE_KEY 'z'

//Sends message back to putty from string message
void send_message(char* message){
	//sends each character of the message array
    while (*message != 0){
        cyBot_sendByte(*message);
        message++;
    }
    cyBot_sendByte('\n');
    cyBot_sendByte('\r');
}

//formats the output of degrees distance into message
//the %3.0 formats it pretty
void format_output(int degrees, float distance, char* message){
    //0 degree edge case
    if (degrees == 0){
        sprintf(message, "  %d %5.0f", degrees, distance);
        return;
    }
	sprintf(message, "%0.3d %5.0f", degrees, distance);
}

void scan_area(cyBOT_Scan_t scan, char* message){
    int angle = 0;
    float distance = 0;
    for (angle = 0; angle <= 180; angle += 2){
        cyBOT_Scan(angle, &scan);
        distance = scan.sound_dist;
        format_output(angle, distance, message);
        send_message(message);
    }
    return;
}

void main(){
	//initialize everything
    cyBOT_init_Scan(0b0111);
	cyBOT_Scan_t scan;
	cyBot_uart_init();
	
	//Arbitrary 50 Bytes, probably could reduce...
	char messageArray[50] = "Angle(Degrees)  Distance(m)";
	char* message = messageArray;
	//sends initial angle/degrees
	send_message(message);
	char input_byte = 'a';
	
	//Main loop goes until exit key is pressed
	    while(input_byte != ESCAPE_KEY){
			input_byte = cyBot_getByte();
			if(input_byte == 'm'){
				scan_area(scan, message);
			}
    }
        lcd_printf("goodbye");
	
	//terminate session

	return;
}

*/

