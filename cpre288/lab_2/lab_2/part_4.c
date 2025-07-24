/*
 * part_4.c
 *
 *  Created on: Sep 12, 2024
 *      Author: tbibus
 */

#include "Timer.h"
#include "lcd.h"
#include "cyBot_uart.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inc/tm4c123gh6pm.h>
#include "open_interface.h"

void send_message(char c, char* message){
    while (*message != 0){
        cyBot_sendByte(*message);
        message++;
    }

    cyBot_sendByte(c);
    cyBot_sendByte('\n');
    cyBot_sendByte('\r');
}


int main (void) {
    char message[] = "Got an ";
    char* msg = message;
    char byte = 'a';
    oi_t* sensor_data = oi_alloc();
    oi_init(sensor_data); //change led color
    timer_init(); // Initialize Timer, needed before any LCD screen fucntions can be called
                  // and enables time functions (e.g. timer_waitMillis)
    lcd_init();   // Initialize the the LCD screen.  This also clears the screen.
    cyBot_uart_init();

    while(byte != 'z'){
        byte = cyBot_getByte();
        lcd_printf("got a: %c", byte);
        send_message(byte, msg);
        if(byte == 'w'){
            oi_setWheels(200,200);
        }
        else if(byte == 's'){
            oi_setWheels(-200,-200);
        }
        else if(byte == 'a'){
            oi_setWheels(200,-200);
        }
        else if(byte == 'd'){
            oi_setWheels(-200,200);
        }
        else {
            oi_setWheels(0,0);
        }
    }
        lcd_printf("goodbye");

        oi_free(sensor_data);

    return 0;
}




