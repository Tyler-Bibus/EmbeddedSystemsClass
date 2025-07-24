/*
#include "Timer.h"
#include "lcd.h"
#include "cyBot_uart.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inc/tm4c123gh6pm.h>
#include "open_interface.h"



void main(){
    char byte = 'a';
    oi_t* sensor_data = oi_alloc();
    oi_init(sensor_data); //change led color
    timer_init(); // Initialize Timer, needed before any LCD screen fucntions can be called
                  // and enables time functions (e.g. timer_waitMillis)
    lcd_init();   // Initialize the the LCD screen.  This also clears the screen.
    cyBot_uart_init();

    while(byte != 'z'){
        byte = cyBot_getByte();
        if(byte == 'w'){
            //move forward
        }
        else if(byte == 's'){
            //move back
        }
        else if(byte == 'a'){
            //left counterclock
        }
        else if(byte == 'd'){
            //right clock
        }
    }
        lcd_printf("goodbye");

        oi_free(sensor_data);
}
*/
