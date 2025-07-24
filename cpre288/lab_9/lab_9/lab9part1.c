/*
 * lab9part1.c
 *
 *  Created on: Oct 31, 2024
 *      Author: tbibus
 */
/*
#include "ping.h"
#include "timer.h"
#include "lcd.h"

volatile enum {LOW, HIGH, DONE} state; //set by ISR
volatile unsigned int rising_time; //start time
volatile unsigned int falling_time; //end time

void main(){
    timer_init();
    lcd_init();
    configure_timer();

    int ping_distance = 0;
    while(1){
        //send_pulse();
        ping_distance = ping_read();
        lcd_printf("ping dist is : %d\n", ping_distance);
        timer_waitMillis(1000);
    }

}
*/
