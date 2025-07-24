/*
 * lab9part1.c
 *
 *  Created on: Oct 31, 2024
 *      Author: tbibus
 */

#include "ping.h"
#include "timer.h"
#include "lcd.h"
#define SPEEDSOUND_MS 34.0
#define TIME_PER_CLOCK 0.0000625


volatile enum {LOW, HIGH, DONE} state; //set by ISR
volatile unsigned int rising_time; //start time
volatile unsigned int falling_time; //end time
volatile unsigned int overflow_count;

void main(){
    timer_init();
    lcd_init();
    configure_timer();

    int ping_distance = 0;
    float real_distance = 0;
    overflow_count = 0;
    float time_ms = 0.0;

    //speed of sound is 340m/s or 34cm/ms
    //clock speed is 16MHz 0.0000000625 = 0.0000625ms (TODO CONFIRM THIS)
    while(1){
        send_pulse();
        ping_distance = ping_read();


        time_ms = TIME_PER_CLOCK * ping_distance;
        real_distance = time_ms * SPEEDSOUND_MS / 2.0; //divide by 2 bc of the return trip
        lcd_printf("ping dist: %3.3f\noverflow: %d\n", real_distance, overflow_count);
        timer_waitMillis(300);
    }

}

