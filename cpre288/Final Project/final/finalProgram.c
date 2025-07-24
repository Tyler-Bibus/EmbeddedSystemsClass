/*
 * finalProgram.c
 *
 *  Created on: Nov 21, 2024
 *      Author: tbibus
 */

#include <stdio.h>
#include <stdlib.h>
#include "adc.h"
#include "button.h"
#include "comms.h"
#include "dataClean.h"
#include "lcd.h"
#include "movement.h"
#include "open_interface.h"
#include "ping.h"
#include "scan.h"
#include "servo.h"
#include "Timer.h"
#include "uart.h"

#define DO_CALIBRATION 1

extern volatile char uart_flag;
extern volatile char uart_data;

void main(){
    /**
     * INIT FUNCTIONS
     */
    timer_init();
    uart_init();
    init_char_interrupt();
    scan_init();
    lcd_init();
    lcd_printf("Loading Open Interface...");
    oi_t* sensor_data = oi_alloc();
    scan_data scanned_data;
    oi_init(sensor_data);

    /**
     * DECLARE VARIABLES
     */
    char message[100];
    float distances[92];
    char command[20]={0};
    char commandIndex = 0;
    char input_char = 0;
    float input_float = 0;
    int run_program = 1;

    /**
     * SET SONG (RANGO)
     */
    int song_pos = 0;
    int num_notes = 15;
    unsigned char notes[] =    {64, 57, 57, 59, 60, 60, 62, 64, 64, 60, 57, 57, 59, 60, 60};
    unsigned char duration[] = {48, 32, 32, 32, 64, 16, 16, 32, 32, 32, 64, 32, 32, 32, 32};
    oi_loadSong(song_pos, num_notes, notes, duration);

    /**
     * Optional Calibration
     */
    if (DO_CALIBRATION){
        lcd_printf("Calibration Mode");
        button_init();
        servo_calibrate();
    }

    lcd_printf("Ready for Flight"); //print message to LCD display, just so we know we're ready
    while(run_program){
        /**
         * RECIEVE COMMAND
         */
        while(!uart_flag){} //wait for command
        uart_flag = 0;
        commandIndex = 0;
		command[0] = 0; //set null at front
		//read command line and parse
        while(input_char != '\n'){
            command[commandIndex] = uart_data;
            input_char = command[commandIndex];
            commandIndex++;
            while(!uart_flag && input_char!='\n'){}
            uart_flag = 0;
        }
        uart_flag = 0;
		command[commandIndex] = 0;
        input_char = command[0];

        /**
         * DO COMMAND
         */
        switch (input_char){
        case('m'):
            //move forward or backwards
            input_float = atof(command + 1);
                if (input_float < 0){
                    move_backwards(sensor_data, (-1) * (input_float * 10), message);
                } else{
                    move_forward(sensor_data, (input_float * 10), message);
                }
                break;
        case('t'):
                //turn
                input_float = atof(command + 1);
                if (input_float < 0){
                    turn_counterclock(sensor_data, (-1) * ((int)(input_float)));
                    } else{
                    turn_clockwise(sensor_data, ((int)(input_float)));
                }
                break;

        case('s'):
				//do scan stuff
				scan_area(&scanned_data, message, distances);
                cleanData(distances, message, &scanned_data);
                break;
        case('p'):
				//ping not really used
				scan(90.0, &scanned_data);
                scan(90.0, &scanned_data);
				//send scanned_data->sound_dist
				sprintf(message, "%3.3f\n", scanned_data.sound_dist);
				send_message(message);
				break;
        case('o'):
                //sing song "orchestrate"
                //worked before? not sure why its so inconsistent also, no way to implement time in seconds, put in an integer as a position between 0 - 13
			    input_float = atof(command + 1);
                //song
                oi_play_song(song_pos + (int)(input_float));
                break;
        case('f'):
                //flush
                //send f\n to allow GUI to flush messages in case backed up
                sprintf(message, "f\n");
                send_message(message);
                break;
        case('z'):
                run_program = 0;
                break;
        }
		//send "done\n"
		sprintf(message, "d\n");
		send_message(message);
    }
    oi_free(sensor_data);
}

