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
#define SONG_RANGO 0
#define SONG_FUNKYTOWN 1

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
    int song_index = 0;
    int run_program = 1;

    /**
     * LOAD SONGS
     * Slot 0 = Rango, slot 1 = Funkytown.  The 'o <index>' command plays one.
     */
    int num_notes = 15;
    unsigned char notes[] =    {64, 57, 57, 59, 60, 60, 62, 64, 64, 60, 57, 57, 59, 60, 60};
    unsigned char duration[] = {48, 32, 32, 32, 64, 16, 16, 32, 32, 32, 64, 32, 32, 32, 32};
    oi_loadSong(SONG_RANGO, num_notes, notes, duration);

    // Funkytown-style riff: C C E F G F E C | G G E C D C A G
    int funk_num_notes = 16;
    unsigned char funk_notes[] =    {72, 72, 76, 77, 79, 77, 76, 72,
                                     79, 79, 76, 72, 74, 72, 69, 67};
    unsigned char funk_duration[] = {16, 16, 16, 16, 16, 16, 16, 16,
                                     16, 16, 16, 16, 16, 16, 16, 48};
    oi_loadSong(SONG_FUNKYTOWN, funk_num_notes, funk_notes, funk_duration);

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
         * RECEIVE COMMAND
         * uart_getchar() blocks until one byte is available from the RX ring
         * buffer.  The buffer is filled by the UART1 interrupt, so bytes cannot
         * be lost while a previous command is being executed.
         */
        commandIndex = 0;
        command[0] = 0;
        while (1){
            input_char = uart_getchar();
            if (input_char == '\n'){
                break;
            }
            if (input_char == '\r'){
                continue;
            }
            if (commandIndex < (sizeof(command) - 1)){
                command[commandIndex] = input_char;
                commandIndex++;
            }
            // Overlong input is drained until the newline and then discarded.
        }
        command[commandIndex] = 0;
        input_char = command[0];

        /**
         * DO COMMAND
         */
        switch (input_char){
        case('m'):
            //move forward or backwards
            input_float = atof(command + 1);
            if (input_float > 500.0) input_float = 500.0;
            if (input_float < -500.0) input_float = -500.0;
                if (input_float < 0){
                    move_backwards(sensor_data, (-1) * (input_float * 10), message);
                } else{
                    move_forward(sensor_data, (input_float * 10), message);
                }
                break;
        case('t'):
                //turn
                input_float = atof(command + 1);
                if (input_float > 360.0) input_float = 360.0;
                if (input_float < -360.0) input_float = -360.0;
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
        case('o'): {
                //sing song "orchestrate"
                //worked before? not sure why its so inconsistent also, no way to implement time in seconds, put in an integer as a position between 0 - 13
			    input_float = atof(command + 1);
                song_index = (int)(input_float);
                if (song_index < SONG_RANGO) song_index = SONG_RANGO;
                if (song_index > SONG_FUNKYTOWN) song_index = SONG_FUNKYTOWN;
                //song
                oi_play_song(song_index);
                break;
        }
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

