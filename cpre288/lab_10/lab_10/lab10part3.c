/**
 * Simple Mission program (SOCKET)
 * @author Tyler Bibus
 */
/*
#include "Timer.h"
#include "servo.h"
#include "scan.h"
#include "movement.h"
#include "lcd.h"
#include "comms.h"
#include "dataClean.h"
#include "navigation.h"
#include "adc.h"
#include "open_interface.h"
#include "uart.h"
#define ESCAPE_KEY 'z'

volatile char uart_data;
volatile char uart_flag;

void main(){
    timer_init();
    lcd_init();
    uart_init();
    init_char_interrupt();
    oi_t* sensor_data = oi_alloc();
    oi_init(sensor_data);
    scan_init(); //initialize IR, ping, servo
    scan_data data;

    //180 to print out depth
    char message[190];
    //float distanceSound[92];
    float distanceIR[92];
    char input_byte = 0;
    int smallestObject[2] = {0,0}; //holds angle and distance to smallest Object
    char previousBump = 0;
    int targetDistance = 0;
    int targetAngle = 0;

    char command[10];
    char commandIndex = 0;


    //do calibration
    //cyBOT_SERVRO_cal_t calibrated_vals;
    //calibrate(&calibrated_vals);
    //right_calibration_value = 269500; // OR calibrated_vals.right;
    //left_calibration_value = 1230250;//calibrated_vals.left;

    while(input_byte != ESCAPE_KEY){
        oi_update(sensor_data);
        //recieving byte (non blocking
        while(!uart_flag){} //blocking
        commandIndex = 0;
        while(input_byte != '\n'){
            command[commandIndex] = input_byte = uart_data;
            commandIndex++;
            while(!uart_flag){} //block for next data
        }
        uart_flag = 0;
        input_byte = command[0];

        if (input_byte == 'h'){
            sprintf(message, "done\n");
            send_message(message);
            continue;
        }

        //oi_update(sensor_data) //update data
        //bump detection is sensor_data->bumpLeft
        //autonomous mode
        if (input_byte == 't'){
            input_byte = 0;
            while (input_byte != 't'){
                //do autonomous stuff until t is pressed again
                //right now itll just navigate to smallest object
                sprintf(message, "Entering Autonomous Mode, press 'h' to scan or 't' to return to manual\r\n");
                send_message(message);
                while(!uart_flag){} //blocking
                uart_flag = 0;
                commandIndex = 0;
                while(input_byte != '\n'){
                    command[commandIndex] = input_byte = uart_data;
                    commandIndex++;
                }
                input_byte = command[0];
                if(input_byte != 'h'){
                    continue;
                }
                sprintf(message, "Scanning Area...\r\n");
                send_message(message);

                scan_area(&data, message, distanceIR);
                cleanData(distanceIR, message, &data, smallestObject);
                printGraphicalView(distanceIR, message);
                smallestObject[0] = smallestObject[0] - 90;
                //wait here for user autonomous confirmation.
                sprintf(message, "Target Angle: %d    Target Distance: %d\r\nProceed to target?\r\n", smallestObject[0] * -1, (smallestObject[1] - 10) * 10);
                send_message(message);
                sprintf(message, "User options: 'h' proceed, 't' go to manual drive, other char will scan again: ");
                send_message(message);
                sprintf(message, "done\n");
                send_message(message);
                //reset flag to prevent previous inputs from interfereing
                input_byte = uart_data;
                uart_flag = 0;
                //blocks until input
                while(!uart_flag){} //blocking
                uart_flag = 0;
                commandIndex = 0;
                while(input_byte != '\n'){
                    command[commandIndex] = input_byte = uart_data;
                    commandIndex++;
                }
                input_byte = command[0];
                if (input_byte != 'h'){
                    //exit loop or scan again
                    continue;
                }
                //go autonomously to target object (navigate around bumps)
                targetDistance = smallestObject[1];
                targetAngle = smallestObject[0];
                moveToTarget(&targetDistance, targetAngle, sensor_data, message);
                sprintf(message, "done\n");
                send_message(message);
            }
            //reset so it doesn't re enter loop
            input_byte = 0;
        }
        //Manual Drive
        else{
            //manual drive bump detection
            if(sensor_data->bumpLeft && !previousBump){
                sprintf(message, "Bump on Left! Stop!");
                send_message(message);
                previousBump = 1;
            }
            else if(sensor_data->bumpRight && !previousBump){
                sprintf(message, "Bump on Right! Stop!");
                send_message(message);
                previousBump = 1;
            }
            else if (!sensor_data->bumpLeft && !sensor_data->bumpRight){
                previousBump = 0;
            }


            if(input_byte == 'm'){
                sprintf(message, "scanning area");
                send_message(message);
                scan_area(&data, message, distanceIR);
                //cleanData(distanceSound, message, &scan);
                cleanData(distanceIR, message, &data, smallestObject);
                printGraphicalView(distanceIR, message);
                input_byte = 0;
                sprintf(message, "done\n");
                send_message(message);
            }
            //this calibrates the bot mid program
            //else if (input_byte == 'n'){
            //  calibrate(&calibrated_vals);
            //  right_calibration_value = calibrated_vals.right;
            //  left_calibration_value = calibrated_vals.left;
            //}
            else if (input_byte == 'w'){
                sprintf(message, "moving");
                send_message(message);
                move_forward(sensor_data, 12);
                input_byte = 0;
            }
            else if (input_byte == 's'){
                sprintf(message, "moving");
                send_message(message);
                move_backwards(sensor_data, 12);
                input_byte = 0;
            }
            else if (input_byte == 'a'){
                sprintf(message, "moving");
                send_message(message);
                turn_counterclock(sensor_data, 5);
                input_byte = 0;
            }
            else if(input_byte == 'd'){
                sprintf(message, "moving");
                send_message(message);
                turn_clockwise(sensor_data, 5);
                input_byte = 0;
            }
            else if (input_byte == 'q'){
                send_message(message);
                scan(90, &data);
                scan(90, &data);
                sprintf(message, "Ping Distance: %3.3f", data.sound_dist - 5);
                send_message(message);
                input_byte = 0;
            }
        }
    }
    oi_free(sensor_data);
}

*/
