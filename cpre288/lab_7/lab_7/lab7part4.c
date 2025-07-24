/*
 * lab6p4.c
 *
 *  Created on: Oct 10, 2024
 *      Author: tbibus
 */

#include "timer.h"
#include "lcd.h"
#include "uart.h"
#include "open_interface.h"
#include "movement.h"
#include <stdio.h>
#include "lcd.h"
#include "cyBot_Scan.h"
#include "Timer.h"
#include <string.h>
#include <inc/tm4c123gh6pm.h>
#include <stdlib.h>
#include <math.h>
#define ESCAPE_KEY 'z'

volatile  char uart_data;  // Your UART interupt code can place read data here
volatile  char flag;       // Your UART interupt can update this flag
                           // to indicate that it has placed new data
                           // in uart_data

//prototyping functions
void send_message(char message[]);
void format_output(int degrees, float distance, char message[]);

void printGraphicalView(float distances[], char message[]){
	char x;
	char y;
	
		for(x = 0; x < 180 - 1; x += 2){
			if (distances[x] > 60.0){
				messages[x] = '*';
			} else if (distances[x] > 50.0) {
				messages [x] = '-';
			} else if (distances[x] > 35.0) {
				messages[x] = '=';
			} else if (distances[x] > 20.0) {
				messages[x] = '@';
			}
			else {
				messages[x] = '#';
			}
}


//returns objects in view (within 50cm)
void objectDetection(float distances[], char message[], cyBOT_Scan_t *scan, int smallest[]){
    char i;
    //char object = 0; //keep if we want to keep track of # of objects
    char startAngle = 0;
    char endAngle = 0;
    float objectDistance = 0;
    float objectSize = 0;
    float smallestObjectSize = 0.0;
    int smallestObjectAngle = 0;
    float smallestObjectDistance;
    for (i = 3; i < 90 - 1; i++){
        //start object
        if ((distances[i] < 60.0) && (distances[i-1] - distances[i]) > 1.0){
            startAngle = i * 2;
            endAngle = startAngle;
            while(fabs(distances[i] - distances[i+1]) < 1.0 || (distances[i] - distances[i+1]) > 1.0){
                i++;
            }
            endAngle = i * 2;
            if (startAngle == endAngle){
                continue;
            }
            //get object distance
            cyBOT_Scan((startAngle + endAngle)/2 , scan);
            cyBOT_Scan((startAngle + endAngle)/2 , scan);
            cyBOT_Scan((startAngle + endAngle)/2 , scan);
            objectDistance = scan->sound_dist;
            objectSize = tan((((endAngle - startAngle) * M_PI) / 180.0)) * objectDistance;
            //print off object
            sprintf(message, "\n\rSome obj angle:%d  Start: %d   End: %d \n\r", (startAngle + endAngle) / 2, startAngle, endAngle);
            send_message(message);
            sprintf(message, "Some obj distance:%3.3f \n\r", objectDistance);
            send_message(message);
            sprintf(message, "Some obj size:%3.3f \n\r", objectSize);
            send_message(message);
            if (smallestObjectSize <= 0.01 || smallestObjectSize > objectSize && !(objectSize < 0.01)){
                smallestObjectSize = objectSize;
                smallestObjectAngle = (startAngle + endAngle) / 2;
                smallestObjectDistance = objectDistance;
            }
        }
    }
    //sprintf(message, "%d   %3.3f", smallestObjectAngle, smallestObjectDistance);
    //send_message(message);
    cyBOT_Scan(smallestObjectAngle, scan);
    sprintf(message, "\n\rsmallest obj angle:%d \n\r", smallestObjectAngle - 90);
    send_message(message);
    sprintf(message, "smallest obj distance:%3.3f \n\r", smallestObjectDistance);
    send_message(message);
    smallest[0] = smallestObjectAngle;
    smallest[1] = smallestObjectDistance;
    return;
}

//cleans input array by removing outliers, then printing them
void cleanData(float uncleanData[], char message[], cyBOT_Scan_t *scan, int smallest[]){
    //First, remove/smooth outliers, assume all values are positive
    char i;
    char j;
    char k;
    float bounds = 5;
    char object = 0; //boolean
    for (i = 0; i < 90 - 3; i++){
        //new method, if dramatically, see if within 10 degrees if it returns to low distance
        if (fabs(uncleanData[i] - uncleanData[i + 1]) < bounds && uncleanData[i] < 59.0){
            //checks next 6 degrees within bounds
            j = 3;
            while (!object && j >= 1){
                //this is an object (within bounds)
                if (fabs(uncleanData[i] - uncleanData[i + j]) < bounds){
                    //smooth out object
                    for (k = 1; k <= j; k++){
                        uncleanData[i + k] = uncleanData[i];
                        object = 1; //exit while loop early
                    }
                }
                j--;
            }
            if (!object){
                //remove drop
                uncleanData[i] = uncleanData[i+1];
            }
        }
        else if(uncleanData[i] > 59.0){
            uncleanData[i] = 70; //ignore
        }
        object = 0;
    }
   for (i = 0; i <= 90; i++){
        sprintf(message, "%d %3.3f",i * 2, uncleanData[i] / 100);
        send_message(message);
    }
    //gets objects
    objectDetection(uncleanData, message, scan, smallest);
    return;
}

void send_message(char message[]){
    //sends each character of the message array
    while (*message != 0){
        uart_sendChar(*message);
        message++;
    }
    uart_sendChar('\n');
    //uart_sendChar('\r');
    return;
}

float irScan(cyBOT_Scan_t *data, char *message, int angle){
    float avgDistRaw = 0.0;
    float avgDistReal = 0.0;
    int i;
    for (i = 0; i < 5; i++){
        cyBOT_Scan(angle, data);
        avgDistRaw += data->IR_raw_val;
    }
    avgDistRaw = avgDistRaw / 4.0;
    //clean actual data using line best fit polynomial 2
    avgDistReal = 114 + (-0.0653 * avgDistRaw) + (0.00000995 * avgDistRaw * avgDistRaw);
    return avgDistReal;
}

//scans area in 2 degree incriments
void scan_area(cyBOT_Scan_t *scan, char message[], float distIR[]){
    int angle = 0;
    float irDist;
    //iterates and prints noisy data
    for (angle = 0; angle <= 180; angle += 2){
        irDist = irScan(scan, message, angle);
        //cyBOT_Scan(angle, scan);
        sprintf(message, "%d  %3.3f", angle, irDist / 100.0);
        send_message(message);
        //distances[angle / 2] = scan->sound_dist;
        distIR[angle / 2] = irDist;
    }
    //return;
}

void calibrate(cyBOT_SERVRO_cal_t* vals){
    *vals = cyBOT_SERVO_cal();
}

void main(){
    timer_init();
    lcd_init();
    uart_init();
    init_char_interrupt();
    oi_t* sensor_data = oi_alloc();
    oi_init(sensor_data);
    cyBOT_init_Scan(0b0111);
    cyBOT_Scan_t scan;

    //Arbitrary 50 Bytes, probably could reduce...
    char message[50];
    //float distanceSound[92];
    float distanceIR[92];
    char input_byte = 'a';
    int smallestObject[2] = {0,0}; //holds angle and distance to smallest Object



    //do calibration
    //cyBOT_SERVRO_cal_t calibrated_vals;
    //calibrate(&calibrated_vals);
    right_calibration_value = 269500; // OR calibrated_vals.right;
    left_calibration_value = 1230250;//calibrated_vals.left;

    while(input_byte != ESCAPE_KEY){
        //recieving byte (non blocking
        while(flag){
            input_byte = uart_data;
        }

        //TODO bumping was lab 2 so we should look at that
        //oi_update(sensor_data) //update data
        //bump detection is sensor_data->bumpLeft
        //autonomous mode
        if (input_byte == 't'){
            input_byte = 0;
            while (input_byte != 't'){
                //do autonomous stuff until t is pressed again
                //right now itll just navigate to smallest object
                scan_area(&scan, message, distanceIR);
                cleanData(distanceIR, message, &scan, smallestObject);
                smallestObject[0] = smallestObject[0] - 90;
				//wait here for user autonomous confirmation
				sprintf(message, "Target Angle: %d    Target Distance: %f\r\nProceed to target?\r\n", smallestObject * -1, (smallesObject[1] - 10) * 10;
				send_message(message);
				sprintf(message, "User options: 'h' proceed, 't' go to manual drive, other char will scan again: ");
				send_message(message);
				//reset flag to prevent previous inputs from interfereing
				input_byte = uart_data;
				flag = 0;
				while(!flag){
				}
				input_byte = uart_data;
				if (input_byte != 'h'){
					//exit loop or scan again
					continue;
				}
				//go autonomously to target object
                if (smallestObject[0] >= 0){
                    turn_counterclock(sensor_data, smallestObject[0]);
                }
                else {
                    turn_clockwise(sensor_data, smallestObject[0] * -1);
                }
                move_forward(sensor_data, (smallestObject[1] - 10) * 10);
                input_byte = 't';
                flag = 0;
            }
			//reset so it doesn't re enter loop
			input_byte = 0;
        }
		//Manual Drive
        else{
			if(input_byte == 'm'){
				scan_area(&scan, message, distanceIR);
				//cleanData(distanceSound, message, &scan);
				cleanData(distanceIR, message, &scan, smallestObject);
				input_byte = 0;
			}
			//this calibrates the bot mid program
			//else if (input_byte == 'n'){
			//  calibrate(&calibrated_vals);
			//  right_calibration_value = calibrated_vals.right;
			//  left_calibration_value = calibrated_vals.left;
			//}
			else if (input_byte == 'w'){
				move_forward(sensor_data, 12);
				input_byte = 0;
			}
			else if (input_byte == 's'){
				move_backwards(sensor_data, 12);
				input_byte = 0;
			}
			else if (input_byte == 'a'){
				turn_counterclock(sensor_data, 5);
				input_byte = 0;
			}
			else if(input_byte == 'd'){
				turn_clockwise(sensor_data, 5);
				input_byte = 0;
			}

			flag = 0;
        }
    }
    oi_free(sensor_data);
}


