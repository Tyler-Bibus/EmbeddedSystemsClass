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

//prototyping
void send_message(char message[]);
void format_output(int degrees, float distance, char message[]);

//returns objects in view (within 80cm)
void objectDetection(float distances[], char message[], cyBOT_Scan_t *scan){
    char i;
    char object = 0; //if we are iterating an object
    char startAngle = 0;
    char endAngle = 0;
    float objectDistance = 0;
    float objectSize = 0;
    float smallestObjectSize = 0.0;
    int smallestObjectAngle = 0;
    float smallestObjectDistance;
    for (i = 3; i < 90 - 1; i++){
        //ignore all "objects" beyond 55 cm
           if (distances[i] < 55 && fabs(distances[i] - distances[i + 1]) < 10){
               object = 1;
               objectDistance = distances[i];
               startAngle = i * 2;
               endAngle = startAngle;
               while (object == 1 && i < 90){
                   //end object
                   if (distances[i] - distances[i+1] > 10){
                       object = 0;
                       endAngle = i * 2;
                       objectSize = ((endAngle - startAngle) / 360.0) * 2 * 3.1415 * objectDistance;
                       sprintf(message, "\n\rSome obj angle:%d \n\r", (startAngle + endAngle) / 2 - 90);
                       send_message(message);
                       sprintf(message, "Some obj distance:%3.3f \n\r", objectDistance);
                       send_message(message);
                       sprintf(message, "Some obj size:%3.3f \n\r", objectSize);
                       send_message(message);
                       continue;
                   }
                   //continue object
                   else {
                       if (objectDistance > distances[i]){
                           objectDistance = distances[i]; //should get accurate distance...
                       }
                       i++;
                   }
               }
               endAngle = i * 2;
               if (smallestObjectSize <= 0.01 || smallestObjectSize > ((endAngle - startAngle) / 360.0) * 2 * 3.1415 * objectDistance){
                   smallestObjectSize = ((endAngle - startAngle) / 360.0) * 2 * 3.1415 * objectDistance;
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

    return;
}

//cleans input array by removing outliers, then printing them
void cleanData(float uncleanData[], char message[], cyBOT_Scan_t *scan){
    //First, remove/smooth outliers, assume all values are positive
    char i;
    char j;
    char k;
    float bounds = -15;
    char object = 0; //boolean
    for (i = 0; i < 90 - 3; i++){
        //new method, if dramatically, see if within 10 degrees if it returns to low distance
        if (uncleanData[i] - uncleanData[i + 1] < bounds){
            //checks next 6 degrees within bounds
            j = 2;
            while (!object && j >= 1){
                //this is an object (within bounds)
                if (fabs(uncleanData[i] - uncleanData[i + j]) < 15){
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
        object = 0;

    }
    //for (i = 0; i <= 90; i++){
        //format_output(i * 2, uncleanData[i], message);
        //send_message(message);
    //}
    //gets objects
    objectDetection(uncleanData, message, scan);
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
void scan_area(cyBOT_Scan_t *scan, char message[], float distances[], float distIR[]){
    int angle = 0;
    float irDist;
    //iterates and prints noisy data
    for (angle = 0; angle <= 180; angle += 2){
        irDist = irScan(scan, message, angle);
        cyBOT_Scan(angle, scan);
        sprintf(message, "%d  %3.3f", angle, irDist / 100.0);
        send_message(message);
        distances[angle / 2] = scan->sound_dist;
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
    float distanceSound[92];
    float distanceIR[92];
    char input_byte = 'a';

    //do calibration
    cyBOT_SERVRO_cal_t calibrated_vals;
    calibrate(&calibrated_vals);
    right_calibration_value = calibrated_vals.right;
    left_calibration_value = calibrated_vals.left;

    while(input_byte != ESCAPE_KEY){
        while(!flag){
        }
        input_byte = uart_data;
        if(input_byte == 'm'){
            scan_area(&scan, message, distanceSound, distanceIR);
            cleanData(distanceSound, message, &scan);
            cleanData(distanceIR, message, &scan);
        }
        //this calibrates the bot mid program
        //else if (input_byte == 'n'){
        //  calibrate(&calibrated_vals);
        //  right_calibration_value = calibrated_vals.right;
        //  left_calibration_value = calibrated_vals.left;
        //}
        else if (input_byte == 'w'){
            move_forward(sensor_data, 25);
        }
        else if (input_byte == 's'){
            move_backwards(sensor_data, 25);
        }
        else if (input_byte == 'a'){
            turn_counterclock(sensor_data, 5);
        }
        else if(input_byte == 'd'){
            turn_clockwise(sensor_data, 5);
        }
        flag = 0;
    }
    oi_free(sensor_data);
}


