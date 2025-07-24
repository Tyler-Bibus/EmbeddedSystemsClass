/**

Author: Tyler Bibus

*/

#include "open_interface.h"
#include "movement.h"
#include <stdio.h>
#include "lcd.h"
#include "cyBot_Scan.h"
#include "Timer.h"
#include "cyBot_uart.h"
#include <string.h>
#include <inc/tm4c123gh6pm.h>
#include <stdlib.h>
#include <math.h>
#define ESCAPE_KEY 'z'

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
    float smallestObjectSize = 0.0;
    int smallestObjectAngle = 0;
    float smallestObjectDistance;
    for (i = 3; i < 90 - 1; i++){
           if (distances[i] < 70 && fabs(distances[i] - distances[i + 1]) < 4){
               object = 1;
               objectDistance = distances[i];
               startAngle = i * 2;
               while (object == 1 && i < 90){
                   //end object
                   if (distances[i] - distances[i+1] > 3){
                       object = 0;
                       continue;
                   }
                   //continue object
                   else {
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
            j = 3;
            while (!object && j >= 2){
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
    for (i = 0; i <= 90; i++){
        //format_output(i * 2, uncleanData[i], message);
        //send_message(message);
    }
    //gets objects
    objectDetection(uncleanData, message, scan);
    return;
}

//Sends message back to putty from string message
void send_message(char message[]){
	//sends each character of the message array
    while (*message != 0){
        cyBot_sendByte(*message);
        message++;
    }
    cyBot_sendByte('\n');
    //cyBot_sendByte('\r');
    return;
}

//formats the output of degrees distance into message
//the %5.5 makes sure it is spread out and trunkates to 5
/*
void format_output(int degrees, float distance, char message[]){
    //convert to meters
    float meters = distance * 0.01;
    //0 degree edge case
    if (degrees == 0){
        sprintf(message, "%0.3d %5.5f", degrees, meters);
        return;
    }
    sprintf(message, "%0.3d %5.5f", degrees, meters);
}
*/

//scans area in 2 degree incriments
void scan_area(cyBOT_Scan_t *scan, char message[], float distances[]){
    int angle = 0;
    //iterates and prints noisy data
    for (angle = 0; angle <= 180; angle += 2){
        cyBOT_Scan(angle, scan);
        //format_output(angle, distance, message);
        //send_message(message);
        distances[angle / 2] = scan->sound_dist;;
    }
    //return;
}

//sets calibrated values after calibration sequence
void calibrate(cyBOT_SERVRO_cal_t* vals){
	*vals = cyBOT_SERVO_cal();
}

void main(){
	//initialize everything
    oi_t* sensor_data = oi_alloc();
    oi_init(sensor_data);
    cyBOT_init_Scan(0b0111);
    cyBOT_Scan_t scan;
    cyBot_uart_init();
    lcd_init();
    timer_init();
	
	//Arbitrary 50 Bytes, probably could reduce...
	char message[50];
	float distances[92];
	//sends initial angle/degrees
	//send_message(message);
	char input_byte = 'a';

	cyBOT_SERVRO_cal_t calibrated_vals;
	//calibration maybte comment out after getting the values...
    calibrate(&calibrated_vals);
	right_calibration_value = calibrated_vals.right;
	left_calibration_value = calibrated_vals.left;
	
	//Main loop goes until exit key is pressed
	    while(input_byte != ESCAPE_KEY){
			input_byte = cyBot_getByte();
			if(input_byte == 'm'){
				scan_area(&scan, message, distances);
			    //cleans data and prints cleaned data
			    cleanData(distances, message, &scan);
			}
			//this calibrates the bot mid program
			//else if (input_byte == 'n'){
			//	calibrate(&calibrated_vals);
			//	right_calibration_value = calibrated_vals.right;
			//	left_calibration_value = calibrated_vals.left;
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
    }
        //lcd_printf("goodbye");


	//terminate session
        oi_free(sensor_data);
	return;
}


