/*
 * lab6p4.c
 *
 *  Created on: Oct 10, 2024
 *      Author: tbibus
 */

#include "adc.h"
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

//moves to target automagically while moving by bumps NOTE: DOES NOT MAKE IT ALL THE WAY TO TARGET IF BUMPED
void moveToTarget(int* targetDistance, int targetAngle, oi_t* sensor_data, char message[]){
    char user_input = 0;
    char atTarget = 0;
    char previousBump = 0;
    //turn to be aligned with target (angle offset = 0)
    if (targetAngle >= 0){
        turn_counterclock(sensor_data, targetAngle);
    }
    else {
        turn_clockwise(sensor_data, targetAngle * -1);
    }
    int tempTarget = 0;
    int returnDistance = 0;
    int returnAngle = 0;
    int internalBump = 0;
    while(*targetDistance > 1 && !atTarget){
        if(flag){
            user_input = uart_data;
            flag = 0;
            if(user_input =='t'){
                return;//user wants to return early
            }
        }


        //attempt to Reach Target
        previousBump = moveWithBump(sensor_data,targetDistance);
        if (*targetDistance < 2 && !previousBump){
            atTarget = 1;
            break;
        }
        //left bump, navigate
        while(previousBump == 1){
            sprintf(message, "Bump on Left! navigating around...\r\n");
            send_message(message);
            if(flag){
                user_input = uart_data;
                flag = 0;
                if(user_input =='t'){
                    return;//user wants to return early
                }
            }
            tempTarget = 450;
            returnAngle += 95;
            move_backwards(sensor_data, 50);
            turn_clockwise(sensor_data, 70);
            internalBump = moveWithBump(sensor_data, &tempTarget);
            returnDistance += 450 - tempTarget + 50;
            if(internalBump){
                sprintf(message, "Bump again! navigating around...\r\n");
                send_message(message);
                continue; //repeat until clear
            }
            turn_counterclock(sensor_data, returnAngle + 20);
            previousBump = 0;
            //previousBump = moveWithBump(sensor_data, &returnDistance);
        }
        while(previousBump == 2){
            sprintf(message, "Bump on Right! navigating around...\r\n");
            send_message(message);
            if(flag){
                user_input = uart_data;
                flag = 0;
                if(user_input =='t'){
                    return;//user wants to return early
                }
            }
            tempTarget = 350;
            returnAngle += 95;
            move_backwards(sensor_data, 10);
            turn_counterclock(sensor_data, 70);
            internalBump = moveWithBump(sensor_data, &tempTarget);
            returnDistance += 350 - tempTarget + 10;
            if(internalBump){
                sprintf(message, "Bump again! navigating around...\r\n");
                send_message(message);
                continue; //repeat until clear
            }
            turn_clockwise(sensor_data, returnAngle + 20);
            previousBump = 0;
            //previousBump = moveWithBump(sensor_data, &returnDistance);
        }

    }
}

//prints a graphical view of the scan, denser the character the closer the object
void printGraphicalView(float distances[], char message[]){
	char j;
	j = 0;
	//alternative implementation could be printing each character at a time in two for loops...
	//alternative implementation could be good if using too much memory
		while (j <= 180){
			if (distances[(180 - j)/2] > 60.0){
				message[j] = '*';
				message[j+1] = '*';
			} else if (distances[(180 - j)/2] > 50.0) {
				message[j] = '-';
				message[j+1] = '-';
			} else if (distances[(180 - j)/2] > 35.0) {
				message[j] = '=';
				message[j+1] = '=';
			} else if (distances[(180 - j)/2] > 20.0) {
				message[j] = '@';
				message[j+1] = '@';
			}
			else if (distances[(180 - j)/2] > 4.0){
				message[j] = '#';
				message[j+1] = '#';
			}
			else {
			    message[j] = '*';
			    message[j+1] = '*';
			}
			j += 2;
		}
		message[j + 1] = 0;
		j = 0;
		for (j = 0; j < 4; j++){
		    uart_sendChar('\r');
		    uart_sendChar('\n');
		    send_message(message);
		}
		uart_sendChar('\r');
		uart_sendChar('\n');
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
    smallest[1] = (int)((smallestObjectDistance - 8) * 10);
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
    //float avgDistReal = 0.0;
    int averageAmount = 1;
    int i;
    for (i = 0; i < averageAmount; i++){
        cyBOT_Scan(angle, data); //points to target angle
        avgDistRaw += adc_read(); //adds to average
    }
    avgDistRaw = avgDistRaw / (averageAmount * 1.0);
    //clean actual data using line best fit polynomial 2
    avgDistRaw = 86.9 - (avgDistRaw * 0.0633) + (0.0000125 * (avgDistRaw * avgDistRaw));;
    return avgDistRaw;
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
    cyBOT_init_Scan(0b011); //enable ping and servo (IR IN ADC)
	adc_init(); //initialize IR
    cyBOT_Scan_t scan;

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
    char receiving = 0;
    char commandIndex = 0;


    //do calibration
    //cyBOT_SERVRO_cal_t calibrated_vals;
    //calibrate(&calibrated_vals);
    right_calibration_value = 269500; // OR calibrated_vals.right;
    left_calibration_value = 1230250;//calibrated_vals.left;

    while(input_byte != ESCAPE_KEY){
        oi_update(sensor_data);
        //recieving byte (non blocking
        while(!flag){} //blocking
        commandIndex = 0;
        while(input_byte != '\n'){
            command[commandIndex] = input_byte = uart_data;
            commandIndex++;
            while(!flag){} //block for next data
        }
        flag = 0;
        input_byte = command[0];

        if (input_byte == 'h'){
            sprintf(message, "done\n");
            send_message(message);
            continue;
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
                sprintf(message, "Entering Autonomous Mode, press 'h' to scan or 't' to return to manual\r\n");
                send_message(message);
                while(!flag){} //blocking
                flag = 0;
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

                scan_area(&scan, message, distanceIR);
                cleanData(distanceIR, message, &scan, smallestObject);
                printGraphicalView(distanceIR, message);
                smallestObject[0] = smallestObject[0] - 90;
				//wait here for user autonomous confirmation
                //TODO rn the target distance has a rounding error...
				sprintf(message, "Target Angle: %d    Target Distance: %d\r\nProceed to target?\r\n", smallestObject[0] * -1, (smallestObject[1] - 10) * 10);
				send_message(message);
				sprintf(message, "User options: 'h' proceed, 't' go to manual drive, other char will scan again: ");
				send_message(message);
				sprintf(message, "done\n");
				send_message(message);
				//reset flag to prevent previous inputs from interfereing
				input_byte = uart_data;
				flag = 0;
				//blocks until input
		        while(!flag){} //blocking
		        flag = 0;
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
				scan_area(&scan, message, distanceIR);
				//cleanData(distanceSound, message, &scan);
				cleanData(distanceIR, message, &scan, smallestObject);
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
			    cyBOT_Scan(90, &scan);
			    cyBOT_Scan(90, &scan);
			    sprintf(message, "Ping Distance: %3.3f", scan.sound_dist - 5);
			    send_message(message);
			    input_byte = 0;
			}
        }
    }
    oi_free(sensor_data);
}



