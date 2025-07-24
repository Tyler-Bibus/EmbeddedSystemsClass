/**
 * just contains autonomous navigation method imperfect
 * @author Tyler Bibus
 */

#include "scan.h"
#include "open_interface.h"
#include "movement.h"
#include "comms.h"
#include "uart.h"

volatile char uart_flag;
volatile char uart_data;

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
        if(uart_flag){
            user_input = uart_data;
            uart_flag = 0;
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
            //sprintf(message, "Bump on Left! navigating around...\r\n");
            //send_message(message);
            if(uart_flag){
                user_input = uart_data;
                uart_flag = 0;
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
               // sprintf(message, "Bump again! navigating around...\r\n");
                //send_message(message);
                continue; //repeat until clear
            }
            turn_counterclock(sensor_data, returnAngle + 20);
            previousBump = 0;
            //previousBump = moveWithBump(sensor_data, &returnDistance);
        }
        while(previousBump == 2){
            //sprintf(message, "Bump on Right! navigating around...\r\n");
           // send_message(message);
            if(uart_flag){
                user_input = uart_data;
                uart_flag = 0;
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
                //sprintf(message, "Bump again! navigating around...\r\n");
                //send_message(message);
                continue; //repeat until clear
            }
            turn_clockwise(sensor_data, returnAngle + 20);
            previousBump = 0;
            //previousBump = moveWithBump(sensor_data, &returnDistance);
        }

    }
}
