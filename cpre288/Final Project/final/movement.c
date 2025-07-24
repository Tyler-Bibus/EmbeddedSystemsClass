/*
 * movement.c
 *
 *  Created on: Sep 13, 2024
 *      Author: tbibus
 */
#include "open_interface.h"
#include "stdio.h"
#include "comms.h"

//IRCLIFF DATA, BOUNDARY >= IRval of 2660
void move_forward(oi_t* sensor, int distance, char message[]){
    double sum = 0;
    int encounter = 0;
    oi_setWheels(130,130); //1/5 speed ahead!
    //go fast until within 10cm
    while(sum < distance - 100){
        oi_update(sensor);
        sum += sensor->distance; //integrates distance
        if(sensor->bumpLeft){
            encounter = 1;
            break;
        }
        if (sensor->bumpRight){
            encounter = 2;
            break;
        }
        if (sensor->cliffFrontLeft || sensor->cliffLeft){
            encounter = 3;
            break;
        }
        if (sensor->cliffFrontRight || sensor->cliffRight){
            encounter = 4;
            break;
        }
        if(sensor->cliffFrontLeftSignal > 2660 || sensor->cliffLeftSignal > 2660 || sensor->cliffRightSignal > 2660 || sensor->cliffFrontRightSignal > 2660){
            encounter = 5;
            break;
        }
    }
    //last 10ish cm go slow
    while(sum < distance && encounter == 0){
        oi_setWheels(50,50);
        oi_update(sensor);
        sum += sensor->distance; //integrates distance

        if(sensor->bumpLeft){
            encounter = 1;
            break;
        }
        if (sensor->bumpRight){
            encounter = 2;
            break;
        }
        if (sensor->cliffFrontLeft){
            encounter = 3;
            break;
        }
        if (sensor->cliffFrontRight){
            encounter = 4;
            break;
        }
        if(sensor->cliffFrontLeftSignal > 2660 || sensor->cliffFrontRightSignal > 2660){
            encounter = 5;
            break;
        }
    }
    oi_setWheels(0,0); //stops bot
    float offDistance = (distance - sum) / 10;
    //determine if bump happened, and the distance remaining
    switch(encounter){
    case 0:
        return;
    case 1:
            sprintf(message, "b %3.3f\n", offDistance);
            send_message(message);
            return;
    case 2:
            sprintf(message, "b %3.3f\n", offDistance);
            send_message(message);
            return;
    case 3:
            sprintf(message, "h %3.3f\n", offDistance);
            send_message(message);
            return;
    case 4:
            sprintf(message, "h %3.3f\n", offDistance);
            send_message(message);
            return;
    case 5:
            sprintf(message, "w %3.3f\n", offDistance);
            send_message(message);
            return;
    }
    return;
}

//NO BUMP DETECTION (its impossible)
//Moves backwards pretty slowly
void move_backwards(oi_t* sensor, int distance, char message[]){
    double sum = 0;
    oi_setWheels(-50,-50); //1/5 speed ahead!
    distance = distance * -1; //makes distance negative for while loop
    while(sum > distance){
        oi_update(sensor);
        sum += sensor->distance; //integrates distance
    }
    oi_setWheels(0,0); //stops bot
    return;

}

void turn_counterclock(oi_t* sensor, int degrees){
    double sum = 0;

    oi_setWheels(75,-75); //turn clockwise slow

    while(sum < degrees - 15){
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(20,-20);
    //-2 for integration overshoot
    while(sum < (float)degrees - 0.5){
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(0,0); //stops turn
}

void turn_clockwise(oi_t* sensor, int degrees){
    double sum = 0;

    oi_setWheels(-75,75); //turn clockwise slow
    degrees = degrees * -1;

    while(sum > degrees + 15){
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(-20,20);
    //+2 degrees for integration overshoot
    while(sum > (float)degrees + 0.5){
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(0,0); //stops turn
}


