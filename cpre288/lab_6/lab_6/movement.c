/*
 * movement.c
 *
 *  Created on: Sep 13, 2024
 *      Author: tbibus
 */
#include "open_interface.h"
#include "stdio.h"

void move_forward(oi_t* sensor, int distance){
    double sum = 0;

    oi_setWheels(100,100); //1/5 speed ahead!

    while(sum < distance){
        oi_update(sensor); //causes faultISR after scan???
        sum += sensor->distance; //integrates distance

    }
    oi_setWheels(0,0); //stops bot
}

void move_backwards(oi_t* sensor, int distance){
    double sum = 0;

    oi_setWheels(-100,-100); //1/5 speed ahead!
    distance = distance * -1; //makes distance negative for while loop
    while(sum > distance){
        oi_update(sensor);
        sum += sensor->distance; //integrates distance
    }

    oi_setWheels(0,0); //stops bot
}

void turn_counterclock(oi_t* sensor, int degrees){
    double sum = 0;

    oi_setWheels(100,-100); //turn clockwise slow

    while(sum < degrees - 15){
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(15,-15);
    //-2 for integration overshoot
    while(sum < degrees - 2){
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(0,0); //stops turn
}

void turn_clockwise(oi_t* sensor, int degrees){
    double sum = 0;

    oi_setWheels(-100,100); //turn clockwise slow
    degrees = degrees * -1;

    while(sum > degrees + 15){
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(-15,15);
    //+2 degrees for integration overshoot
    while(sum > degrees + 2){
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(0,0); //stops turn
}


