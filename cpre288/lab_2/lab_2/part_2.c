/*
 * part_2.c
 *
 *  Created on: Sep 12, 2024
 *      Author: tbibus
 */
/**
#include "open_interface.h"
#include "stdio.h"

void turn_clockwise(oi_t* sensor, int degrees){
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

void move_forward(oi_t* sensor, int distance){
    double sum = 0;

    oi_setWheels(100,100); //1/5 speed ahead!

    while(sum < distance){
        oi_update(sensor);
        sum += sensor->distance;

    }

    oi_setWheels(0,0); //stops bot

}

void main(){
    oi_t* sensor_data = oi_alloc();
    oi_init(sensor_data); //change led color
    move_forward(sensor_data, 500);
    turn_clockwise(sensor_data, 90);
    move_forward(sensor_data, 500);
    turn_clockwise(sensor_data, 90);
    move_forward(sensor_data, 500);
    turn_clockwise(sensor_data, 90);
    move_forward(sensor_data, 500);
    turn_clockwise(sensor_data, 90);

    oi_free(sensor_data);

}
*/

