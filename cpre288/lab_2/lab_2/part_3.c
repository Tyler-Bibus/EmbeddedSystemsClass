/*
 * part_3.c
 *
 *  Created on: Sep 12, 2024
 *      Author: tbibus
 */
/**
#include "open_interface.h"
#include "stdio.h"

void go_around_left(oi_t* sensor_bot, int recursion);
void go_around_right(oi_t* sensor_bot, int recursion);
void turn_clockwise(oi_t* sensor, int degrees);
void turn_counterclock(oi_t* sensor, int degrees);
void move_forward(oi_t* sensor, int distanc, int recursione);
void move_backwards(oi_t* sensor, int distance);

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
    //-2 for integration overshoot
    while(sum > degrees + 2){
        oi_update(sensor);
        sum += sensor->angle;
    }

    oi_setWheels(0,0); //stops turn
}

void move_forward(oi_t* sensor, int distance, int recursion){
    double sum = 0;
    int bumpDetectedLeft = 0;
    int bumpDetectedRight = 0;

    oi_setWheels(100,100); //1/5 speed ahead!

    while(sum < distance){
        oi_update(sensor);
        sum += sensor->distance;
        bumpDetectedLeft = sensor->bumpLeft;
        bumpDetectedRight = sensor->bumpRight;
        if (bumpDetectedLeft == 1){
            go_around_left(sensor, recursion);
            bumpDetectedLeft = 0;
            bumpDetectedRight = 0;
            sum += -150;
            sensor->distance;
            oi_setWheels(100,100); //reset speed
        }
        if (bumpDetectedRight == 1){
            go_around_right(sensor, recursion);
            bumpDetectedLeft = 0;
            bumpDetectedRight = 0;
            sum += -150;
            sensor->distance;
            oi_setWheels(100,100); //reset speed
        }
    }
    oi_setWheels(0,0); //stops bot
}

void move_backwards(oi_t* sensor, int distance){
    double sum = 0;

    oi_setWheels(-100,-100); //1/5 speed ahead!
    distance = distance * -1;
    while(sum > distance){
        oi_update(sensor);
        sum += sensor->distance;
    }

    oi_setWheels(0,0); //stops bot
}

//limit recursion to 2
void go_around_left(oi_t* sensor_bot, int recursion){
    recursion++;
    if (recursion > 3){
        return;
    }
    move_backwards(sensor_bot, 150);
    turn_clockwise(sensor_bot,90);
    move_forward(sensor_bot, 250, recursion);
    turn_counterclock(sensor_bot,90);
}

void go_around_right(oi_t* sensor_bot, int recursion){
    recursion++;
    if (recursion > 3){
        return;
    }
    move_backwards(sensor_bot, 150);
    turn_counterclock(sensor_bot,90);
    move_forward(sensor_bot, 250, recursion);
    turn_clockwise(sensor_bot,90);
}

void main(){
    oi_t* sensor_data = oi_alloc();
    oi_init(sensor_data); //change led color
    move_forward(sensor_data, 2000, 0);


    oi_free(sensor_data);

}
*/

