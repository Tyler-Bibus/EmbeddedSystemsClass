/*
 * part_1.c
 *
 *  Created on: Sep 12, 2024
 *      Author: tbibus
 */

#include "open_interface.h"
#include "movement.h"


void main(){
    oi_t* sensor_data = oi_alloc();
    oi_init(sensor_data); //change led color

    move_forward(sensor_data, 1000);
    turn_clockwise(sensor_data, 90);

    oi_free(sensor_data);

}



