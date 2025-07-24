/*
 * movement.h
 *
 *  Created on: Sep 13, 2024
 *      Author: tbibus
 */
//moves the bot forward by distance in mm
void move_forward(oi_t* sensor, int distance);

//moves the bot backwards by distance in mm
void move_backwards(oi_t* sensor, int distance);

//turns the bot counterclockwise by degrees
void turn_counterclock(oi_t* sensor, int degrees);

//turns the bot clockwise by degrees
void turn_clockwise(oi_t* sensor, int degrees);


