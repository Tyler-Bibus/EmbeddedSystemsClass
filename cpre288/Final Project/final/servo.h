/*
 * servo.h
 *
 *  Created on: Nov 7, 2024
 *      Author: tbibus
 */

#ifndef SERVO_H_
#define SERVO_H_

void servo_init(void);

int servo_move(float angle);

void servo_calibrate(void);

#endif /* SERVO_H_ */
