/*
 * scan.h
 *
 *  Created on: Nov 13, 2024
 *      Author: tbibus
 */

#ifndef SCAN_H_
#define SCAN_H_

#include "adc.h"
#include "ping.h"
#include "servo.h"

typedef struct{
    float sound_dist;
    int ir_Raw;
} scan_data;

//moves servo and gets the distance from object
void scan(float angle, scan_data *data);

//scans from 0 to 180, uses IR data TODO add irscan into this
void scan_area(scan_data *data, char message[], float distIR[]);

void scan_init();

#endif /* SCAN_H_ */
