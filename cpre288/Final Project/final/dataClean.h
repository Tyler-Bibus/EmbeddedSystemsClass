/*
 * dataClean.h
 *
 *  Created on: Nov 13, 2024
 *      Author: tbibus
 */

#ifndef DATACLEAN_H_
#define DATACLEAN_H_

#include "scan.h"

void cleanData(float uncleanData[], char message[], scan_data *data);

void objectDetection(float distances[], char message[], scan_data *data);



#endif /* DATACLEAN_H_ */
