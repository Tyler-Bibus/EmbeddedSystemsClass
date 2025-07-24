/**
 * This contains methods for cleaning data and object detection
 * @author Tyler Bibus
 */

#include "scan.h"
#include "dataClean.h"
#include "comms.h"
#include <stdio.h>
#include <math.h>

void cleanData(float uncleanData[], char message[], scan_data *data){
    //First, remove/smooth outliers, assume all values are positive
    char i;
    char j;
    char k;
    float bounds = 5; //max variation of 5cm (could maybe make more conservative?)
    char object = 0; //boolean
    float MAX_RANGE = 63.0;
    int MIN_GAP = 3;
    for (i = 0; i < 90 - MIN_GAP; i++){
        //if drops suddenly, considered object until distance increases again
        if (fabs(uncleanData[i] - uncleanData[i + 1]) < bounds && uncleanData[i] < MAX_RANGE){
            //checks next 4 degrees within bounds
            j = MIN_GAP;
            while (!object && j >= 1){
                //this is an object (within bounds)
                if (fabs(uncleanData[i] - uncleanData[i + j]) < bounds){
                    //smooth out object
                    for (k = 1; k <= j; k++){
                        uncleanData[i + k] = uncleanData[i];
                        object = 1; //exit while loop early
                    }
                }
                j--;
            }
            if (!object){
                uncleanData[i] = uncleanData[i+1]; //smooth data
            }
        }
        else if(uncleanData[i] > MAX_RANGE){
            uncleanData[i] = 70; //ignore distance past 59cm
        }
        object = 0;
    }
    //gets objects
    objectDetection(uncleanData, message, data);
    return;
}

void objectDetection(float distances[], char message[], scan_data *data){
    char i;
    //char object = 0; //keep if we want to keep track of # of objects
    char startAngle = 0;
    char endAngle = 0;
    float objectDistance = 0;
    float objectSize = 0;
    for (i = 3; i < 90 - 1; i++){
        //start object
        if ((distances[i] < 65.0) && (distances[i-1] - distances[i]) > 1.0){
            startAngle = i * 2;
            endAngle = startAngle;
            while(fabs(distances[i] - distances[i+1]) < 1.0 || (distances[i] - distances[i+1]) > 1.0){
                i++;
            }
            endAngle = i * 2;
            if (startAngle == endAngle){
                continue;
            }
            //get object distance (scan 2x)
            scan((startAngle + endAngle)/2 , data);
            scan((startAngle + endAngle)/2 , data);
            objectDistance = data->sound_dist;
            objectSize = tan((((endAngle - startAngle) * M_PI) / 180.0) / 2.0) * objectDistance * 2; //I hate this equation (size)
            //if object size smaller than 3 ignore
            if (objectSize <= 3.0){
                continue;
            }

            //print off angle (for debug uncomment)
            //sprintf(message, "%d %d 10\n", startAngle, endAngle);
            //send_message(message);

            //print off object
            sprintf(message, "%d %3.3f %3.3f\n", (startAngle + endAngle) / 2, objectDistance, objectSize);
            send_message(message);
        }
    }
    return;
}
