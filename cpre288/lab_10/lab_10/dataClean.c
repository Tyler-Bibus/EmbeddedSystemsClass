/**
 * This contains methods for cleaning data and object detection
 * @author Tyler Bibus
 */

#include "scan.h"
#include "dataClean.h"
#include "comms.h"
#include <stdio.h>
#include <math.h>

void cleanData(float uncleanData[], char message[], scan_data *data, int smallest[]){
    //First, remove/smooth outliers, assume all values are positive
    char i;
    char j;
    char k;
    float bounds = 5;
    char object = 0; //boolean
    for (i = 0; i < 90 - 3; i++){
        //new method, if dramatically, see if within 10 degrees if it returns to low distance
        if (fabs(uncleanData[i] - uncleanData[i + 1]) < bounds && uncleanData[i] < 59.0){
            //checks next 6 degrees within bounds
            j = 3;
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
                //remove drop
                uncleanData[i] = uncleanData[i+1];
            }
        }
        else if(uncleanData[i] > 59.0){
            uncleanData[i] = 70; //ignore
        }
        object = 0;
    }
   sprintf(message, "start\n");
   send_message(message);
   for (i = 0; i <= 90; i++){
        sprintf(message, "%d %3.3f",i * 2, uncleanData[i] / 100);
        send_message(message);
    }
    //gets objects
    objectDetection(uncleanData, message, data, smallest);
    return;
}

void objectDetection(float distances[], char message[], scan_data *data, int smallest[]){
    char i;
    //char object = 0; //keep if we want to keep track of # of objects
    char startAngle = 0;
    char endAngle = 0;
    float objectDistance = 0;
    float objectSize = 0;
    float smallestObjectSize = 0.0;
    int smallestObjectAngle = 0;
    float smallestObjectDistance;
    for (i = 3; i < 90 - 1; i++){
        //start object
        if ((distances[i] < 60.0) && (distances[i-1] - distances[i]) > 1.0){
            startAngle = i * 2;
            endAngle = startAngle;
            while(fabs(distances[i] - distances[i+1]) < 1.0 || (distances[i] - distances[i+1]) > 1.0){
                i++;
            }
            endAngle = i * 2;
            if (startAngle == endAngle){
                continue;
            }
            //get object distance
            scan((startAngle + endAngle)/2 , data);
            scan((startAngle + endAngle)/2 , data);
            scan((startAngle + endAngle)/2 , data);
            objectDistance = data->sound_dist;
            objectSize = tan((((endAngle - startAngle) * M_PI) / 180.0)) * objectDistance;
            //print off object
            //sprintf(message, "\n\rSome obj angle:%d  Start: %d   End: %d \n\r", (startAngle + endAngle) / 2, startAngle, endAngle);
            //send_message(message);
            //sprintf(message, "Some obj distance:%3.3f \n\r", objectDistance);
           // send_message(message);
           // sprintf(message, "Some obj size:%3.3f \n\r", objectSize);
            //send_message(message);
            if (smallestObjectSize <= 0.01 || smallestObjectSize > objectSize && !(objectSize < 0.01)){
                smallestObjectSize = objectSize;
                smallestObjectAngle = (startAngle + endAngle) / 2;
                smallestObjectDistance = objectDistance;
            }
        }
    }
    //sprintf(message, "%d   %3.3f", smallestObjectAngle, smallestObjectDistance);
    //send_message(message);
    scan(smallestObjectAngle, data);
    //sprintf(message, "\n\rsmallest obj angle:%d \n\r", smallestObjectAngle - 90);
    //send_message(message);
    //sprintf(message, "smallest obj distance:%3.3f \n\r", smallestObjectDistance);
   // send_message(message);
    smallest[0] = smallestObjectAngle;
    smallest[1] = (int)((smallestObjectDistance - 8) * 10);
    return;
}
