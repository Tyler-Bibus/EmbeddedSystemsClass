/*
 * cleaningTest.C
 *
 *  Created on: Sep 25, 2024
 *      Author: tbibus
 */
/*
#include <math.h>
#include <stdio.h>

void cleanData(float uncleanData[]){
    //First, remove/smooth outliers, assume all values are positive
    int i;
    float bounds = 0.015;
    for (i = 0; i < 17 - 2; i++){
        //check to see if greater than x out of bounds in both directions (outlier)
        if (fabs(uncleanData[i + 1] - uncleanData[i]) > bounds && fabs(uncleanData[i + 1] - uncleanData[i+2]) > bounds){
            //if out of bounds, set to average of two values
            uncleanData[i+1] = (uncleanData[i] + uncleanData[i+2]) / 2;
        }
    }
}

void printData(float uncleanData[]){
    int i = 0;
    while (i < 17){
        printf("%d  %f\n", i * 2, uncleanData[i]);
        i++;
    }
}

void main(){
    float uncleanedVals[17] = {1.24329, 0.65171, 0.76523, 0.81505, 1.19213, 1.10835, 1.27019, 1.24274, 1.46593, 1.42196, 1.18302, 1.29989, 1.44742, 1.05031, 1.07506, 0.94737, 1.21622};
    cleanData(uncleanedVals);
    printData(uncleanedVals);
}
*/

