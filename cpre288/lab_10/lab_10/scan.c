/**
 * holds methods for scanning for getting ping, raw, servo data
 * @author Tyler Bibus
 */

#include "adc.h"
#include "servo.h"
#include "ping.h"
#include "comms.h"
#include <stdio.h>

#define SPEED_SOUND 34.0
#define TIME_PER_CLOCK 0.0000625

extern volatile int LEFT_CAL_VAL;
extern volatile int RIGHT_CAL_VAL;

typedef struct{
    float sound_dist;
    int ir_Raw;
} scan_data;

void scan_init(){
    adc_init();
    servo_init();
    ping_init();
    servo_move(90);//set to 90 initially
}

void scan(float angle, scan_data *data){
    //point servo
    servo_move(angle);
    data->ir_Raw = adc_read();
    unsigned int travel_time = ping_read();
    data->sound_dist = ((travel_time * TIME_PER_CLOCK) * SPEED_SOUND) / 2.0;
}

void scan_area(scan_data *data, char message[], float distIR[]){
    int angle = 0;
    float irDist;
    //iterates and prints noisy data
    for (angle = 0; angle <= 180; angle += 2){
        scan(angle, data);
        irDist = data->ir_Raw;
        irDist = 86.9 - (irDist * 0.0633) + (0.0000125 * (irDist * irDist));
        //cyBOT_Scan(angle, scan);
        //sprintf(message, "%d  %3.3f", angle, irDist / 100.0);
        //send_message(message);
        //distances[angle / 2] = scan->sound_dist;
        distIR[angle / 2] = irDist;
    }
}

void calibrate(int LEFT_VAL, int RIGHT_VAL){
    LEFT_CAL_VAL = LEFT_VAL;
    RIGHT_CAL_VAL = RIGHT_VAL;
}
