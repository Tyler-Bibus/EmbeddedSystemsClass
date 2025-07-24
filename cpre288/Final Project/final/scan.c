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

//gets raw IR data
void scan_area(scan_data *data, char message[], float distIR[]){
    int angle = 0;
    float irDist;
    for (angle = 0; angle <= 180; angle += 2){
        scan(angle, data);
        irDist = data->ir_Raw;
        irDist = 86.9 - (irDist * 0.0633) + (0.0000125 * (irDist * irDist));
        distIR[angle / 2] = irDist;
    }
}

