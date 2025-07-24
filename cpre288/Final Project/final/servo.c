/*
 * servo.c
 *
 *  Created on: Nov 7, 2024
 *      Author: tbibus
 */

#include "servo.h"
#include <math.h>
#include "timer.h"
#include "button.h"
#include "lcd.h"

//Default Calibration Values
int LEFT_CAL_VAL = 36088;
int RIGHT_CAL_VAL = 8000;

extern volatile int button_event;
extern volatile int button_num;

void servo_init(void){

    const char PIN5 = 0b100000;

    //system clock
    SYSCTL_RCGCTIMER_R |= 0b10; //enable system timer
    SYSCTL_RCGCGPIO_R |= 0b10; //enable port B

    //GPIO set for Timer 1B GPIO PB5
    GPIO_PORTB_DEN_R |= PIN5; //enable port 5
    GPIO_PORTB_AFSEL_R |= PIN5; //set to alt function on port 5
    GPIO_PORTB_PCTL_R |= 0x700000;//set to TC1
    GPIO_PORTB_DIR_R |= PIN5; //set direction (output)

    //Set timer to 16-bit PWM with 8-bit prescale, count down, periodic mode
    TIMER1_CTL_R &= ~0x100; //disable timer A and B
    TIMER1_CTL_R = (TIMER1_CTL_R & ~0x6F) | 0x4C00; //set desired functionality both edges
    TIMER1_CFG_R = (TIMER1_CFG_R & ~0x7) | 0x4; //set to 16 bit with 8 bit prescale
    TIMER1_TBMR_R = (TIMER1_TBMR_R & ~0xFFF) | 0b1010; //set mode
    TIMER1_TBILR_R = (TIMER1_TBILR_R & ~0xFFFF) | 0xE200; //set upper bound (confirm val)
    TIMER1_TBPR_R  = (TIMER1_TBPR_R & 0x00) | 0x04; //set PS upper bound total is 20ms
    TIMER1_TBMATCHR_R = 0x4E200; //Match register (set to not move servo)
    TIMER1_CTL_R |= 0x100; //enable timer
}

int servo_move(float angle){

    const int ANGLE_MOD = 100; //minimum time between moves
    const float ANGLE_MULTIPLIER = 12; //max time, ideally 3s for 0 -> 180, but shorter as distance decreases

    //get previous angle
    int previous_match = TIMER1_TBMATCHR_R;
    float prev_angle = 0.0;
    prev_angle = ((previous_match - RIGHT_CAL_VAL) / ((LEFT_CAL_VAL - RIGHT_CAL_VAL) / 180.0));

    //NEW METHOD USED 180 MATCH and 0deg MATCH
    int match_val = (int) ((((LEFT_CAL_VAL - RIGHT_CAL_VAL) / 180.0) * angle) + RIGHT_CAL_VAL);

    TIMER1_TBMATCHR_R = match_val;

    //wait for certain time...
    timer_waitMillis((int)(ANGLE_MULTIPLIER * (abs(angle - prev_angle)) + ANGLE_MOD));

    return match_val;
}


/**
 * KEY
 * button 1: move left 1 degree
 * button 2: move right 1 degree
 * button 3: set left then right calibration value (loops back and forth)
 * button 4: exit
 */
void servo_calibrate(){
    int angle = 177; //start biased towards left
    int matchVal = 0;
    int direction = 0; // 0 = set left val, 1 = set right val
    int exitLoop = 0;

    while(!exitLoop){
        //wait for button presses
        while(!button_event && button_num == 0){}
        button_event = 0;
        switch(button_num){
            case 1:
                angle++; //move 1 degree
                break;
            case 2:
                angle--; //move 1 degrees
                break;
            case 3:
                if(direction % 2 == 1){
                    //set right val
                    RIGHT_CAL_VAL = matchVal;
                    angle = 180; //shortcut to set next val
                }
                else{
                    //set left val
                    LEFT_CAL_VAL = matchVal;
                    angle = 0; //shortcut
                }
                direction++;
                timer_waitMillis(500);
                break;
            case 4:
                exitLoop = 1;
                break;
            default:
                break;
        }
        button_num = 0;
        matchVal = servo_move(angle);
        lcd_printf("Match: %d\nDir: %d", matchVal, direction % 2);
    }
}
