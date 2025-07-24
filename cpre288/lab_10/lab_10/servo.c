/*
 * servo.c
 *
 *  Created on: Nov 7, 2024
 *      Author: tbibus
 */

#include "servo.h"
#include <math.h>
#include "timer.h"

volatile int LEFT_CAL_VAL = 36088;
volatile int RIGHT_CAL_VAL = 8000;

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
    //TODO set PWM stuff... not sure exactly how
    TIMER1_CTL_R &= ~0x100; //disable timer A and B
    TIMER1_CTL_R = (TIMER1_CTL_R & ~0x6F) | 0x4C00; //set desired functionality both edges (TODO confirm?)
    TIMER1_CFG_R = (TIMER1_CFG_R & ~0x7) | 0x4; //set to 16 bit with 8 bit prescale
    TIMER1_TBMR_R = (TIMER1_TBMR_R & ~0xFFF) | 0b1010; //set mode (TODO confirm no legacy (bit 11) option needed)
    TIMER1_TBILR_R = (TIMER1_TBILR_R & ~0xFFFF) | 0xE200; //set upper bound (confirm val)
    TIMER1_TBPR_R  = (TIMER1_TBPR_R & 0x00) | 0x04; //set PS upper bound total is 20ms
    TIMER1_TBMATCHR_R = 0x4E200; //Match register (set to not move servo)
    //TIMER1_IMR_R |= ; //set interrupt if needed
    TIMER1_CTL_R |= 0x100; //enable timer

    //NVIC_EN0_R |= ; //set NVIC for interrupt if needed

    //IntRegister(INT_TIMER1, TIMER1B_Handler); //bind indterrupt if needed
    //IntMasterEnable(); //set master enable if needed

}


//Cybot #9 match for 180 deg = 36088, 0 deg = 8000
int servo_move(float angle){

    const int ANGLE_MOD = 100; //minimum time between moves
    const float ANGLE_MULTIPLIER = 10; //max time, ideally 2s for 0 -> 180

    //catch potential outliers
    /*
    if (angle > 180){
        angle = 180.0;
    }
    if (angle < 0){
        angle = 0.0;
    }
    */

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
