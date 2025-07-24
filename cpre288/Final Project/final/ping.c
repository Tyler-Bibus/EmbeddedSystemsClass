/*
 * ping.c
 *
 *  Created on: Oct 31, 2024
 *      Author: tbibus
 */

#include "ping.h"
#include "timer.h"

volatile enum {LOW, HIGH, DONE} state; //set by ISR
volatile unsigned int rising_time; //start time
volatile unsigned int falling_time; //end time
volatile unsigned int overflow_count; //overflow count

void ping_init(){

    state = LOW;

    SYSCTL_RCGCTIMER_R |= 0x8; //enable system timer

    SYSCTL_RCGCGPIO_R |= 0b10; //enable port B

    //GPIO stuff
    GPIO_PORTB_DEN_R |= 0x8;
    GPIO_PORTB_AFSEL_R |= 0x8;
    GPIO_PORTB_PCTL_R |= 0x7000;
    GPIO_PORTB_DIR_R |= 0x8;

    //TIMER Stuff

    TIMER3_CTL_R &= ~0x1100; //disable timer3 B

    TIMER3_CTL_R |= 0xC00; //Disable Capture Interrupt (write to 0)

    TIMER3_CFG_R = (TIMER3_CFG_R & ~0x7) | 0x4; //16 bit mode

    TIMER3_TBMR_R = (TIMER3_TBMR_R & ~0b110011000) | 0b00111; //Capture mode, edge time mode, count down

    TIMER3_TBILR_R |= 0xFFFF; //set upper bound (max value)

	TIMER3_TBPR_R |= 0xFF; //this might not be right...

    TIMER3_IMR_R |= 0x400; //Enable Capture Interrupt

    TIMER3_CTL_R |= 0x100; //re-enable timer

    NVIC_EN1_R |= 0x10;

    IntRegister(INT_TIMER3B, TIMER3B_Handler); //Bind interrupt
    IntMasterEnable();

}

//capture ping response start and end time
void TIMER3B_Handler(void){

    while(~TIMER3_MIS_R & 0x400){} //Wait for interrupt status

    TIMER3_ICR_R |= 0x400; //clear interrupt (confirm register)

    if(state == LOW){
        rising_time = ((TIMER3_TBPS_R & 0xFF) << 16) + (TIMER3_TBR_R & 0xFFFF); //get clock value TODO (CONFIM THIS)
        state = HIGH;
    }
    else if(state == HIGH){
        falling_time = ((TIMER3_TBPS_R & 0xFF) << 16) + (TIMER3_TBR_R & 0xFFFF); //get clock value TODO (CONFIM THIS)
        state = DONE;
    }
    return;
}

unsigned int ping_read(void) {
    send_pulse(); //request PING

    while(state != DONE){}

    if (falling_time > rising_time){
        rising_time += 0xFFFF;
        overflow_count++;
    }


    return rising_time - falling_time;
}

void send_pulse(void){
    state = LOW;

    TIMER3_IMR_R &= ~0x400; //Disable Capture Interrupt (write to 0)

    GPIO_PORTB_AFSEL_R &= ~0x8; //set to digitally controlled

    GPIO_PORTB_DIR_R |= 0b1000;  //set PB3 as out
    GPIO_PORTB_DATA_R |= 0b1000; //set PB3 to high

    timer_waitMicros(5);   //WAIT 5 us

    GPIO_PORTB_DATA_R &= ~0b1000; //set PB3 to low
    GPIO_PORTB_DIR_R &= ~0b1000;  //set PB3 as input

    GPIO_PORTB_AFSEL_R |= 0x8; //set to alternative function control
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~0xF000) | 0x7000; //set to timer controlled TODO is this what is broken?

    TIMER3_ICR_R |= 0x400; //clear interrupt (confirm register)
    TIMER3_IMR_R |= 0x400; //Enable Capture Interrupt (write to 1)
    return;
}
