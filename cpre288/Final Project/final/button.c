/*
 * button.c
 *
 *  @author Tyler Bibus
 *
 *  @edit: Lindsey Sleeth and Sam Stifter on 02/04/2019
 *  @edit: Phillip Jone 05/30/2019: Mearged Spring 2019 version with Fall 2018
 */
 


//The buttons are on PORTE 3:0
// GPIO_PORTE_DATA_R -- Name of the memory mapped register for GPIO Port E, 
// which is connected to the push buttons
#include "button.h"
volatile int button_event;
volatile int button_num;

/**
 * Initialize PORTE and configure bits 0-3 to be used as inputs for the buttons.
 */
void button_init() {
	static uint8_t initialized = 0;

	//Check if already initialized
	if(initialized){
		return;
	}

	// 1) Turn on PORTE system clock, do not modify other clock enables
	SYSCTL_RCGCGPIO_R |= 0x0000010;

	// 2) Set the buttons as inputs, do not modify other PORTE wires
	GPIO_PORTE_DIR_R &= 0xFFFFFF0;
	
	// 3) Enable digital functionality for button inputs, 
	//    do not modify other PORTE enables
	GPIO_PORTE_DEN_R |= 0x0000000F;

	init_button_interrupts();
	
	initialized = 1;
}



/**
 * Returns the position of the rightmost button being pushed.
 * @return the position of the righttmost button being pushed. 4 is the rightmost button, 1 is the leftmost button.  0 indicates no button being pressed
 */
uint8_t button_getButton() {
	if (~(GPIO_PORTE_DATA_R) & 0b00001000){
	    return 4; //button 4
	}
	else if (~(GPIO_PORTE_DATA_R) & 0b00000100){
	    return 3; // button 3
	}
	else if (~(GPIO_PORTE_DATA_R) & 0b00000010){
	    return 2; //button 2
	}
	else if (~(GPIO_PORTE_DATA_R) & 0b00000001){
	    return 1; //button 1
	}
	return 0; // no button
}

/**
 * Initialize and configure PORTE interupts
 */
void init_button_interrupts() {

    // 1) Mask the bits for pins 0-3
    GPIO_PORTE_IM_R &= 0xFFFFFFF0;

    // 2) Set pins 0-3 to use edge sensing
    GPIO_PORTE_IS_R &= 0xFFFFFFF0;

    // 3) Set pins 0-3 to use both edges. We want to update the LCD
    //    when a button is pressed, and when the button is released.
    GPIO_PORTE_IBE_R |= 0x0000000F;

    // 4) Clear the interrupts
    GPIO_PORTE_ICR_R = 0x0000000F;

    // 5) Unmask the bits for pins 0-3
    GPIO_PORTE_IM_R |= 0x0000000F;

    // 6) Enable GPIO port E interrupt
    NVIC_EN0_R |= 0x00000050;

    // Bind the interrupt to the handler.
    IntRegister(INT_GPIOE, gpioe_handler);
    IntMasterEnable();
}


/**
 * Interrupt handler -- executes when a GPIO PortE hardware event occurs (i.e., for this lab a button is pressed)
 */
void gpioe_handler() {
    button_num = button_getButton();
    button_event = 1;
    GPIO_PORTE_ICR_R |= 0x0F;
}



