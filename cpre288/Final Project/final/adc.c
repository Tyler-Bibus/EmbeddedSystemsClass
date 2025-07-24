/*
* @author Tyler Bibus
*
*/
#include "adc.h"
#include <inc/tm4c123gh6pm.h>

//will initialize GPIO and ADC for IR sensor
void adc_init(void){
	/**
	 * INITIALIZE GPIO
	 */
	//enable clock for port B
	SYSCTL_RCGCGPIO_R |= 0b10;
	
    //ADC run mode for ADC 0
    SYSCTL_RCGCADC_R |= 0b1;

	//set PB4 to alt function
	GPIO_PORTB_AFSEL_R |= 0b10000;
	
	// set PB4 to output with bit preservation
	GPIO_PORTB_DIR_R &= 0xFFFFFFEF;
	
	//disable digital function PB4
	GPIO_PORTB_DEN_R &= 0xFFFFFFEF;
	
	//enable analog function
	GPIO_PORTB_AMSEL_R |= 0b10000;
	
	//Write ADC control to 0x00
	GPIO_PORTB_ADCCTL_R &= 0xFFFFFF00;
	
	/**
	 * INITIALIZE ADC
	 */
	//Active Sample Sequence SS1
	ADC0_ACTSS_R |= 0b1;
	
	//set Event Mux (with bit preservation) software driven
	ADC0_EMUX_R &= 0xFFFFFFF0;
	
    //maybe need to set input mux? pin 4? Clears Mux 1 and writes 4 into mux1
    ADC0_SSMUX0_R = (ADC0_SSMUX0_R & 0xFFFFFFF0) | 0xA;

	//Set sample 1 to end of sequence
	ADC0_SSCTL0_R = (ADC0_SSCTL0_R & 0xFFFFFFF0) | 0x6;

	//hardware averageing w/ bit pres
	ADC0_SAC_R = (ADC0_SAC_R & 0xFFFFFFF8) | 0x4;

	//set Interrupt mask ss0
	ADC0_IM_R |= 0b1;
	
	//clear interrupt status
	ADC0_ISC_R |= 0b1;
}

//Reads ADC Value
int adc_read(void){
	
	//start sampling ss0
	ADC0_PSSI_R |= 0b1;
	
	while (~ADC0_RIS_R & 0b1){
		//wait for conversion to be completed
	}
	
	//clear interrupt status
	ADC0_ISC_R |= 0b1;
	
	//return the ADC value
	return ADC0_SSFIFO0_R & 0x00000FFF;
}
