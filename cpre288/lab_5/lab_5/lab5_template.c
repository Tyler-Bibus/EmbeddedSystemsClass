/**
 * lab5_template.c
 * 
 * Template file for CprE 288 lab 5
 *
 * @author Zhao Zhang, Chad Nelson, Zachary Glanz
 * @date 08/14/2016
 *
 * @author Phillip Jones, updated 6/4/2019
 */
/*
#include "button.h"
#include "timer.h"
#include "lcd.h"

#include "cyBot_uart.h"  // Functions for communiticate between CyBot and Putty (via UART)
                         // PuTTy: Buad=115200, 8 data bits, No Flow Control, No Party,  COM1

#include "cyBot_Scan.h"  // For scan sensors 

#define REPLACEME 0


// Defined in button.c : Used to communicate information between the
// the interupt handeler and main.
extern volatile int button_event;
extern volatile int button_num;

void send_message(char message[]){
    //sends each character of the message array
    while (*message != 0){
        cyBot_sendByte(*message);
        message++;
    }
    cyBot_sendByte('\n');
    //cyBot_sendByte('\r');
    return;
}


void irScan(cyBOT_Scan_t *data, char *message){
    float avgDistRaw = 0.0;
    float avgDistReal = 0.0;
    int i;
    for (i = 0; i < 5; i++){
        cyBOT_Scan(90, data);
        avgDistRaw += data->IR_raw_val;
    }
    avgDistRaw = avgDistRaw / 4.0;
    //clean actual data using line best fit polynomial 2
    avgDistReal = 114 + (-0.0653 * avgDistRaw) + (0.00000995 * avgDistRaw * avgDistRaw);
    sprintf(message, "%3.3f \n %4.2f", avgDistRaw, avgDistReal);
    send_message(message);
    lcd_printf(message);
}


int main(void) {
    timer_init();
	button_init();
	lcd_init();
	//cyBot_uart_init();
	init_button_interrupts();
	            // Don't forget to initialze the cyBot UART before trying to use it
	

    cyBot_uart_init_clean();  // Clean UART initialization, before running your UART GPIO init code

	// Complete this code for configuring the  (GPIO) part of UART initialization
     SYSCTL_RCGCGPIO_R |= 0b00000011;
     timer_waitMillis(1);            // Small delay before accessing device after turning on clock
     GPIO_PORTB_AFSEL_R |= 0b00000011;
     GPIO_PORTB_PCTL_R &= 0xFFFFFF00;     // Force 0's in the disired locations
     GPIO_PORTB_PCTL_R |= 0x00000011;     // Force 1's in the disired locations
     GPIO_PORTB_DEN_R |= 0b00000011;
     GPIO_PORTB_DIR_R &= 0b11111110;      // Force 0's in the disired locations
     GPIO_PORTB_DIR_R |= 0b00000010;      // Force 1's in the disired locataions
    
    cyBot_uart_init_last_half();  // Complete the UART device initialization part of configuration
	
	// Initialze scan sensors
    cyBOT_init_Scan(0b0111);
    cyBOT_Scan_t scanData;
    char message[50];



	// YOUR CODE HERE
		
	while(1)
	{
	
      //chillin

	    if (button_event){
	        lcd_printf("%d", button_num);
	        button_event = 0;
	        irScan(&scanData, message);
	    }

  


	
	}
	
}
*/
