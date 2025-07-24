/**
 * lab6_template_extra_help.c
 *
 * Description: This is file is meant for those that would like a little
 *              extra help with formatting their code.
 *
 */

#define _RESET 0
#define _PART1 0
#define _PART2 0
#define _PART3 0

#include "timer.h"
#include "lcd.h"
#include "uart.h"

#if _RESET
#include "Simulation_User Files_resetSimulation.h"
#endif

// Adding global volatile varibles for communcating between 
// your Interupt Service Routine, and your non-interupt code.

volatile  char uart_data;  // Your UART interupt code can place read data here
volatile  char flag;       // Your UART interupt can update this flag
                           // to indicate that it has placed new data
                           // in uart_data                     

/*
void main()
{
#if _RESET
    resetSimulationBoard();
#else

    timer_init();
    lcd_init();

#if _PART1      // Receive and display text
    uart_init();
    char string[21];
    int i = 0;
    char data = 0;
    while (1){
        if (!(UART1_FR_R & 0b1000) && (UART1_FR_R & 0b1000000)){
            data = uart_receive();
            uart_transmit(data);
            uart_transmit('\n';)
            if (data == '\r'){
                lcd_printf(string);
                //uart_sendStr(string);
                string[0] = 0; //reset null byte
                i = 0; //reset counter
                continue;
            }
            string[i] = data;
            string[i + 1] = 0;
            i++;
            if (i > 19){
                lcd_printf(string);
                //uart_sendStr(string);
                string[0] = 0; //reset null byte
                i = 0; //reset counter
            }
        }
    }
	

#endif

#if _PART2      // Echo Received Character
    uart_init();
    char string[21];
    int i = 0;
    char data = 0;
    while (1){
        if (!(UART1_FR_R & 0b1000) && (UART1_FR_R & 0b1000000)){
            data = uart_receive();
            if (data == '\r'){
                lcd_printf(string);
                //uart_sendStr(string);
                string[0] = 0; //reset null byte
                i = 0; //reset counter
                uart_sendChar('\n');
                continue;
            }
            uart_sendChar(data);
            string[i] = data;
            string[i + 1] = 0;
            i++;
            if (i > 19){
                lcd_printf(string);
                //uart_sendStr(string);
                string[0] = 0; //reset null byte
                i = 0; //reset counter
            }
        }
    }

#endif

#if _PART3 // Receive and send characters using interrupts.
    uart_init();
    init_char_interrupt();
    char string[21];
    int i = 0;
    char data = 0;
    while (1){
        if (flag){
            data = uart_data;
            if (data == '\r'){
                lcd_printf(string);
                //uart_sendStr(string);
                string[0] = 0; //reset null byte
                i = 0; //reset counter
                uart_sendChar('\n');
                flag = 0;
                continue;
            }
            uart_sendChar(data);
            string[i] = data;
            string[i + 1] = 0;
            i++;
            if (i > 19){
                lcd_printf(string);
                //uart_sendStr(string);
                string[0] = 0; //reset null byte
                i = 0; //reset counter
            }
            flag = 0;
        }
    }
 
#endif

#endif
}
*/
