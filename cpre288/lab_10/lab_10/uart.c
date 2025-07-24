/*
*
*   uart.c
*
*
*
*
*
*   @author
*   @date
*/

#include "uart.h"
#include "timer.h"
#define REPLACE_ME 0x00;

volatile  char uart_data;  // Your UART interupt code can place read data here
volatile  char uart_flag;       // Your UART interupt can update this flag
                           // to indicate that it has placed new data
                           // in uart_data

void uart_init(void){
    SYSCTL_RCGCGPIO_R |= 0b0010;      // enable clock GPIOB (page 340)
    SYSCTL_RCGCUART_R |= 0b0010;      // enable clock UART1 (page 344)
    GPIO_PORTB_AFSEL_R |= 0b0011;      // sets PB0 and PB1 as peripherals (page 671) TODO make sure |= is correct not =
    GPIO_PORTB_PCTL_R  = (GPIO_PORTB_PCTL_R & 0xFFFFFF00) | 0x11;       // pmc0 and pmc1       (page 688)  also refer to page 650
    GPIO_PORTB_DEN_R   = GPIO_PORTB_DEN_R | 0b11;        // enables pb0 and pb1
    GPIO_PORTB_DIR_R   = (GPIO_PORTB_DIR_R | 0b1) & 0xFD;        // sets pb0 as output, pb1 as input

    //compute baud values [UART clock= 16 MHz]
    double fbrd;
    int    ibrd;

    fbrd = 16000000 / (16 * 115200.0); // page 903
    ibrd = (int) fbrd;
    fbrd -= ibrd;

    UART1_CTL_R &= 0xFFFE;      // disable UART1 (page 918)
    UART1_IBRD_R = ibrd;        // write integer portion of BRD to IBRD
    UART1_FBRD_R = (int)(fbrd * 64 + 0.5);   // write fractional portion of BRD to FBRD
    UART1_LCRH_R = 0b01100000;        // write serial communication parameters (page 916) * 8bit and no parity
    UART1_CC_R   = 0x0;          // use system clock as clock source (page 939)
    UART1_CTL_R |= 0b1;        // enable UART1
}

void uart_sendChar(char data){
	while (~UART1_FR_R & 0x80){
	    //wait...
	}
	UART1_DR_R = data;
}

char uart_receive(void){
	char data = 0;
	while (!(UART1_FR_R & 0x80)){
	    //wait...
	}
	data = (char)(UART1_DR_R & 0xFF);
	return data;
}

void uart_sendStr(const char *data){
	// for reference see lcd_puts from lcd.c file
}

void init_char_interrupt(){
    //turn of uart1
    UART1_CTL_R &= 0xFFFE;

    //clear itr flags
    UART1_ICR_R |= 0b1011111110010;

    //enable send recieve intr
    UART1_IM_R |= 0b110000;

    //set priority
    NVIC_PRI1_R |= 0x00200000;

    //enable NVIC for uart1 (page 104)
    NVIC_EN0_R |= 0x00000058;

    IntRegister(INT_UART1, UART1_Handler);

    IntMasterEnable();

    //enable Uart1
    UART1_CTL_R |= 0b1;
}

void UART1_Handler(){
    //received byte
    if (UART1_MIS_R & 0b10000){
        uart_data = uart_receive();
        uart_flag = 1;
        UART1_ICR_R |= 0b10000;
    }
    //transmit byte
    if(UART1_MIS_R & 0b100000){
        //idfk
        UART1_ICR_R |= 0b100000;
    }
}
