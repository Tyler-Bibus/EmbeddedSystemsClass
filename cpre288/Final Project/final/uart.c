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

/* Ring buffer written by the RX ISR and read by the command parser.  The
 * previous single-byte uart_data/uart_flag pair could lose a byte if a new
 * interrupt arrived while the main loop was copying the last one. */
volatile unsigned char uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile unsigned int uart_rx_head = 0;
volatile unsigned int uart_rx_tail = 0;

void uart_init(void){
    uart_rx_head = 0;
    uart_rx_tail = 0;
    SYSCTL_RCGCGPIO_R |= 0b0010;      // enable clock GPIOB (page 340)
    SYSCTL_RCGCUART_R |= 0b0010;      // enable clock UART1 (page 344)
    GPIO_PORTB_AFSEL_R |= 0b0011;      // sets PB0 and PB1 as peripherals (page 671
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
	// Wait while the receive FIFO is empty (RXFE, bit 4), not TXFE.
	while ((UART1_FR_R & 0x10)){
	    //wait...
	}
	data = (char)(UART1_DR_R & 0xFF);
	return data;
}

int uart_available(void){
    return uart_rx_head != uart_rx_tail;
}

char uart_getchar(void){
    char data;
    while (!uart_available()){
        // wait for a complete byte from the RX interrupt
    }
    data = (char)uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUFFER_SIZE;
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

    //enable receive interrupt only; TX is polled in uart_sendChar
    UART1_IM_R |= 0b010000;

    //set priority
    NVIC_PRI1_R |= 0x00200000;

    //enable NVIC for uart1 (IRQ 6 = bit 6)
    NVIC_EN0_R |= 0x00000040;

    IntRegister(INT_UART1, UART1_Handler);

    IntMasterEnable();

    //enable Uart1
    UART1_CTL_R |= 0b1;
}

void UART1_Handler(){
    unsigned int next;
    //received byte
    if (UART1_MIS_R & 0b10000){
        next = (uart_rx_head + 1) % UART_RX_BUFFER_SIZE;
        // If the ring is full, drop the byte rather than corrupting queued data.
        if (next != uart_rx_tail){
            uart_rx_buffer[uart_rx_head] = (unsigned char)uart_receive();
            uart_rx_head = next;
        }
        else{
            (void)uart_receive();
        }
        UART1_ICR_R |= 0b10000;
    }
    // No TX interrupt is enabled; TX is polled in uart_sendChar.
    if (UART1_MIS_R & 0b100000){
        UART1_ICR_R |= 0b100000;
    }
}
