/**
 * Contains methods for communication (high level) between cybot and user
 * @author Tyler Bibus
 */

#include "uart.h"

void send_message(char message[]){
    //sends each character of the message array
    while (*message != 0){
        uart_sendChar(*message);
        message++;
    }
    uart_sendChar('\n');
    //uart_sendChar('\r');
    return;
}

void printGraphicalView(float distances[], char message[]){
    char j;
    j = 0;
    //alternative implementation could be printing each character at a time in two for loops...
    //alternative implementation could be good if using too much memory
        while (j <= 180){
            if (distances[(180 - j)/2] > 60.0){
                message[j] = '*';
                message[j+1] = '*';
            } else if (distances[(180 - j)/2] > 50.0) {
                message[j] = '-';
                message[j+1] = '-';
            } else if (distances[(180 - j)/2] > 35.0) {
                message[j] = '=';
                message[j+1] = '=';
            } else if (distances[(180 - j)/2] > 20.0) {
                message[j] = '@';
                message[j+1] = '@';
            }
            else if (distances[(180 - j)/2] > 4.0){
                message[j] = '#';
                message[j+1] = '#';
            }
            else {
                message[j] = '*';
                message[j+1] = '*';
            }
            j += 2;
        }
        message[j + 1] = 0;
        j = 0;
        for (j = 0; j < 4; j++){
            uart_sendChar('\r');
            uart_sendChar('\n');
            send_message(message);
        }
        uart_sendChar('\r');
        uart_sendChar('\n');
}
