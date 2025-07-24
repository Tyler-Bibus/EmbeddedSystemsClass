/// Simple 'Hello, world' program
/**
 * This program prints "Hello, world" to the LCD screen
 * @author Chad Nelson
 * @date 06/26/2012
 *
 * updated: phjones 9/3/2019
 * Describtion: Added timer_init call, and including Timer.h
 */

#include "Timer.h"
#include "lcd.h"


int main (void) {
    int currentChar = 0; //max 52
    char printString[21] = "                    ";
    char message[] = "H";


	timer_init(); // Initialize Timer, needed before any LCD screen fucntions can be called 
	              // and enables time functions (e.g. timer_waitMillis)

	lcd_init();   // Initialize the the LCD screen.  This also clears the screen. 

	// Clear the LCD screen and print "Hello, world" on the LCD
	//lcd_printf("Hello, world");

	 //lcd_puts("Hello, world");// Replace lcd_printf with lcd_puts
                                 // step through in debug mode
                                 // and explain to TA how it
                                 // works
	int i;
	while(1){
	    //waits
	    timer_waitMillis(300);

	 //shifts print string
	    for (i = 0; i < 19; i++){
	        //TODO maybe broken?
	        printString[i] = printString[i+1];
	    }

	    //Copy last digit


	    if (currentChar < strlen(message)){
	        printString[19] = message[currentChar];
	        currentChar++;
	    }
	    else if (currentChar < strlen(message) + 19){
	        printString[19] = ' ';
	        currentChar++;
	    }
	    else currentChar = 0;

	    //prints to bot
	    lcd_printf(printString);

	}
    
	// NOTE: It is recommended that you use only lcd_init(), lcd_printf(), lcd_putc, and lcd_puts from lcd.h.
    // NOTE: For time fuctions see Timer.h

	return 0;
}
