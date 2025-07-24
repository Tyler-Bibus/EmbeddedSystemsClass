/*
* @author Tyler Bibus
*
*/
/*
#include "adc.h"
#include "timer.h"
#include "lcd.h"

void main(){
	//initialize everything
	timer_init();
	lcd_init();
	adc_init();
	
	float IRDistance = 0;
	float IRDistReal = 0;
	
	//take IR sample 3 times, average, wait
	while(1){
		//get 3 IR distances
		IRDistance = 0;
		IRDistance = 1.0 * adc_read();

		//average distances
		//IRDistance = IRDistance / 3.0;
		
		IRDistReal = 86.9 - (IRDistance * 0.0633) + (0.0000125 * (IRDistance * IRDistance));

		lcd_printf("Val: %3.3f\nRaw: %d", IRDistReal, (int)IRDistance);
		
		//wait 5 seconds
		timer_waitMillis(3000);
	}
	
}
*/
