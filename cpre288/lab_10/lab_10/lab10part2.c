/**
*@author Tyer Bibus
* my proudest code yet
*/
/*
#include "Timer.h"
#include "servo.h"
#include "lcd.h"
#include "button.h" //buttons will be interrupt driven
#include <math.h>

extern volatile int button_event;
extern volatile int button_num;

int max(int x, int y){
	return (x > y) ? x : y; //im so proud of myself this is beautiful
}

//Cybot #9 match for  0 deg = 8000, 45 deg = 15022, 90 = 22044, 135 deg = 29066  180 deg = 36088
void main(){
	timer_init();
    lcd_init();
    servo_init();
	button_init();
	int angle = 90;
	int matchVal = 0;
	int directionMod = 1; //1 = counterClock, -1 = clockwise
	servo_move(90); //initially set to 90 deg
	while(1){
		//wait for button presses
		while(!button_event && button_event == 0){}
		button_event = 0;
		switch(button_num){
			case 1:
				angle = angle + (1 * directionMod); //move 1 degree
				break;
			case 2:
				angle = angle + (5 * directionMod); //move 5 degrees
				break;
			case 3:
				directionMod = directionMod * -1; //invert direction
				break;
			case 4:
				angle = 0 + max(0, 180 * directionMod); //move to 0 or 180
				break;
			default:
				break;
		}
		matchVal = servo_move(angle);
		lcd_printf("Match: %d\nDir: %d", matchVal, directionMod);
	}
}
*/
