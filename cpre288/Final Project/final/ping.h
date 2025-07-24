/*
 * ping.h
 *
 *  Created on: Oct 31, 2024
 *      Author: tbibus
 */

#ifndef PING_H_
#define PING_H_

//This configures timer for PING specifically :D
void ping_init(void);

//interrupt handler
void TIMER3B_Handler(void);

//reads the ping
unsigned int ping_read(void);

//sends pulse
void send_pulse(void);


#endif /* PING_H_ */
