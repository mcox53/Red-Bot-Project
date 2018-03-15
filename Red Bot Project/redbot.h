/*
 *	redbot.h
 *	
 *	Created: 3/14/2018 3:15:46 PM
 *	Author: Matthew Cox
 *	Description: Function Definitions for Motor Control
 *
 */

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//Line Center IN			ADC6
//Line Left IN				ADC0
//Line Right IN				ADC1
//Accel SDA					PINC4
//Accel SCL					PINC5

#define MOTOR_LEFT_CONTROL1		PIND2
#define MOTOR_LEFT_CONTROL2		PIND4
#define MOTOR_RIGHT_CONTROL1	PIND7
#define MOTOR_RIGHT_CONTROL2	PINB0
#define WHEELSPEED_RIGHT
#define WHEELSPEED_LEFT

