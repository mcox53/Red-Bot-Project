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
#define WHEELSPEED_RIGHT		PINC2
#define WHEELSPEED_LEFT			PINC3

#define LINE_CENTER_IN			6
#define LINE_LEFT_IN			0
#define LINE_RIGHT_IN			1


// The two functions below are a work in Progress
// function will change timer compare match registers?
void CHANGE_MOTOR_SPEED(void);

// function will read IR sensor?
// Should there be a function for all 3 or just check each one at once?
uint16_t READ_LINE_SENSOR(int line_channel);


// Below are low level motor control function definitions
// These define the basic movements of the motors
// We will use these basic functions to build more complex movements

//Set the left motor to move forward at a specific speed
void LEFT_MOTOR_FWD(int speed);

//Set the right motor to move forward at a specific speed
void RIGHT_MOTOR_FWD(int speed);

//Set the left motor to move in reverse at a specific speed
void LEFT_MOTOR_REV(int speed);

//Set the right motor to move in reverse at a specific speed
void RIGHT_MOTOR_REV(int speed);

//Set the left motor to brake (not a stop)
void LEFT_MOTOR_BRAKE(void);

//Set the right motor to brake (not a stop)
void RIGHT_MOTOR_BRAKE(void);

//Set the left motor to stop
void LEFT_MOTOR_STOP(void);

//Set the right motor to stop
void  RIGHT_MOTOR_STOP(void);



