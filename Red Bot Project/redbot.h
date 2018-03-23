/*
 *	redbot.h
 *	
 *	Created: 3/14/2018 3:15:46 PM
 *	Author: Matthew Cox
 *	Description: Function Definitions for Motor Control
 *
 */

#ifndef redbot_h
#define redbot_h

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define MOTOR_LEFT_CONTROL1		PIND2
#define MOTOR_LEFT_CONTROL2		PIND4
#define MOTOR_LEFT_PWM			PIND5
#define MOTOR_RIGHT_CONTROL1	PIND7
#define MOTOR_RIGHT_CONTROL2	PINB0
#define MOTOR_RIGHT_PWM			PIND6
#define WHEELSPEED_RIGHT		PINC2
#define WHEELSPEED_LEFT			PINC3

#define LINE_CENTER_IN			6
#define LINE_LEFT_IN			4
#define LINE_RIGHT_IN			1

#endif



/*

// Function will read IR sensor depending on channel argument
uint16_t READ_LINE_SENSOR(uint8_t line_channel);

// Function will read all three line sensors and choose a background level
uint16_t SET_IR_BACKGROUND_LEVEL(void);

// Function will read the center line sensor and set a detection level for the tape
uint16_t SET_IR_DETECT_LEVEL(void);

// Function will initialize the PWM motor timers after they have been disabled
void pwm_timer_init(void);

// Function will disable the PWM motor timers by disconnecting the output pins
void pwm_timer_stop(void);

// Below are low level motor control function definitions
// These define the basic movements of the motors
// We will use these basic functions to build more complex movements

//Set the left motor to move forward. Speed is set separately
void LEFT_MOTOR_FWD(void);

//Set the right motor to move forward. Speed is set separately
void RIGHT_MOTOR_FWD(void);

//Set the left motor to move in reverse. Speed is set separately
void LEFT_MOTOR_REV(void);

//Set the right motor to move in reverse. Speed is set separately
void RIGHT_MOTOR_REV(void);

//Set the left motor to brake (not a stop)
void LEFT_MOTOR_BRAKE(void);

//Set the right motor to brake (not a stop)
void RIGHT_MOTOR_BRAKE(void);

//Set the left motor to stop
void LEFT_MOTOR_STOP(void);

//Set the right motor to stop
void  RIGHT_MOTOR_STOP(void);

*/
