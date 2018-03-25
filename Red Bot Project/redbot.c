/*
 *	redbot.h
 *	
 *	Created: 3/14/2018 3:15:46 PM
 *	Author: Matthew Cox
 *	Description: Functions for Motor Control
 *
 */

#include "redbot.h"

void pwm_timer_init(void){
	
	// Timer 0 PWM for Left & Right Motor
	TCCR0A |= (1 << COM0A1);								// Clear OC0A on Compare Match, set OC0A at BOTTOM
	TCCR0A |= (1 << COM0B1);								// Clear OC0B on Compare Match, set OC0B at BOTTOM
	TCCR0A |= (1 << WGM01) | (1 << WGM00);					// Fast PWM TOP is default
	TCCR0B |= (1 << CS01);									// Prescaler of 8 for 7.812kHz PWM signal
	TIMSK0 |= (1 << OCIE0A) | (1 << OCIE0B);				// Set Compare Match Interrupts for OC0A and OC0B
}

void pwm_timer_stop(void){
	TCCR0A &= ~(1 << COM0A1) | ~(1 << COM0A0);		// Disables PWM output for timer and enables normal port operation
}

void RIGHT_MOTOR_FWD(void){
	PORTD |= (1 << MOTOR_RIGHT_CONTROL1);			// Set IN1 to High according to motor driver datasheet
	PORTB &= ~(1 << MOTOR_RIGHT_CONTROL2);			// Set IN2 to Low according to motor driver datasheet
}

void LEFT_MOTOR_FWD(void){
	PORTD |= (1 << MOTOR_LEFT_CONTROL1);			// Set IN1 to High according to motor driver datasheet
	PORTD &= ~(1 << MOTOR_LEFT_CONTROL2);			// Set IN2 To Low according to motor driver datasheet
}

void RIGHT_MOTOR_REV(void){
	PORTD &= ~(1 << MOTOR_RIGHT_CONTROL1);			// Set IN1 to Low according to motor driver datasheet
	PORTB |= (1 << MOTOR_RIGHT_CONTROL2);			// Set IN2 to High according to motor driver datasheet
}

void LEFT_MOTOR_REV(void){
	PORTD &= ~(1 << MOTOR_LEFT_CONTROL1);			// Set IN1 to Low according to motor driver datasheet
	PORTB |= (1 << MOTOR_LEFT_CONTROL2);			// Set IN2 to High according to motor driver datasheet
}

void RIGHT_MOTOR_STOP(void){
	PORTD &= ~(1 << MOTOR_RIGHT_CONTROL1);			// Set IN1 to Low according to motor driver datasheet
	PORTB &= ~(1 << MOTOR_RIGHT_CONTROL2);			// Set IN2 to Low according to motor driver datasheet
	pwm_timer_stop();
	PORTD |= (1 << PIND6);							// PWM Out required to be High for Motor stop
}

void LEFT_MOTOR_STOP(void){
	PORTD &= ~(1 << MOTOR_LEFT_CONTROL1);			// Set IN1 to Low according to motor driver datasheet
	PORTD &= ~(1 << MOTOR_LEFT_CONTROL2);			// Set IN2 to Low according to motor driver datasheet
	pwm_timer_stop();
	PORTD |= (1 << PIND6);							// PWM Out required to be High for Motor stops
}

void RIGHT_MOTOR_BRAKE(void){
	PORTD |= (1 << MOTOR_RIGHT_CONTROL1);			// Set IN1 to High according to motor driver datasheet
	PORTB |= (1 << MOTOR_RIGHT_CONTROL2);			// Set IN2 to High according to motor driver data sheet
	pwm_timer_stop();
	PORTD |= (1 << PIND6);							// Set PWM output pin to be high. PWM out is a don't care state for short brake
}

void LEFT_MOTOR_BRAKE(void){
	PORTD |= (1 << MOTOR_LEFT_CONTROL1);			// Set IN1 to High according to motor driver datasheet
	PORTD |= (1 << MOTOR_LEFT_CONTROL2);			// Set IN2 to High according to motor driver datasheet
	pwm_timer_stop();
	PORTD |= (1 << PIND5);							// Set PWM output pin to be high. PWM out is a don't care state for short brake
}

