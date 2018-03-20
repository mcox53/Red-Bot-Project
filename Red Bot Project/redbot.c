/*
 *	redbot.h
 *	
 *	Created: 3/14/2018 3:15:46 PM
 *	Author: Matthew Cox
 *	Description: Functions for Motor Control
 *
 */

#include "redbot.h"
#include "includes.h"

uint16_t READ_LINE_SENSOR(uint8_t line_channel){
	uint16_t LINE_SENSOR_OUT_RAW;
	
	// ADMUX Selection for IR Line Sensor
	switch(line_channel)
	{
		case LINE_LEFT_IN:
			ADMUX &= LINE_LEFT_MUX
			break;
		case LINE_RIGHT_IN:
			ADMUX |= LINE_LEFT_MUX
			break;
		case LINE_CENTER_IN:
			ADMUX |= LINE_CENTER_MUX;
		default:
			break;
	}
	// Start ADC Conversion and wait until complete
	ADCSRA |= (1 << ADSC);
	while((ADCSRA & (1 << ADSC)));
	
	// Will Decide whether to output something different based on sensor data
	LINE_SENSOR_OUT_RAW = (ADCH << 8) | ADCL;
	return(LINE_SENSOR_OUT_RAW);
}

uint16_t SET_IR_DETECT_LEVEL(void){
	uint16_t CENTER_IR_LEVEL = READ_LINE_SENSOR(LINE_CENTER_IN);
	return(CENTER_IR_LEVEL);
}

uint16_t SET_IR_BACKGROUND_LEVEL(void){
	uint16_t LEFT_IR_LEVEL = READ_LINE_SENSOR(LINE_LEFT_IN);
	uint16_t RIGHT_IR_LEVEL = READ_LINE_SENSOR(LINE_RIGHT_IN);
	
	uint16_t average = (LEFT_IR_LEVEL + RIGHT_IR_LEVEL) / 2;
	return(average);
}

void SET_PWM_OUTPUT(uint8_t pwm_duty_cycle, uint8_t channel){
	
	switch (channel)
	{
		case MOTOR_LEFT_PWM:
			DDRD |= (1 << DDD5);			// Set PIND5 as Output
			OCR0B = pwm_duty_cycle;			// Set Compare register for desired duty cycle
			break;
		case MOTOR_RIGHT_PWM:
			DDRD |= (1 << DDD6);			// Set PIND6 as Output
			OCR0A = pwm_duty_cycle;			// Set Compare registers for desired duty cycle
			break;
		default:
			break;	
	}
	
}

void pwm_timer_init(void){
	
	// Timer 0 PWM for Left & Right Motor
	TCCR0A |= (1 << COM0A1);								// Clear OC0A on Compare Match, set OC0A at BOTTOM
	TCCR0A |= (1 << COM0B1);								// Clear OC0B on Compare Match, ser OC0B at BOTTOM
	TCCR0A |= (1 << WGM01) | (1 << WGM00);					// Fast PWM OCRA is TOP
	TCCR0B |= (1 << CS02);									// CHANGE THIS - Clock Prescaler
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


