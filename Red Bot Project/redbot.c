#include "redbot.h"
#include <avr/io.h>

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
	TCCR0A &= ~(1 << COM0A1) | ~(1 << COM0A0);		// Disables PWM output for timer and enables normal port operation
	PORTD |= (1 << PIND6);							// PWM Out required to be High for Motor stop
}

void LEFT_MOTOR_STOP(void){
	PORTD &= ~(1 << MOTOR_LEFT_CONTROL1);			// Set IN1 to Low according to motor driver datasheet
	PORTD &= ~(1 << MOTOR_LEFT_CONTROL2);			// Set IN2 to Low according to motor driver datasheet
	TCCR0A &= ~(1 << COM0B1) | ~(1 << COM0B0);		// Disables PWM output for timer and enables normal port operation
	PORTD |= (1 << PIND6);							// PWM Out required to be High for Motor stops
}

void RIGHT_MOTOR_BRAKE(void){
	PORTD |= (1 << MOTOR_RIGHT_CONTROL1);			// Set IN1 to High according to motor driver datasheet
	PORTB |= (1 << MOTOR_RIGHT_CONTROL2);			// Set IN2 to High according to motor driver data sheet
	TCCR0A &= ~(1 << COM0A1) | ~(1 << COM0A0);		// Disables PWM output for timer and enables normal port operation
	PORTD |= (1 << PIND6);							// Set PWM output pin to be high. PWM out is a don't care state for short brake
}

void LEFT_MOTOR_BRAKE(void){
	PORTD |= (1 << MOTOR_LEFT_CONTROL1);			// Set IN1 to High according to motor driver datasheet
	PORTD |= (1 << MOTOR_LEFT_CONTROL2);			// Set IN2 to High according to motor driver datasheet
	TCCR0A &= ~(1 << COM0B1) | ~(1 << COM0B0);		// Disables PWM output for timer and enables normal port operation
	PORTD |= (1 << PIND5);							// Set PWM output pin to be high. PWM out is a don't care state for short brake
}


