/*
 * Red Bot Project.c
 *
 * Created: 3/20/2018 2:31:28 PM
 * Author : Matthew Cox
 */ 

#include "includes.h"
#include "redbot.h"

void initialize_all(void){
	sei();
	// Configure Port D and B as outputs
	DDRD |= (1 << MOTOR_LEFT_PWM) | (1 << MOTOR_LEFT_CONTROL1) | (1 << MOTOR_LEFT_CONTROL2);
	DDRB |= (1 << MOTOR_RIGHT_PWM) | (1 << MOTOR_RIGHT_CONTROL1);
	DDRB |= (1 << MOTOR_RIGHT_CONTROL2);
	
	uart_init();
	stdin = stdout = stderr = &uart_stream;
	
	// ADC Initializations
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// 128 prescaler
	ADCSRA |= (1 << ADEN);									// Enable ADC and									// Enable ADC
	ADMUX |= (1 << REFS0);									// Change Reference voltage to AVcc with external cap
	
	// Program timer that counts every 50ms
	OCR1A = program_timer_period;
	TCCR1B |= (1 << CS11) | (1 << CS10);					// 64 prescaler
	TCCR1B |= (1 << WGM12);									// CTC Mode
	TIMSK1 |= (1 << OCIE1A);								// Timer Match A Interrupt
}