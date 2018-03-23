/*
 * Red Bot Project.c
 *
 * Created: 3/9/2018 1:33:17 PM
 * Author : Matthew Cox / Pawel Bezubik
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"
//#include "redbot.h"

volatile uint8_t line_channel;
volatile uint8_t program_counter_one = 0;
volatile uint8_t program_counter_two = 0;

volatile uint8_t new_left_duty_cycle;
volatile uint8_t new_right_duty_cycle;

volatile uint16_t left_line;
volatile uint16_t center_line;
volatile uint16_t right_line;
volatile uint8_t duty_cycle = 150;

#define SPEED_RAMP		25
#define AVOIDANCE_DEC	50
#define LINETHRESHOLD	700							// Not Set Will Change

#define MOTOR_LEFT_CONTROL1		PIND2
#define MOTOR_LEFT_CONTROL2		PIND4
#define MOTOR_LEFT_PWM			PIND5
#define MOTOR_RIGHT_CONTROL1	PIND7
#define MOTOR_RIGHT_CONTROL2	PINB0
#define MOTOR_RIGHT_PWM			PIND6
#define WHEELSPEED_RIGHT		PINC2
#define WHEELSPEED_LEFT			PINC3

#define LINE_CENTER_IN			6
#define LINE_LEFT_IN			0
#define LINE_RIGHT_IN			1

const uint16_t program_timer_period = 6249;			// Timer Period for 50ms timer
const uint8_t MOTOR_TIME_PERIOD = 255;				// This is the max value for Timer 0
uint8_t LAST_STATE;

// UART stream for testing
FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
char UART_BUFFER[50];

volatile uint16_t Ain;

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

ISR(TIMER1_COMPA_vect){
	program_counter_one++;
	program_counter_two++;
}

ISR(TIMER0_COMPA_vect){
	OCR0A = new_right_duty_cycle;
}

ISR(TIMER0_COMPB_vect){
	OCR0B = new_left_duty_cycle;
}

void start_ADC_and_wait(void){
	ADCSRA |= (1 << ADSC);
	while((ADCSRA & (1 << ADSC)));
}

uint16_t READ_LINE_SENSOR(uint8_t line_channel){
	uint16_t LINE_SENSOR_OUT_RAW;
	
	// ADMUX Selection for IR Line Sensor
	switch(line_channel)
	{
		case LINE_LEFT_IN:
		ADMUX = LINE_LEFT_IN;
		break;
		case LINE_RIGHT_IN:
		ADMUX |= LINE_RIGHT_IN;
		break;
		case LINE_CENTER_IN:
		ADMUX |= LINE_CENTER_IN;
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

void pwm_timer_init(void){
	
	// Timer 0 PWM for Left & Right Motor
	TCCR0A |= (1 << COM0A1);								// Clear OC0A on Compare Match, set OC0A at BOTTOM
	TCCR0A |= (1 << COM0B1);								// Clear OC0B on Compare Match, ser OC0B at BOTTOM
	TCCR0A |= (1 << WGM01) | (1 << WGM00);					// Fast PWM TOP is default
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

void SET_PWM_OUTPUT(uint8_t pwm_duty_cycle, uint8_t channel){
	switch (channel)
	{
		case MOTOR_LEFT_PWM:
		DDRD |= (1 << DDD5);			// Set PIND5 as Output
		new_left_duty_cycle = pwm_duty_cycle;			// Set Compare register for desired duty cycle
		break;
		case MOTOR_RIGHT_PWM:
		DDRD |= (1 << DDD6);			// Set PIND6 as Output
		new_right_duty_cycle = pwm_duty_cycle;			// Set Compare registers for desired duty cycle
		break;
		default:
		break;
	}
}

int main(void)
{
    initialize_all();
    while(1){
		
		ADMUX = LINE_LEFT_IN;
		ADMUX |= (1 << REFS0);
		start_ADC_and_wait();
		Ain = (ADCL);
		Ain |= (ADCH << 8);
		
		fprintf(stdout, "Left: %d\n", Ain);
		
		ADMUX = LINE_CENTER_IN;
		ADMUX |= (1 << REFS0);
		start_ADC_and_wait();
		Ain = (ADCL);
		Ain |= (ADCH << 8);
		
		fprintf(stdout, "Center: %d\n", Ain);
		
		_delay_ms(100);
		
    }
	return(0);
}



/*
if(program_counter_one >= 1){
	left_line = READ_LINE_SENSOR(LINE_CENTER_IN);
	center_line = READ_LINE_SENSOR(LINE_CENTER_IN);
	right_line = READ_LINE_SENSOR(LINE_RIGHT_IN);
	
	if(center_line > LINETHRESHOLD){
		RIGHT_MOTOR_FWD();
		LEFT_MOTOR_FWD();
		SET_PWM_OUTPUT(duty_cycle, MOTOR_LEFT_PWM);
		SET_PWM_OUTPUT(duty_cycle, MOTOR_RIGHT_PWM);
		LAST_STATE = LINE_CENTER_IN;
		duty_cycle += SPEED_RAMP;
		} else if (right_line > LINETHRESHOLD){
		RIGHT_MOTOR_FWD();
		SET_PWM_OUTPUT((duty_cycle + SPEED_RAMP), MOTOR_RIGHT_PWM);
		LEFT_MOTOR_FWD();
		SET_PWM_OUTPUT((duty_cycle - AVOIDANCE_DEC), MOTOR_LEFT_PWM);
		LAST_STATE = LINE_RIGHT_IN;
		duty_cycle -= SPEED_RAMP;
		} else if (left_line > LINETHRESHOLD){
		LEFT_MOTOR_FWD();
		SET_PWM_OUTPUT((duty_cycle + SPEED_RAMP), MOTOR_LEFT_PWM);
		RIGHT_MOTOR_FWD();
		SET_PWM_OUTPUT((duty_cycle - AVOIDANCE_DEC), MOTOR_RIGHT_PWM);
		LAST_STATE = LINE_LEFT_IN;
		} else if(left_line < LINETHRESHOLD && right_line < LINETHRESHOLD && center_line < LINETHRESHOLD){
		switch (LAST_STATE)
		{
			case LINE_LEFT_IN:
			RIGHT_MOTOR_FWD();
			SET_PWM_OUTPUT((duty_cycle + SPEED_RAMP), MOTOR_RIGHT_PWM);
			LEFT_MOTOR_FWD();
			SET_PWM_OUTPUT((duty_cycle - AVOIDANCE_DEC), MOTOR_LEFT_PWM);
			break;
			case LINE_RIGHT_IN:
			LEFT_MOTOR_FWD();
			SET_PWM_OUTPUT((duty_cycle + SPEED_RAMP), MOTOR_LEFT_PWM);
			RIGHT_MOTOR_FWD();
			SET_PWM_OUTPUT((duty_cycle - AVOIDANCE_DEC), MOTOR_RIGHT_PWM);
			break;
			case LINE_CENTER_IN:
			LEFT_MOTOR_BRAKE();
			SET_PWM_OUTPUT(0, MOTOR_LEFT_PWM);
			RIGHT_MOTOR_BRAKE();
			SET_PWM_OUTPUT(0, MOTOR_RIGHT_PWM);
			LEFT_MOTOR_REV();
			SET_PWM_OUTPUT(duty_cycle, MOTOR_LEFT_PWM);
			RIGHT_MOTOR_REV();
			SET_PWM_OUTPUT(duty_cycle, MOTOR_RIGHT_PWM);
		}
	}
*/