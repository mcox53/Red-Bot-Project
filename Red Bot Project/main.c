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
#include "redbot.h"

volatile uint8_t line_channel;
volatile uint8_t program_counter_one = 0;
volatile uint8_t program_counter_two = 0;

volatile uint8_t new_left_duty_cycle;
volatile uint8_t new_right_duty_cycle;

volatile uint16_t left_line;
volatile uint16_t center_line;
volatile uint16_t right_line;
volatile uint8_t duty_cycle = 200;
volatile int8_t direction = -1;					// -1 when going left, 1 when going right

const uint16_t program_timer_period = 6249;			// Timer Period for 50ms timer
const uint8_t MOTOR_TIME_PERIOD = 255;				// This is the max value for Timer 0
uint8_t LAST_STATE;

// UART stream for testing
FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
char UART_BUFFER[50];

// PID variables
volatile float error = 0;
volatile float last_error = 0;
volatile float derivative = 0;
volatile float integral = 0;
volatile float setpoint = 700;						// Calibrated value for edge of tape
volatile float PIDOutput = 0;
volatile float Kp = 0.1;
volatile float Ki = 0;
volatile float Kd = 0;
volatile float PID_to_duty_left = 0;
volatile float PID_to_duty_right = 0;

#define LOW_ERROR_THRESHOLD		10					// 0 - 90 value representing low error range
#define HIGH_ERROR_THRESHOLD	25					// 0 - 90 value representing high error range
#define TURN_SPEED				50					// How much the motor will turn
#define SPEED_INCR				.1					// Incrementing the speed when on the right path
#define SPEED_DECR				.25					// Decrementing the speed when off the correct path

volatile float duty_multiplier = TURN_SPEED;

void PIDCalculateOutput(float setpoint){
	error = setpoint - center_line;
	
	integral += error;
	
	derivative = error - last_error;
	
	PIDOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
	
	last_error = error;
	
	if(PIDOutput > 90){
		PIDOutput = 90;
	}else if(PIDOutput < -90){
		PIDOutput = -90;
	}
	
}

void PIDspeedControl(float PIDOut){
	
	if(abs(PIDOut) < LOW_ERROR_THRESHOLD){
		if(duty_multiplier > 0){
			duty_multiplier -= SPEED_INCR;
		}
	} else if(abs(PIDOut) < HIGH_ERROR_THRESHOLD){
		if(duty_multiplier < TURN_SPEED){
			duty_multiplier += SPEED_DECR;
		}
	} else {
		duty_multiplier = TURN_SPEED;
	}
	
	PID_to_duty_left = (int)((direction * -1 * (PIDOut / 90) * duty_multiplier) + (100 - duty_multiplier));
	PID_to_duty_right = (int)((direction * (PIDOut / 90) * duty_multiplier) + (100 - duty_multiplier));
}

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
	ADCSRA |= (1 << ADEN);									// Enable ADC and ADC Conversion Complete Interrupt
	ADMUX |= (1 << REFS0);									// Change Reference voltage to AVcc with external cap
	
	// Program timer that counts every 50ms
	OCR1A = program_timer_period;
	TCCR1B |= (1 << CS11) | (1 << CS10);					// 64 prescaler
	TCCR1B |= (1 << WGM12);									// CTC Mode
	TIMSK1 |= (1 << OCIE1A);								// Timer Match A Interrupt
}

ISR(TIMER1_COMPA_vect){
	program_counter_one++;
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

void READ_LINE_SENSOR(void){
	
	ADMUX = LINE_LEFT_IN;
	ADMUX |= (1 << REFS0);
	
	start_ADC_and_wait();
	left_line = (ADCL);
	left_line |= (ADCH << 8);
	
	ADMUX = LINE_CENTER_IN;
	ADMUX |= (1 << REFS0);
	
	start_ADC_and_wait();
	center_line = (ADCL);
	center_line |= (ADCH << 8);

	ADMUX = LINE_RIGHT_IN;
	ADMUX |= (1 << REFS0);
	
	start_ADC_and_wait();
	right_line = (ADCL);
	right_line |= (ADCH << 8);
}


int main(void)
{
    initialize_all();
	pwm_timer_init();
	RIGHT_MOTOR_FWD();
	LEFT_MOTOR_FWD();
	
    while(1){
		
		if (program_counter_one >= 1){
			READ_LINE_SENSOR();
			if(direction > 0 && left_line > 700){
				direction *= -1;
			}
			
			if(direction < 0 && right_line > 700){
				direction *= -1;
			}
			
			PIDCalculateOutput(setpoint);
			PIDspeedControl(PIDOutput);
			SET_PWM_OUTPUT(PID_to_duty_left, MOTOR_LEFT_PWM);
			SET_PWM_OUTPUT(PID_to_duty_right, MOTOR_RIGHT_PWM);
			
		}
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