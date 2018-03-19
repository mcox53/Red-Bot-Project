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

volatile uint8_t fifty_ms_delay_count;
uint16_t timer_period = 6249;						// Timer Period for 50ms timer

// UART stream for testing
FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
char UART_BUFFER[50];

void initialize_all(void){
	// Configure Port D and B as outputs
	DDRD = 0xFF;
	DDRB = 0xFF;
	
	uart_init();
	stdin = stdout = stderr = &uart_stream;
	
	// ADC INITS FOR PRESCALER/ENABLE ONLY
	// CONVERSION HANDLED ELSEWHERE
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// 128 prescaler
	ADCSRA |= (1 << ADEN);									// Enable ADC
	
	// Program timer counter that counts every 50ms
	OCR1A = timer_period;
	TCCR1B |= (1 << CS11) | (1 << CS10);					// 64 prescaler
	TCCR1B |= (1 << WGM12);									// CTC Mode
	TIMSK1 |= (1 << OCIE1A);								// Timer Match A Interrupt
}

ISR(TIMER1_COMPA_vect){
	fifty_ms_delay_count++;
}


int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
    }
}

