/*
 * Red Bot Project.c
 *
 * Created: 3/20/2018 2:31:28 PM
 * Author : Matthew Cox
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
volatile uint8_t fifty_ms_delay_count = 0;

const uint16_t program_timer_period = 6249;			// Timer Period for 50ms timer
const uint8_t MOTOR_TIME_PERIOD = 256;				// This is the max value for Timer 0

// UART stream for testing
FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
char UART_BUFFER[50];

// Initialize All
void initialize_all(void);