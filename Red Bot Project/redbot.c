#include "redbot.h"
#include <avr/io.h>

uint16_t READ_LINE_SENSOR(int line_channel){
	uint16_t LINE_SENSOR_OUT_RAW;
	
	// ADMUX Selection for IR Line Sensor
	switch(line_channel)
	{
		case LINE_LEFT_IN:
			ADMUX &= ~(1 << MUX3) | ~(1 << MUX2) | ~(1 << MUX1) | ~(1 << MUX0);
			break;
		case LINE_RIGHT_IN:
			ADMUX |= (1 << MUX0);
			break;
		case LINE_CENTER_IN:
			ADMUX |= (1 << MUX2) | (1 << MUX1);
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