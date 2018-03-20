/*
 * Red Bot Project.c
 *
 * Created: 3/9/2018 1:33:17 PM
 * Author : Matthew Cox / Pawel Bezubik
 */ 

#include "includes.h"
#include "redbot.h"

ISR(TIMER1_COMPA_vect){
	fifty_ms_delay_count++;
}


int main(void)
{
    initialize_all();
    while (1) 
    {
    }
}

