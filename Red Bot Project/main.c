/*
 * Red Bot Project.c
 *
 * Created: 3/9/2018 1:33:17 PM
 * Author : Matthew Cox / Pawel Bezubik
 */ 

#include "includes.h"
#include "redbot.h"

ISR(TIMER1_COMPA_vect){
	program_counter_one++;
	program_counter_two++;
}


int main(void)
{
    initialize_all();
    while (1) 
    {
		// Read Line Sensors Every 50 ms
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
				duty_cycle -= SPEED_RAMP
			} else if (left_line > LINETHRESHOLD){
				LEFT_MOTOR_FWD();
				SET_PWM_OUTPUT((duty_cycle + SPEED_RAMP), MOTOR_LEFT_PWM);
				RIGHT_MOTOR_FWD();
				SET_PWM_OUTPUT((duty_cycle - AVOIDANCE_DEC), MOTOR_RIGHT_PWM);
				LAST_STATE = LINE_LEFT_IN;			
			} else if(left_line < LINETHRESHOLD && right_line < LINETHRESHOLD && center_line << LINETHRESHOLD){
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
    }
}

