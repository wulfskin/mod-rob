/*
 * Navigationtest.c
 *
 * Created: 05/06/2012 13:29:59
 *  Author: Walter Gambelunghe
 */ 
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "macro.h"
#include "sensor.h"
#include "motor.h"
#include "dynamixel.h"
#include "serial.h"
#include "error.h"
#include "serialzigbee.h"
#include "io.h"
#include "timer.h"

#define SENSOR_FRONT 1
#define SENSOR_FRONTLEFT 3
#define SENSOR_FRONTRIGHT 4
#define SENSOR_BACKLEFT 2
#define SENSOR_BACKRIGHT 5
#define MOTOR_LEFT 11
#define MOTOR_RIGHT 12

#define MAX_SPEED_IN_PERCENTAGE 40ul
#define MINIMUM_DISTANCE	50
#define REDUCED_TURN_SPEED	1023/2	

#define STATE_AVOID_OBSTACLES			1
#define STATE_AVOID_OBSTACLES_DIFF		2

void timer0_compA()
{
	; // Do something every ms	
};

int timer0_initialize()
{	
	// Return value
	int res = 0;	
	// Timer number
	uint8_t timer = 0;
	// Compare match A value
	uint8_t compare_value = 250;
	// Set compare match A value
	if (timer_set_value(timer, TVT_OUTPUT_COMPARE_A, compare_value) == TIMER_ERROR_SUCCESS)
	{
		// Set interrupt on compare match A
		if (timer_set_interrupt(timer, TIT_OUTPUT_COMPARE_MATCH_A, &timer0_compA) == TIMER_ERROR_SUCCESS)
		{
			// Initialize and start timer
			if (timer_init(timer, TOM_CLEAR_ON_COMPARE, TPS_DIV_64, 0) == TIMER_ERROR_SUCCESS)
			{
				res = 1;	
			}
		}
	}				
	return res;
}

int main() {
	timer0_initialize();
	// Initialize stuff		
	dxl_initialize(0,1);
	serial_initialize(57600);
	sensor_init(SENSOR_FRONT,DISTANCE);
	sensor_init(SENSOR_FRONTLEFT,IR);
	sensor_init(SENSOR_FRONTRIGHT,IR);
	sensor_init(SENSOR_BACKLEFT,IR);
	sensor_init(SENSOR_BACKRIGHT,IR);
	io_init();
	
	// Active general interrupts
	sei();
	
	// Set modes
	motor_set_mode(254, MOTOR_WHEEL_MODE);
	serial_set_zigbee();
	
	//dxl_write_word(254, 14, 1023);
			
	uint8_t ir_frontleft, ir_frontright, ir_backleft, ir_backright, dis_front;
	int speed_left, speed_right;
	int speed_diff = 0;
	uint8_t state = STATE_AVOID_OBSTACLES_DIFF;
	
	uint16_t speed_l, speed_r;

	printf("PROGRAM IS RUNNING!\n");

	while(1) {
		dis_front = sensor_read(SENSOR_FRONT, DISTANCE);
		ir_frontleft = (sensor_read(SENSOR_FRONTLEFT, IR));
		ir_frontright = (sensor_read(SENSOR_FRONTRIGHT, IR));
		ir_backleft = sensor_read(SENSOR_BACKLEFT, IR);
		ir_backright = sensor_read(SENSOR_BACKRIGHT, IR);

		// Remove noise from sensors
		if (ir_frontleft < 15)
			ir_frontleft = 0;
		if (ir_frontright < 15)
			ir_frontright = 0;
		if (ir_backleft < 15)
			ir_backleft = 0;
		if (ir_backright < 15)
			ir_backright = 0;

		// State machine
		switch (state)
		{
			case STATE_AVOID_OBSTACLES:
			{
				// Calculate speed
				speed_left = ((255 - ir_frontright) << 2);
				speed_right = ((255 - ir_frontleft) << 2);
				
				// Set direction of wheels in case of a turn
				if (ir_frontright > MINIMUM_DISTANCE)
				{
					speed_left = -REDUCED_TURN_SPEED;
				}					
				
				if (ir_frontleft > MINIMUM_DISTANCE)
				{
					speed_right = -REDUCED_TURN_SPEED;
				}						
				break;
			}				
			
			case STATE_AVOID_OBSTACLES_DIFF:
			{
				// Behaviour: Avoid obstacles
				speed_diff = (int) (ir_frontleft - ir_frontright) / 4;
				uint8_t free_dist = 255 - dis_front;
				
				// Behaviour: Follow-the-wall
				if (ir_backleft > 25 && ir_backright == 0)
					speed_diff -= (75 - ir_backleft) / 4;
					
				if (ir_backright > 25 && ir_backleft == 0)
					speed_diff += (75 - ir_backright) / 4;
				
				free_dist = 124;
				speed_left = free_dist + speed_diff;
				speed_right = free_dist - speed_diff;
				
				if (speed_left > 255)
					speed_left = 255;
					
				if (speed_left < -255)
					speed_left = -255;
					
				if (speed_right > 255)
					speed_right = 255;
					
				if (speed_right < -255)
					speed_right = -255;
					
				speed_left = speed_left << 2;
				speed_right = speed_right << 2;
				break;
			}
			
			default:
				; //nothing
			break;
		}	
		
		// Set direction bit
		char direction_left, direction_right;
		if (speed_left < 0)
		{
			direction_left = MOTOR_CW;
			speed_left = -speed_left;
		}
		else
			direction_left = MOTOR_CCW;
			
		if (speed_right < 0)
		{
			direction_right = MOTOR_CCW;
			speed_right = -speed_right;
		}
		else
			direction_right = MOTOR_CW;
		
		speed_l = (uint16_t) speed_left * MAX_SPEED_IN_PERCENTAGE / 100ul;
		speed_r = (uint16_t) speed_right * MAX_SPEED_IN_PERCENTAGE / 100ul;
		
		//printf("Speed: L: %d, R: %d.\n", speed_l, speed_r);
		
		// Set motor speed
		motor_set_speed_dir(MOTOR_LEFT, speed_l, direction_left);
		motor_set_speed_dir(MOTOR_RIGHT, speed_r, direction_right);	
		
/*
            if (!state) {
				LED_OFF(LED_PLAY);
				if (ir_frontleft < 15)
						ir_frontleft = 0;
				if (ir_frontright < 15)
					ir_frontright = 0;
				if(ir_backleft >100 || ir_backleft>100)
					state=1;
					
				speed_left= ir_frontright < 50 ? (1023-(16*ir_frontright)) : (1023+(2*ir_frontright));
				speed_right=ir_frontleft  < 50 ? 2047-(16*ir_frontleft) : (2*ir_frontleft);
			}
			else if(state){
				LED_ON(LED_PLAY);
				speed_left=0;
				speed_right=0;
				 
			}			
			
	*/
/*
		if (ir_backright > ir_frontright)
			ir_frontright = 0;
		else
			ir_frontright -= ir_backright;
				
		if (ir_backright > ir_frontright)
			ir_frontright = 0;
		else
			ir_frontright -= ir_backleft;
			
		if (ir_frontleft < 15)
			ir_frontleft = 0;
		if (ir_frontright < 15)
			ir_frontright = 0;
		if(ir_frontright<50){
			speed_left=100-(ir_frontright)
			dir_left=MOTOR_CCW;
		}
		else{
			speed_left=ir_frontright/8;
			dir_left=
		}
		speed_left= ir_frontright < 50 ? (1023-(16*ir_frontright)) : (1023+(2*ir_frontright));
		speed_right=ir_frontleft  < 50 ? 2047-(16*ir_frontleft) : (2*ir_frontleft);
					
		speed_left=*/
		//speed_left=ir_backleft>speed_left? (speed_left-ir_backleft): 0;
		//speed_rightt=ir_backright>speed_right? (speed_left-ir_backleft): 0;
		//speed_right-=ir_backright;
			
			/*
			
		int target_left = (ir_frontleft < 50) ? (1023ul - ir_frontleft) : (-ir_frontleft);
		if (ir_backleft > ir_frontleft)
			target_left = 0;
		else
			target_left -= ir_backleft;
				
		int target_right = ir_frontright;
		if (ir_backright > ir_frontright)
			target_right = 0;
		else
			target_right -= ir_backright;
			
		uint8_t speed_right = (uint8_t) (target_left * 100ul / 1023ul);
		uint8_t speed_left = (uint8_t) (target_right * 100ul / 1023ul);
			
			
		motor_set_speed_dir(MOTOR_LEFT,speed_left,MOTOR_CCW);
		motor_set_speed_dir(MOTOR_RIGHT,speed_right,MOTOR_CW);
		
		*/
		//}			
		//printf("distance= %3d, left= %3d, right=%3d, speed_left=%3d, speed_right=%3d \n",touch,ir_left,ir_right,speed_left,speed_right);
		printf("f= %3u, fl= %3u, fr=%3u, bl= %3u, br=%3u, speed=%3d,\n",dis_front,ir_frontleft,ir_frontright,ir_backleft, ir_backright, speed_diff);
	}
	return 0;
}

		/*if(touch){
			motor_set_speed(MOTOR_LEFT,0);
			motor_set_speed(MOTOR_RIGHT,0);
		}
		else {*/

//motor_set_speed(MOTOR_LEFT,speed_left<0 ? (2048-speed_left): (speed_left) );
//speed_left=1023;
//speed_right=2047;
//int diff= (ir_left>ir_right) ? (ir_left-ir_right) : (ir_right-ir_left);
//if (ir_left<ir_right)
//{
	//	speed_right-=2*(ir_right-ir_left);
//}
//else
//	{
	//	speed_left-=2*(ir_left-ir_right);
//	}
	