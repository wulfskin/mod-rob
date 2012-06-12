/*
 * Navigationtest_old.c
 *
 * Created: 
 *  Author: Dennis Hellner
 */ 
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "../macro.h"
#include "../sensor.h"
#include "../motor.h"
#include <dynamixel.h>
#include <serial.h>
#include "../error.h"
#include "../serialzigbee.h"
#include "../io.h"
#include "../timer.h"

#define SENSOR_FRONT 1
#define SENSOR_FRONTLEFT 3
#define SENSOR_FRONTRIGHT 4
#define SENSOR_BACKLEFT 2
#define SENSOR_BACKRIGHT 5
#define MOTOR_LEFT 11
#define MOTOR_RIGHT 12

#define MAX_SPEED_IN_PERCENTAGE 100ul
#define MIN_SPEED_IN_PERCENTAGE 25ul
#define DISTANCE_TO_BRAKE	200

#define DISTANCE_TO_TURN		25
#define DISTANCE_TO_RECOVER		10

#define WALL_MODE	0

#define NOISE_LEVEL	5

void timer0_compA()
{
	; // Do something every ms	
};

int timer0_initialize()
{	
	return 0;
}

static volatile int state = -1;

void reset_state()
{
	if (BTN_START_PRESSED)
		state = 0;
}

int get_state()
{
	int loc_state = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		loc_state = state;
	}	
	return loc_state;
}

void set_state(int new_state)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		state = new_state;
	}		
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
	io_set_interrupt(BTN_START, &reset_state);
	
	// Active general interrupts
	sei();
	
	// Set modes
	motor_set_mode(254, MOTOR_WHEEL_MODE);
	serial_set_zigbee();
	
	//dxl_write_word(254, 14, 1023);
			
	uint8_t ir_frontleft, ir_frontright, ir_backleft, ir_backright, dis_front;
	int speed_left = 0, speed_right = 0;
	int speed_diff = 0;
	int offset = 10;
	//uint8_t state = STATE_AVOID_OBSTACLES_DIFF;
	
	char direction_left = 0, direction_right = 0;
	
	uint16_t speed_l, speed_r;
	printf("PROGRAM IS RUNNING!\n");

	while(1) {
		dis_front = sensor_read(SENSOR_FRONT, DISTANCE);
		ir_frontleft = (sensor_read(SENSOR_FRONTLEFT, IR));
		ir_frontright = (sensor_read(SENSOR_FRONTRIGHT, IR));
		ir_backleft = sensor_read(SENSOR_BACKLEFT, IR);
		ir_backright = sensor_read(SENSOR_BACKRIGHT, IR);
		
		// Remove noise
		if (ir_frontleft < NOISE_LEVEL)
			ir_frontleft = 0;
		if (ir_frontright < NOISE_LEVEL)
			ir_frontright = 0;
		if (ir_backleft < NOISE_LEVEL)
			ir_backleft = 0;
		if (ir_backright < NOISE_LEVEL)
			ir_backright = 0;
			
		//if (state == 0 && (ir_frontleft > DISTANCE_TO_TURN) && (ir_frontright > DISTANCE_TO_TURN))
		//	state = 1;
		//else if (state == 1 && (ir_frontleft < DISTANCE_TO_RECOVER) && (ir_frontright < DISTANCE_TO_RECOVER) && (dis_front < 100))
		//	state = 0;
		
		switch (get_state())
		{
			case 0:
			{
				// State 0: Avoid obstacles
				LED_OFF(LED_PLAY);
				LED_OFF(LED_AUX);
			    // Going forward
				if (ir_frontright < DISTANCE_TO_TURN) {
				    direction_left = MOTOR_CCW;
				    speed_left = 255;//-10*ir_frontright;
			    }
			    else {
				    direction_left = MOTOR_CW;
				    speed_left = ir_frontright;
			    }
			    // Going forward
			    if (ir_frontleft < DISTANCE_TO_TURN) {
				    direction_right = MOTOR_CW;
				    speed_right = 255;//-10*ir_frontleft;
			    }
			    else {
				    direction_right = MOTOR_CCW;
				    speed_right = ir_frontleft;
			    }
				
				if (offset > 0)
					speed_right -= offset;
				else
					speed_left -= offset;
				
				if (WALL_MODE)
				{
					if(ir_backleft > 80)
					{
						set_state(2);
					}
				}						
				break;
			}
			/*
			case 1:
			{
				// State 1: Get out of stuck mode
				LED_ON(LED_PLAY);
				int speed_diff = ir_backleft - ir_backright;
				speed_left = 255/2 - speed_diff;
				direction_left = MOTOR_CW;
				speed_right = 255/2 + speed_diff;
				direction_right = MOTOR_CCW;

				break;	
			}
			*/
			case 2:
			{
				// State 2: Follow the left wall
				LED_ON(LED_AUX);
				if (ir_frontleft < 20)
				{
					speed_left = 255-40;
					speed_right = 255;
				}
				else
				{
					speed_left = 255;
					speed_right = 255-40;
				}
				// Go forward
				direction_left = MOTOR_CCW;
				direction_right = MOTOR_CW;
				if (ir_frontleft == 0)
					set_state(0);
					offset = -offset;			
				break;
			}
			default:
			{
				// default state: stop
				speed_left = 0;
				speed_right = 0;
				break;
			}							
        }		
		
		uint8_t dist = 0;
		if (DISTANCE_TO_BRAKE > dis_front)
			dist = DISTANCE_TO_BRAKE - dis_front;
		else
			dist = 0;
			
	    if (direction_left == MOTOR_CW && direction_right == MOTOR_CCW)
		 dist = 255;
	
		uint8_t speed = (uint8_t)(MAX_SPEED_IN_PERCENTAGE * (uint32_t)dist / DISTANCE_TO_BRAKE);
		if (speed < MIN_SPEED_IN_PERCENTAGE)
			speed = MIN_SPEED_IN_PERCENTAGE;		
			
		speed_l = (uint16_t) ((speed_left<<2) * (uint32_t)speed / 100ul);
		speed_r = (uint16_t) ((speed_right<<2) * (uint32_t)speed / 100ul);
		
		//printf("sl=%4d, sr=%4d, sl2=%4u, sr2=%4u\n", speed_left, speed_right, speed_l, speed_r);
		
		motor_set_speed_dir(MOTOR_LEFT, speed_l, direction_left);
		motor_set_speed_dir(MOTOR_RIGHT, speed_r, direction_right);			

		//printf("distance= %3d, left= %3d, right=%3d, speed_left=%3d, speed_right=%3d \n",touch,ir_left,ir_right,speed_left,speed_right);
		printf("f= %3u, fl= %3u, fr=%3u, bl= %3u, br=%3u, speed=%3d, speed_l=%3u, speed_r=%3u,\n",dis_front,ir_frontleft,ir_frontright,ir_backleft, ir_backright, speed_diff, speed_l, speed_r);
	}
	return 0;
}

