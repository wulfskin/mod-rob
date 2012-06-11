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

#define MAX_SPEED_IN_PERCENTAGE 100ul
#define MIN_SPEED_IN_PERCENTAGE 25ul
#define DISTANCE_TO_BRAKE	200


void timer0_compA()
{
	; // Do something every ms	
};

int timer0_initialize()
{	

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
	//uint8_t state = STATE_AVOID_OBSTACLES_DIFF;
	
	char direction_left, direction_right;
	
	uint16_t speed_l, speed_r;
    char state = 0x00;
	printf("PROGRAM IS RUNNING!\n");

	while(1) {
		dis_front = sensor_read(SENSOR_FRONT, DISTANCE);
		ir_frontleft = (sensor_read(SENSOR_FRONTLEFT, IR));
		ir_frontright = (sensor_read(SENSOR_FRONTRIGHT, IR));
		ir_backleft = sensor_read(SENSOR_BACKLEFT, IR);
		ir_backright = sensor_read(SENSOR_BACKRIGHT, IR);

            if (state == 0x00) {
				LED_OFF(LED_PLAY);
				if (ir_frontleft < 6)
						ir_frontleft = 0;
				if (ir_frontright < 6)
					ir_frontright = 0;
				//if(ir_backleft >100 || ir_backleft>100)
					//state=1;
					
			    if (ir_frontright < 25) {
				    direction_left = MOTOR_CCW;
				    speed_left = 255;//-10*ir_frontright;
			    }
			    else {
				    direction_left = MOTOR_CW;
				    speed_left = ir_frontright;
			    }
				
				if (ir_frontleft < 25) {
					direction_right = MOTOR_CW;
					speed_right = 255;//-10*ir_frontleft;
				}
				else {
					direction_right = MOTOR_CCW;
					speed_right = ir_frontleft;
				}
				
			
				//if (speed_right > 255-dis_front && direction_right == MOTOR_CW)
				 
											
			}
			else if(state == 0x01){
				LED_ON(LED_PLAY);
				speed_left=0;
				speed_right=0;
				 
			}			
		
		
		uint8_t dist = 0;
		if (DISTANCE_TO_BRAKE > dis_front)
			dist = DISTANCE_TO_BRAKE - dis_front;
		else
			dist = 0;

		// In case we go backwards
		if ((direction_left == MOTOR_CW) && (direction_right == MOTOR_CCW))
		{
			dist = DISTANCE_TO_BRAKE;
		}
	
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

