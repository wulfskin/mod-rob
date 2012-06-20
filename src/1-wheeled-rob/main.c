/*
 * main.c
 *
 * Created: 
 *  Author: Walter GAmbelunghe
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
#include "../serial.h"
#include "../error.h"
#include "../serialzigbee.h"
#include "../io.h"
#include "../timer.h"

#define SENSOR_FRONT		1
#define SENSOR_FRONTLEFT	3
#define SENSOR_FRONTRIGHT	4
#define SENSOR_BACKLEFT		2
#define SENSOR_BACKRIGHT	5

#define NOISE_LEVEL	5

#define MOTOR_LEFT	11
#define MOTOR_RIGHT 12

#define MAX_SPEED_IN_PERCENTAGE 100ul
#define MIN_SPEED_IN_PERCENTAGE 25ul

#define DISTANCE_TO_BRAKE		200
#define DISTANCE_TO_TURN		25
#define DISTANCE_TO_RECOVER		10

#define WALL_MODE	0		//Enable wall-following behavior

//Function prototypes
void reset_state(void);
int get_state();
void set_state(int new_state);
void firmware_init();
void controller_run();

static volatile int state = -1;

int main() {
	
	firmware_init();
	controller_run();
	return 1;
}

void firmware_init(){
	// Initialize firmware
	dxl_initialize(0,1);		//initialize dynamixel communication
	serial_initialize(57600);	//initialize serial communication
	//initialize sensors
	sensor_init(SENSOR_FRONT,SENSOR_DISTANCE);
	sensor_init(SENSOR_FRONTLEFT,SENSOR_IR);
	sensor_init(SENSOR_FRONTRIGHT,SENSOR_IR);
	sensor_init(SENSOR_BACKLEFT,SENSOR_IR);
	sensor_init(SENSOR_BACKRIGHT,SENSOR_IR);
	//initialize I/O
	io_init();
	io_set_interrupt(BTN_START, &reset_state);
	
	// Activate general interrupts
	sei();
	
	// Set motors to wheel mode
	motor_set_mode(254, MOTOR_WHEEL_MODE);
	// Set serial communication through ZigBee
	serial_set_zigbee();
	
}

controller_run(){
	//Declare variables
	uint8_t ir_frontleft, ir_frontright, ir_backleft, ir_backright, dis_front;
	int speed_left = 0, speed_right = 0;
	int speed_diff = 0;
	int offset = 10;
	//uint8_t state = STATE_AVOID_OBSTACLES_DIFF;
	
	char direction_left = 0, direction_right = 0;
	
	uint16_t speed_l, speed_r;
	printf("PROGRAM IS RUNNING!\n");

	while(1) {
		//Read sensor values
		dis_front = sensor_read(SENSOR_FRONT, SENSOR_DISTANCE);
		ir_frontleft = (sensor_read(SENSOR_FRONTLEFT, SENSOR_IR));
		ir_frontright = (sensor_read(SENSOR_FRONTRIGHT, SENSOR_IR));
		ir_backleft = sensor_read(SENSOR_BACKLEFT, SENSOR_IR);
		ir_backright = sensor_read(SENSOR_BACKRIGHT, SENSOR_IR);
		
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
				
				if (ir_frontright < DISTANCE_TO_TURN) { //no obstacle close
					direction_left = MOTOR_CCW;		// Going forward
					speed_left = 255;				//At full speed
				}
				else {								//close to an obstacle
					direction_left = MOTOR_CW;		//Going backwards
					speed_left = ir_frontright;		//speed proportional to obstacle distance (proximity)
				}
				// Going forward
				if (ir_frontleft < DISTANCE_TO_TURN) { //same as above for the other wheel
					direction_right = MOTOR_CW;
					speed_right = 255;
				}
				else {
					direction_right = MOTOR_CCW;
					speed_right = ir_frontleft;
				}
				
				if (offset > 0)		//apply speed offset to left/right wheel (before/after wall mode)
				speed_right -= offset;
				else
				speed_left -= offset;
				//If wall follow is activated
				if (WALL_MODE)
				{
					if(ir_backleft > 80) //If close to wall
					{
						set_state(2);	//Go to wall-follow state
					}
				}
				break;
			}
			
			case 2:		// State 2: Follow the left wall
			{
				LED_ON(LED_AUX);
				if (ir_frontleft < 20)	//too far
				{
					speed_left = 255-40;
					speed_right = 255;
				}
				else
				{						//too close
					speed_left = 255;
					speed_right = 255-40;
				}
				// Go forward
				direction_left = MOTOR_CCW;
				direction_right = MOTOR_CW;
				if (ir_frontleft == 0)	{	//If arrived to the end of the wall
					set_state(0);			//Return to Obstacle avoidance state
					offset = -offset;		//Invert speed offset (always head slightly to the left)
				}		
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
		
		//Calculate front distance, providing min distance to be able to stop/turn on time
		uint8_t dist = 0;
		if (DISTANCE_TO_BRAKE > dis_front)
		dist = DISTANCE_TO_BRAKE - dis_front;
		else
		dist = 0;
		//unless already turning
		if (direction_left == MOTOR_CW && direction_right == MOTOR_CCW)
		dist = 255;
		
		//Adjust speed according to distance
		uint8_t speed = (uint8_t)(MAX_SPEED_IN_PERCENTAGE * (uint32_t)dist / DISTANCE_TO_BRAKE);
		if (speed < MIN_SPEED_IN_PERCENTAGE)
		speed = MIN_SPEED_IN_PERCENTAGE;
		
		//Calculate motors' speed
		speed_l = (uint16_t) ((speed_left<<2) * (uint32_t)speed / 100ul);
		speed_r = (uint16_t) ((speed_right<<2) * (uint32_t)speed / 100ul);
		
		//Set motors' speed and direction
		motor_set_speed_dir(MOTOR_LEFT, speed_l, direction_left);
		motor_set_speed_dir(MOTOR_RIGHT, speed_r, direction_right);

		
		//printf("f= %3u, fl= %3u, fr=%3u, bl= %3u, br=%3u, speed=%3d, speed_l=%3u, speed_r=%3u,\n",dis_front,ir_frontleft,ir_frontright,ir_backleft, ir_backright, speed_diff, speed_l, speed_r);
	}
}

void reset_state() {
	if (BTN_START_PRESSED)
		state = 0;
}

int get_state() {
	int loc_state = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		loc_state = state;
	}
	return loc_state;
}

void set_state(int new_state) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		state = new_state;
	}
}