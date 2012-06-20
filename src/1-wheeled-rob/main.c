/*! \file 1-wheeled-rob/main.c
    \brief Runtime file with main function for the wheeled project 1
	\author Walter Gambelunghe
	\copyright GNU Public License V3
	\date 2012
	
	\file 1-wheeled-rob/main.c
	
	\details \details This file implements the control logic for the autonomous wheeled robot in project 1, as shown
	by the following flowchart.<br><img src="../../doc/img/wheeled_flowchart.jpg">
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

///Symbolic map for front sensor port
#define SENSOR_FRONT		1
///Symbolic map for front-left sensor port
#define SENSOR_FRONTLEFT	3
///Symbolic map for front-right sensor port
#define SENSOR_FRONTRIGHT	4
///Symbolic map for back-left sensor port
#define SENSOR_BACKLEFT		2
///Symbolic map for back-right sensor port
#define SENSOR_BACKRIGHT	5

/// Used to filter sensor values. Values below this lever will be treated as zero
#define NOISE_LEVEL	5

///Symbolic definition for left motor id
#define MOTOR_LEFT	11
///Symbolic definition for right motor id
#define MOTOR_RIGHT 12

///Maximum allowed speed in percentage. Used for adaptive speed calculation.
#define MAX_SPEED_IN_PERCENTAGE 100ul
///Minimum allowed speed in percentage. Used for adaptive speed calculation.
#define MIN_SPEED_IN_PERCENTAGE 25ul


#define DISTANCE_TO_BRAKE		200
#define DISTANCE_TO_TURN			25

///Symbolic definition for the stopped state
#define STATE_STOPPED -1
///Symbolic definition for the Braitenberg state
#define STATE_BRAITENBERG 0
///Symbolic definition for the wall-follow state
#define STATE_FOLLOW 2

///Enable/Disable wall-following behavior
#define WALL_MODE	0		

//Function prototypes
void reset_state(void);
int get_state(void);
void set_state(int new_state);
void firmware_init(void);
void controller_run(void);

/// Global status variable 
static volatile int state = -1;

int main() {
	
	firmware_init();	//initialize firmware
	controller_run();	//run control logic
	
	return -1;	//should never get here
}


void firmware_init(){
	// Initialize firmware
	dxl_initialize(0,1);		//initialize dynamixel communication
	serial_initialize(57600);		//initialize serial communication
	
	//initialize sensors
	sensor_init(SENSOR_FRONT,SENSOR_DISTANCE);
	sensor_init(SENSOR_FRONTLEFT,SENSOR_IR);
	sensor_init(SENSOR_FRONTRIGHT,SENSOR_IR);
	sensor_init(SENSOR_BACKLEFT,SENSOR_IR);
	sensor_init(SENSOR_BACKRIGHT,SENSOR_IR);
	
	//initialize I/O
	io_init();
	io_set_interrupt(BTN_START, &reset_state); //assign callback function when start button is pressed 
	
	// Activate general interrupts
	sei();
	
	// Set motors to wheel mode
	motor_set_mode(254, MOTOR_WHEEL_MODE);
	
	// Set serial communication through ZigBee
	serial_set_zigbee();
	
}

controller_run(){
	//Declare variables
	///Store sensors readings
	uint8_t ir_frontleft, ir_frontright, ir_backleft, ir_backright, dis_front;
	
	///Speeds to be managed by the control logic, without taking into account speed adaption
	int speed_left = 0, speed_right = 0;
	
	///Speeds applied to the motors after adaptive speed calculation
	uint16_t speed_l, speed_r;
	
	///Speed offset, to make sure initially heading slightly to the right and finally slightly to the left
	int offset = 10;
	
	///Directions of the wheels
	char direction_left = 0, direction_right = 0;
	
	//main loop
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
		
		switch (get_state())
		{
			case STATE_BRAITENBERG:
			{
				// State 0: Avoid obstacles
				LED_OFF(LED_PLAY);
				LED_OFF(LED_AUX);
				
				if (ir_frontright < DISTANCE_TO_TURN) { //no obstacle close
					direction_left = MOTOR_CCW;		// Going forward
					speed_left = 255;				//At full speed
				}
				else {		//close to an obstacle
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
				
				if (WALL_MODE) //If wall follow mode is activated
				{
					if(ir_backleft > 80) //If close to wall
					{
						set_state(STATE_FOLLOW);	//Go to wall-follow state
					}
				}
				break;
			}
			
			case STATE_FOLLOW:		// State: Follow the left wall
			{
				LED_ON(LED_AUX);
				if (ir_frontleft < 20)	//too far
				{
					speed_left = 255-40;	//Head towards the wall
					speed_right = 255;
				}
				else
				{						//too close
					speed_left = 255;	//head away from the wall
					speed_right = 255-40;
				}
				// Go forward
				direction_left = MOTOR_CCW;
				direction_right = MOTOR_CW;
				if (ir_frontleft == 0)	{	//If arrived to the end of the wall
					set_state(STATE_BRAITENBERG);			//Return to Obstacle avoidance state
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
		}//close switch statement
		
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
		
		//Calculate motors speed
		speed_l = (uint16_t) ((speed_left<<2) * (uint32_t)speed / 100ul);
		speed_r = (uint16_t) ((speed_right<<2) * (uint32_t)speed / 100ul);
		
		//Apply motors speed and direction
		motor_set_speed_dir(MOTOR_LEFT, speed_l, direction_left);
		motor_set_speed_dir(MOTOR_RIGHT, speed_r, direction_right);

	}//close main loop
}//close main function

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