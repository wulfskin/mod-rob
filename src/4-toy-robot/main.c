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
#include "../serial.h"
#include "../error.h"
#include "../serialzigbee.h"
#include "../io.h"
#include "../timer.h"

#define SENSOR_FRONTLEFT 3
#define SENSOR_FRONTRIGHT 4
#define NOISE_LEVEL	5
#define MOTOR_LOWER	6
#define MOTOR_UPPER	1


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


int main() {
	timer0_initialize();
	// Initialize stuff		
	dxl_initialize(0,1);
	serial_initialize(57600);
	sensor_init(SENSOR_FRONTLEFT,IR);
	sensor_init(SENSOR_FRONTRIGHT,IR);
	io_init();
	io_set_interrupt(BTN_START, &reset_state);
	
	// Active general interrupts
	sei();
	
			
	uint8_t ir_frontleft, ir_frontright;
	uint16_t motor_pos_upper,motor_pos_lower;
	signed int difference;	
	//uint8_t state = STATE_AVOID_OBSTACLES_DIFF;
	
	char direction_left = 0, direction_right = 0;
	printf("PROGRAM IS RUNNING!\n");


	motor_set_position(MOTOR_BROADCAST_ID, 512,1);
	//motor_set_position(MOTOR_UPPER, 512,1);
	dxl_write_byte(MOTOR_BROADCAST_ID,TORQUE_ENABLE,0);
	while(!BTN_START_PRESSED);// WAIT FOREVER
	
	while(1) {
		ir_frontleft = (sensor_read(SENSOR_FRONTLEFT, IR));
		ir_frontright = (sensor_read(SENSOR_FRONTRIGHT, IR));
		motor_pos_lower = motor_get_position(MOTOR_LOWER);	//read motor 
		_delay_ms(100);
		motor_pos_upper = motor_get_position(MOTOR_UPPER);	//read motor 
		_delay_ms(100);
		difference=ir_frontleft-ir_frontright;
		printf("left %d right %d MposUp %d \n",ir_frontleft,ir_frontright,motor_pos_upper);
		//motor_set_position(MOTOR_UPPER, motor_pos_lower - difference,0);
		_delay_ms(100);
		motor_set_position(MOTOR_LOWER, motor_pos_upper + difference,0);
	_delay_ms(300);
	dxl_write_byte(MOTOR_UPPER,TORQUE_ENABLE,0);

		
	}
	return 0;
}

