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

#define SENSOR_FINGER 4
#define MOTOR_LOWER	6
#define MOTOR_UPPER	1
#define BUTTON_BITE	3

#define MIN_WAIT_TIME	2000ul
#define MAX_WAIT_TIME	30000ul

/// Elapsed time in ms
static volatile uint32_t global_elapsed_time = 0;

void timer0_compare_match()
{
	global_elapsed_time++;
}

#define GET_RANDOM_TIMESTAMP(x)		((x) + MIN_WAIT_TIME + (uint32_t)(random() % (MAX_WAIT_TIME - MIN_WAIT_TIME)))

int main() {
	
	int timer=0;
	timer_set_interrupt(timer, TIT_OUTPUT_COMPARE_MATCH_A, &timer0_compare_match);
	timer_set_value(timer, TVT_OUTPUT_COMPARE_A, F_CPU / 64 / 1000 - 1);
	timer_init(timer, TOM_CLEAR_ON_COMPARE, TPS_DIV_64, 0);
	
	// Initialize stuff		
	dxl_initialize(0,1);
	serial_initialize(57600);
	
	sensor_init(SENSOR_FINGER,IR);
	sensor_init(BUTTON_BITE,TOUCH);
	io_init();
	
	// Active general interrupts
	sei();
	
	
	printf("PROGRAM IS RUNNING!\n");
	
	motor_set_position(MOTOR_UPPER, 812, MOTOR_MOVE_BLOCKING);
	motor_set_position(MOTOR_LOWER, 512, MOTOR_MOVE_BLOCKING);
	
	while(!BTN_START_PRESSED);// WAIT FOREVER
	
	int randomdelay=0;
	int points=0;
	randomdelay=random()%30000;
	printf("random %d \n" ,randomdelay);
	//set timer to random number

	uint32_t elapsed_time = 0;
	uint32_t new_timestamp = 0;	
		
	while(1) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			elapsed_time = global_elapsed_time;
		}
		if (elapsed_time == 0)
		{
			new_timestamp = GET_RANDOM_TIMESTAMP(elapsed_time);
			printf("Current %u, Future %u\n", elapsed_time, new_timestamp);
		}			
		if (elapsed_time >= new_timestamp)
		{
			motor_set_position(MOTOR_UPPER, 512, MOTOR_MOVE_NON_BLOCKING);
			if(motor_get_position(MOTOR_UPPER) <600){
				if (sensor_read(BUTTON_BITE,TOUCH)>10){
					//bitten
					points++;
				}				
				else
					//not bitten
					points--;
				motor_set_position(MOTOR_UPPER,812,MOTOR_MOVE_BLOCKING);
				new_timestamp = GET_RANDOM_TIMESTAMP(elapsed_time);
				printf("Current %u, Future %u\n", elapsed_time, new_timestamp);
			}
		}							
	}
	return 0;
}

