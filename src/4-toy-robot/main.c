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
#include <stdlib.h>
#include <string.h>

#include "../macro.h"
#include "../sensor.h"
#include "../motor.h"
#include <dynamixel.h>
#include "../serial.h"
#include "../error.h"
#include "../serialzigbee.h"
#include "../io.h"
#include "../timer.h"

#define CONF_SENSOR_FINGER	4
#define CONF_BUTTON_BITE	3

#define CONF_SENSOR_FINGER2	2
#define CONF_BUTTON_BITE2	1

#define CONF_MOTOR_NUMBER	4
#define CONF_MOTOR_MOVE		6
#define CONF_MOTOR_STILL	1
#define CONF_MOTOR_MOVE2	2
#define CONF_MOTOR_STILL2	8

#define CONF_MOTOR_STILL_OPEN		512
#define CONF_MOTOR_STILL2_OPEN		512
#define CONF_MOTOR_MOVE_OPEN		300
#define CONF_MOTOR_MOVE_CLOSE		512
#define CONF_MOTOR_MOVE2_OPEN		712
#define CONF_MOTOR_MOVE2_CLOSE		512

#define CONF_MOTOR_DISTANCE			10

#define CONF_FINGER_PROXIMITY_IN	32
#define CONF_FINGER_PROXIMITY_OUT	14

#define CONF_FINGER_IN_DELAY		1000ul

#define CONF_BITE_MIN_WAIT_TIME	2000ul
#define CONF_BITE_MAX_WAIT_TIME	15000ul

/// Elapsed time in ms
static volatile uint32_t global_elapsed_time = 0;

#define CONF_TEXT_NUMBER_OF_CHARS	30
#define CONF_TEXT_NUMBER_WIN_ROUND	4
const char text_win_round[CONF_TEXT_NUMBER_WIN_ROUND][CONF_TEXT_NUMBER_OF_CHARS] = {
                                                                                     "Good job!",
									                                                 "You got me!",
									                                                 "Well done!",
									                                                 "This round is yours!"
															                       };
															
#define CONF_TEXT_NUMBER_LOST_ROUND	4
const char text_lost_round[CONF_TEXT_NUMBER_LOST_ROUND][CONF_TEXT_NUMBER_OF_CHARS] = {
                                                                                       "Haha, I owned you!",
																                       "How did that bite feel?",
																                       "Did I injure you?",
																                       "Better luck next time!"
                                                                                     };															  	

void timer0_compare_match()
{
	global_elapsed_time++;
}

uint32_t get_random_future_timestamp(uint32_t current_time)
{
	return current_time + CONF_BITE_MIN_WAIT_TIME + (uint32_t)random() % (CONF_BITE_MAX_WAIT_TIME - CONF_BITE_MIN_WAIT_TIME);
}

/// Function to generate seed using heap memory data
unsigned short get_seed()
{
	unsigned short seed = 0;
	unsigned short *p = (unsigned short*) (RAMEND+1);
	extern unsigned short __heap_start;
	
	while (p >= &__heap_start + 1)
	seed ^= * (--p);
	return seed;
}

uint8_t calc_score_and_check_for_win(uint8_t * hum_points, uint8_t * comp_points)
{
	uint8_t res = 0;
	// Show result on LEDs
	LED_OFF(LED_PLAY || LED_PROGRAM || LED_MANAGE || LED_AUX || LED_RXD || LED_TXD);
	if (*hum_points >= 1)
		LED_ON(LED_PLAY);
	if (*hum_points >= 2)
		LED_ON(LED_PROGRAM);
	if (*hum_points >= 3)
		LED_ON(LED_MANAGE);
	if (*hum_points > 3)
	{
		printf("Congratz! You win!\n");
		*hum_points = 0;
		*comp_points = 0;
		res = 1;
	}
	if (*comp_points >= 1)
		LED_ON(LED_AUX);
	if (*comp_points >= 2)
		LED_ON(LED_RXD);
	if (*comp_points >= 3)
		LED_ON(LED_TXD);
	if (*comp_points > 3)
	{
		printf("I owned you!\n");
		*hum_points = 0;
		*comp_points = 0;
		res = 1;
	}
	return res;
}

void print_random_text(const char * text_array, uint8_t array_size)
{
	uint8_t index = (random() % array_size) * CONF_TEXT_NUMBER_OF_CHARS;
	char text[CONF_TEXT_NUMBER_OF_CHARS];
	strncpy(text, &text_array[index], CONF_TEXT_NUMBER_OF_CHARS);
	printf("%s \n", text);
}

int main() {
	
	// Seed random seed.
	srandom(get_seed());
	
	// Initialize timer for 1 kHz
	int timer=0;
	timer_set_interrupt(timer, TIT_OUTPUT_COMPARE_MATCH_A, &timer0_compare_match);
	timer_set_value(timer, TVT_OUTPUT_COMPARE_A, F_CPU / 64 / 1000 - 1);
	timer_init(timer, TOM_CLEAR_ON_COMPARE, TPS_DIV_64, 0);
	
	// Initialize other stuff		
	dxl_initialize(0,1);
	serial_initialize(57600);
	
	sensor_init(CONF_SENSOR_FINGER, SENSOR_IR);
	sensor_init(CONF_BUTTON_BITE, SENSOR_TOUCH);
	sensor_init(CONF_SENSOR_FINGER2, SENSOR_IR);
	sensor_init(CONF_BUTTON_BITE2, SENSOR_TOUCH);
	io_init();
	
	// Activate general interrupts
	sei();
	
	printf("\n\nWelcome to SharkBite!\n\nPress start to begin new game.\n");
	
	// Open mouth
	const uint8_t ids[CONF_MOTOR_NUMBER] = {CONF_MOTOR_STILL, CONF_MOTOR_MOVE, CONF_MOTOR_STILL2, CONF_MOTOR_MOVE2};
	uint16_t pos[CONF_MOTOR_NUMBER] = {CONF_MOTOR_STILL_OPEN, CONF_MOTOR_MOVE_OPEN, CONF_MOTOR_STILL2_OPEN, CONF_MOTOR_MOVE2_OPEN};
	
	motor_sync_move(CONF_MOTOR_NUMBER, ids, pos, MOTOR_MOVE_BLOCKING);
	
	// Wait for start button to begin
	while (!BTN_START_PRESSED);
	
	// Variables to count points
	uint8_t hum_points = 0, comp_points = 0;

	// Variables for timing and sensors
	uint32_t elapsed_time = 0;
	uint32_t future_timestamp = 0, future_timestamp2 = 0;
	uint8_t dist_finger = 0, dist_finger2 = 0, bitten = 0, last_bitten = 0, bitten2 = 0, last_bitten2 = 0;
	
	// Helper variables and edge detection
	uint8_t bite_request = 0, bite_request_old = 0, bite_request2 = 0, bite_request2_old = 0;
	uint8_t finger_in = 0, finger_in_old = 1;
		
	while(1) {
		// Copy global variables 
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			elapsed_time = global_elapsed_time;
		}
		
		// Calculate new future timestamp
		if (!future_timestamp)
			future_timestamp = get_random_future_timestamp(elapsed_time);
		if (!future_timestamp2)
			future_timestamp2 = get_random_future_timestamp(elapsed_time);
			
		// Read sensor values
		dist_finger = sensor_read(CONF_SENSOR_FINGER, SENSOR_IR);
		bitten = sensor_read(CONF_BUTTON_BITE, SENSOR_TOUCH);
		dist_finger2 = sensor_read(CONF_SENSOR_FINGER2, SENSOR_IR);
		bitten2 = sensor_read(CONF_BUTTON_BITE2, SENSOR_TOUCH);
		
		// Build internal flags
		bite_request = (elapsed_time >= future_timestamp);
		bite_request2 = (elapsed_time >= future_timestamp2);
		//finger_in = (dist_finger >= CONF_FINGER_PROXIMITY_IN));
		finger_in = 1;
		
		// Debug prints
		//if (bite_request != bite_request_old)
		//	printf("Bite request: %d.\n", bite_request);
		//if (finger_in != finger_in_old)
		//	printf("Finger in: %d.\n", finger_in);
		//printf("Sensor: %d %d.\n", dist_finger, finger_in);
			
		// Check if finger is in range
		if (finger_in)
		{
			if (bite_request)
			{
				// Give motor command to close mouth only once
				if (!bite_request_old)
					motor_move(CONF_MOTOR_MOVE, CONF_MOTOR_MOVE_CLOSE, MOTOR_MOVE_NON_BLOCKING);
				_delay_ms(15);
				uint16_t p = motor_get_position(CONF_MOTOR_MOVE);
				uint16_t dist = 0;
				// Calculate distance between current position and goal position
				if (p > CONF_MOTOR_MOVE_CLOSE)
					dist = p - CONF_MOTOR_MOVE_CLOSE;
				else
					dist = CONF_MOTOR_MOVE_CLOSE - p;
				if (bitten)
				{
					print_random_text(text_lost_round[0], CONF_TEXT_NUMBER_LOST_ROUND);
				}
				// Open mouth in case the goal has been reached or finger was bitten
				if (dist < CONF_MOTOR_DISTANCE || bitten)
				{
					motor_move(CONF_MOTOR_MOVE, CONF_MOTOR_MOVE_OPEN, MOTOR_MOVE_BLOCKING);
					future_timestamp = get_random_future_timestamp(elapsed_time);
					if (!bitten)
					{
						print_random_text(text_win_round[0], CONF_TEXT_NUMBER_WIN_ROUND);
						hum_points++;
					}
					else
						comp_points++;
					// Dirty hack to debounce button
					_delay_ms(10);
				}
			}
			if (bite_request2)
			{
				// Give motor command to close mouth only once
				if (!bite_request2_old)
					motor_move(CONF_MOTOR_MOVE2, CONF_MOTOR_MOVE2_CLOSE, MOTOR_MOVE_NON_BLOCKING);
				_delay_ms(15);
				uint16_t p = motor_get_position(CONF_MOTOR_MOVE2);
				uint16_t dist = 0;
				if (p > CONF_MOTOR_MOVE2_CLOSE)
					dist = p - CONF_MOTOR_MOVE2_CLOSE;
				else
					dist = CONF_MOTOR_MOVE2_CLOSE - p;
				if (bitten2)
				{
					print_random_text(text_lost_round[0], CONF_TEXT_NUMBER_LOST_ROUND);
				}
				// Open mouth in case the goal has been reached or finger was bitten
				if (dist < CONF_MOTOR_DISTANCE || bitten2)
				{
					motor_move(CONF_MOTOR_MOVE2, CONF_MOTOR_MOVE2_OPEN, MOTOR_MOVE_BLOCKING);
					future_timestamp2 = get_random_future_timestamp(elapsed_time);
					if (!bitten2)
					{
						print_random_text(text_win_round[0], CONF_TEXT_NUMBER_WIN_ROUND);
						hum_points++;
					}
					else
						comp_points++;
					// Dirty hack to debounce button
					_delay_ms(10);
				}
			}
		}
		// Path is never executed because finger_in is always true
		else
		{
			// If there is a request to bite or the finger has been removed
			if (bite_request || finger_in_old)
			{
				printf("Come on! Don't be a chicken!\n");
				future_timestamp = get_random_future_timestamp(elapsed_time);
			}
		}
		// If the button has been pressed without any reason!
		if ((bitten & !last_bitten || bitten2 & !last_bitten2) & !bite_request)
		{
			printf("Don't touch me!\n");
		}
		
		// Do scoring and check for end of game
		if (calc_score_and_check_for_win(&hum_points, &comp_points))
		{
			future_timestamp = 0;
			future_timestamp2 = 0;
			bite_request = 0;
			bite_request2 = 0;
			printf("\nPress start to begin new game.\n");
			// Wait for start button to begin
			while (!BTN_START_PRESSED);
		}			
		last_bitten = bitten;
		last_bitten2 = bitten2;		
		bite_request_old = bite_request;
		bite_request2_old = bite_request2;
		finger_in_old = finger_in;
		
		// Print motor errors
		PrintErrorCode();
	}
	return 0;
}

