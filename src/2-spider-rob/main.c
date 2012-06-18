#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
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

#define NUMBER_OF_MOTORS				6
#define MOTOR_UPDATE_POSITION_INTERVAL	20
#define NUMBER_OF_MOVEMENTS				4

#define MOVEMENT_FORWARD		0
#define MOVEMENT_BACKWARD		1
#define MOVEMENT_RIGHT			2
#define MOVEMENT_LEFT			3

#define SENSOR_FRONT			2
#define SENSOR_LEFT				6
#define SENSOR_RIGHT			1

#define SENSOR_FRONT_MIN_PROXIMITY		40
#define SENSOR_FRONT_MAX_PROXIMITY		150

#define SENSOR_LEFT_MAX_PROXIMITY		40
#define SENSOR_RIGHT_MAX_PROXIMITY		40

#define SENSOR_FRONT_NUMBER_OF_SAMPLES		64
#define SENSOR_LEFT_NUMBER_OF_SAMPLES		16
#define SENSOR_RIGHT_NUMBER_OF_SAMPLES		16
		
// Global variables
volatile uint8_t global_release = 0;
volatile uint8_t global_release_autonomous = 0;
volatile uint8_t global_use_zigbee = 0;
volatile uint32_t global_elapsed_time = 0; // elapsed time in ms
volatile uint8_t global_movement_type = MOVEMENT_FORWARD;
		
// **********************************************
// Configuration 
// **********************************************
// Motor IDs
uint8_t ids[NUMBER_OF_MOTORS] = {6, 1, 3, 8, 2, 5};
// Amplitude of oscillations
const uint16_t amplitude[NUMBER_OF_MOVEMENTS][NUMBER_OF_MOTORS] = { {120 / 2, 44 / 2, 512/2 - 20, 512/2 - 20, 44 / 2, 120 / 2},
	                                                                {120 / 2, 44 / 2, 512/2 - 20, 512/2 - 20, 44 / 2, 120 / 2},
																	{(995-540)/2, (512-480)/2, (512-350)/2, (512-350)/2, (512-480)/2, (995-540)/2},
																	{(995-540)/2, (512-480)/2, (512-350)/2, (512-350)/2, (512-480)/2, (995-540)/2} };
// Frequency in Hz
const float frequency[NUMBER_OF_MOVEMENTS][NUMBER_OF_MOTORS] = { {1, 1, 1, 1, 1, 1},
	                                                             {1, 1, 1, 1, 1, 1},
																 {1, 1, 1, 1, 1, 1},
																 {1, 1, 1, 1, 1, 1} };
// Phase angle in rad
const float angle[NUMBER_OF_MOVEMENTS][NUMBER_OF_MOTORS] = { {M_PI/3, M_PI/3, 0, M_PI, M_PI/3, M_PI/3},
															 {M_PI/3, M_PI/3, M_PI, 0, M_PI/3, M_PI/3},
															 {0, M_PI/3, M_PI/2, M_PI/2, M_PI/3, M_PI},
															 {M_PI, M_PI/3, M_PI/2, M_PI/2, M_PI/3, 0} };
// Offset
const uint16_t offset[NUMBER_OF_MOVEMENTS][NUMBER_OF_MOTORS] = { {630, 556 + 50 - 70, 512/2+20, 512/2+20, 556 + 50 - 70, 630},
																 {630, 556 + 50 - 70, 512/2+20, 512/2+20, 556 + 50 - 70, 630 },
																 {(995-540)/2+540, (512-480)/2+480, (512-350)/2+350, (512-350)/2+350, (512-480)/2+480, (995-540)/2+540},
																 {(995-540)/2+540, (512-480)/2+480, (512-350)/2+350, (512-350)/2+350, (512-480)/2+480, (995-540)/2+540} };


void serial_receive_data(void){
	// Toggle receive LED
	LED_TOGGLE(LED_RXD);
	// Receive data
	unsigned char data = 0;
	serial_read(&data, 1);
	
	switch (data)
	{
		// Output motor positions
		case 'p':
		{
			global_release = 0;
			printf("\nPositions of motors:\n");
			for (uint16_t i=0; i<6; i++) {
				uint16_t v = motor_get_position(ids[i]);
				printf("Motor %d position is %u\n", ids[i], v);
			}
			break;
		}
		
		// Output sensor positions
		case 'o':
		{
			global_release = 0;
			uint16_t dist_front = sensor_read(SENSOR_FRONT, DISTANCE);
			uint16_t dist_left = sensor_read(SENSOR_LEFT, IR);
			uint16_t dist_right = sensor_read(SENSOR_RIGHT, IR);
			printf("Sensors: Front: %3u, Left: %3u, Right: %3u.\n", dist_front, dist_left, dist_right);
			break;
		}
		
		// Forward
		case 'w':
		{
			global_release = 1;
			global_movement_type = MOVEMENT_FORWARD;
			printf("Going forward.\n");
			break;
		}
		
		// Backward
		case 's':
		{
			global_release = 1;
			global_movement_type = MOVEMENT_BACKWARD;
			printf("Going backward.\n");
			break;
		}
		
		// Left
		case 'a':
		{
			global_release = 1;
			global_movement_type = MOVEMENT_LEFT;
			printf("Going left.\n");
			break;
		}
		
		// Right
		case 'd':
		{
			global_release = 1;
			global_movement_type = MOVEMENT_RIGHT;
			printf("Going right.\n");			
			break;
		}
	
		// Stop
		case 'q':
		{
			// Toggle release
			global_release ^= 1;
			break;
		}
		
		// Enable ZigBee
		case 'z':
		{
			global_use_zigbee ^= 1;
			if (global_use_zigbee)
			{
				// Enable ZigBee connection
				serial_set_zigbee();
				printf("Using ZigBee connection.\n");
			}
			else
			{
				serial_set_wire();
				printf("Using wired connection.\n");
			}
			break;
		}
		
		// Enable autonomous control
		case 'r':
		{
			global_release_autonomous ^= 1;
			if (global_release_autonomous)
				printf("Autonomous control activated.\n");
			else
				printf("Autonomous control deactivated.\n");
			break;
		}
	}
}

void btn_press_start(void)
{
	if (BTN_START_PRESSED)
		global_release ^= 1;
}

void update_motor_position(uint16_t time_in_ms, uint8_t movement_type) {
	if (movement_type < NUMBER_OF_MOVEMENTS)
	{
		uint16_t pos[NUMBER_OF_MOTORS];
		for (int i=0; i<NUMBER_OF_MOTORS; i++)
		{
			float c = cos(2 * M_PI * frequency[movement_type][i] * (float)time_in_ms / 1000 + angle[movement_type][i]);
			pos[i] = amplitude[movement_type][i]*c + offset[movement_type][i];
		}
		motor_sync_move(NUMBER_OF_MOTORS, ids, pos, MOTOR_MOVE_NON_BLOCKING);		
	}
}

void timer0_compare_match(void)
{
	if (global_release)
	{
		global_elapsed_time++;
		LED_ON(LED_PLAY);
	}
	else
		LED_OFF(LED_PLAY);
	// Toggle live bit
	if (!(global_elapsed_time % 1000))
		LED_TOGGLE(LED_AUX);
}

int main() {
	// Initialize motor
	dxl_initialize(0,1);
	// Initialize serial connection and active ZigBee
	serial_initialize(57600);
	serial_set_zigbee();
	serial_set_rx_callback(&serial_receive_data);
	// Initialize timer
	uint8_t timer = 0;
	timer_set_interrupt(timer, TIT_OUTPUT_COMPARE_MATCH_A, &timer0_compare_match);
	timer_set_value(timer, TVT_OUTPUT_COMPARE_A, F_CPU / 64 / 1000 - 1);
	timer_init(timer, TOM_CLEAR_ON_COMPARE, TPS_DIV_64, 0);
	// Initialize I/O
	io_init();
	io_set_interrupt(BTN_START, &btn_press_start);
	// Initialize sensors
	sensor_init(SENSOR_FRONT, DISTANCE);
	sensor_init(SENSOR_LEFT, IR);
	sensor_init(SENSOR_RIGHT, IR);
	// Enable global interrupts
	sei();
	// Define center position
	const uint16_t center_pos[NUMBER_OF_MOTORS] = {512, 512, 512, 512, 512, 512};
	// Go to center position
	motor_sync_move(NUMBER_OF_MOTORS, ids, center_pos, MOTOR_MOVE_BLOCKING);
	// Declare local variables
	uint8_t release = 0;
	uint32_t elapsed_time = 0;
	uint32_t last_elapsed_time = 0;
	uint8_t movement_type = 0;
	uint8_t last_movement_type = 0;
	uint16_t dist_front[SENSOR_FRONT_NUMBER_OF_SAMPLES], dist_front_avg = 0;
	uint16_t dist_left[SENSOR_LEFT_NUMBER_OF_SAMPLES], dist_left_avg = 0;
	uint16_t dist_right[SENSOR_RIGHT_NUMBER_OF_SAMPLES], dist_right_avg = 0;
	uint8_t front_pointer = 0, left_pointer = 0, right_pointer = 0;
	while(1)
	{
		// Copy global variables for local usage
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			release = global_release;
			elapsed_time = global_elapsed_time;
			movement_type = global_movement_type;
		}
		// Read sensor measurements
		dist_front[front_pointer] = sensor_read(SENSOR_FRONT, DISTANCE);
		if (++front_pointer >= SENSOR_FRONT_NUMBER_OF_SAMPLES)
			front_pointer = 0;
		dist_left[left_pointer] = sensor_read(SENSOR_LEFT, IR);
		if (++left_pointer >= SENSOR_LEFT_NUMBER_OF_SAMPLES)
			left_pointer = 0;
		dist_right[right_pointer] = sensor_read(SENSOR_RIGHT, IR);
		if (++right_pointer >= SENSOR_RIGHT_NUMBER_OF_SAMPLES)
			right_pointer = 0;
		// Calculate average
		// Front
		dist_front_avg = 0;
		for (uint8_t i = 0; i < SENSOR_FRONT_NUMBER_OF_SAMPLES; i++)
			dist_front_avg += dist_front[i];
		dist_front_avg /= SENSOR_FRONT_NUMBER_OF_SAMPLES;
		// Left
		dist_left_avg = 0;
		for (uint8_t i = 0; i < SENSOR_LEFT_NUMBER_OF_SAMPLES; i++)
			dist_left_avg += dist_left[i];
		dist_left_avg /= SENSOR_LEFT_NUMBER_OF_SAMPLES;
		// Right
		dist_right_avg = 0;
		for (uint8_t i = 0; i < SENSOR_RIGHT_NUMBER_OF_SAMPLES; i++)
			dist_right_avg += dist_right[i];
		dist_right_avg /= SENSOR_RIGHT_NUMBER_OF_SAMPLES;
		// Check for release to move
		if (release)
		{
			
			// Check for autonomous control
			if (global_release_autonomous)
			{
				// Autonomous movement logic
				if (movement_type == MOVEMENT_FORWARD && dist_front_avg > SENSOR_FRONT_MAX_PROXIMITY)
				{
					for (uint8_t i = 0; i < SENSOR_FRONT_NUMBER_OF_SAMPLES; i++)
						dist_front[i] = SENSOR_FRONT_MAX_PROXIMITY;
					printf("Going right, sensor: %d.\n", dist_front_avg);
					movement_type = MOVEMENT_RIGHT;
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						global_movement_type = movement_type;		
					}
				}				
				else if ((movement_type == MOVEMENT_LEFT || movement_type == MOVEMENT_RIGHT) &&
					dist_front_avg < SENSOR_FRONT_MIN_PROXIMITY)
				{
					printf("Going forward, sensor: %u.\n", dist_front_avg);
					movement_type = MOVEMENT_FORWARD;
					for (uint8_t i = 0; i < SENSOR_FRONT_NUMBER_OF_SAMPLES; i++)
						dist_front[i] = SENSOR_FRONT_MIN_PROXIMITY;
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						global_movement_type = movement_type;
					}					
				}
				else if (movement_type == MOVEMENT_LEFT && dist_left_avg > SENSOR_LEFT_MAX_PROXIMITY)
				{
					printf("Found left wall, sensor: %u.\n", dist_left_avg);
					movement_type = MOVEMENT_RIGHT;
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						global_movement_type = movement_type;
					}				
				}
				else if (movement_type == MOVEMENT_RIGHT && dist_right_avg > SENSOR_RIGHT_MAX_PROXIMITY)
				{
					printf("Found right wall, sensor: %u.\n", dist_right_avg);
					movement_type = MOVEMENT_LEFT;
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						global_movement_type = movement_type;
					}
				}
			}				
			// Check for turn
			if (movement_type != last_movement_type)
			{
				// Go to center position
				motor_sync_move(NUMBER_OF_MOTORS, ids, center_pos, MOTOR_MOVE_BLOCKING);
				// Reset timer
				timer_reset(timer);
				// Update position
				last_elapsed_time = 0;
				// Store current value
				last_movement_type = movement_type;
			}
			// Check if positions should be updated
			if ((elapsed_time - last_elapsed_time) > MOTOR_UPDATE_POSITION_INTERVAL)
			{
				update_motor_position((uint16_t)elapsed_time, movement_type);
				// Store current time
				last_elapsed_time = elapsed_time;
			}					
		}
		// Print motor errors
		PrintErrorCode();
	}	
	return 0;
}