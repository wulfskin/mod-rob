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
		
// Global variables
volatile uint8_t global_release = 0;
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
	// Initialize serial connection
	serial_initialize(57600);
	serial_set_rx_callback(&serial_receive_data);
	// Initialize timer
	uint8_t timer = 0;
	timer_set_interrupt(timer, TIT_OUTPUT_COMPARE_MATCH_A, &timer0_compare_match);
	timer_set_value(timer, TVT_OUTPUT_COMPARE_A, F_CPU / 64 / 1000 - 1);
	timer_init(timer, TOM_CLEAR_ON_COMPARE, TPS_DIV_64, 0);
	// Initialize I/O
	io_init();
	io_set_interrupt(BTN_START, &btn_press_start);
	// Enable global interrupts
	sei();
	// Define center position
	const uint16_t center_pos[NUMBER_OF_MOTORS] = {512, 512, 512, 512, 512, 512};
	// Go to center position
	motor_sync_move(NUMBER_OF_MOTORS, ids, center_pos, MOTOR_MOVE_BLOCKING);
	// Declare variables
	uint8_t release = 0;
	uint32_t elapsed_time = 0;
	uint32_t last_elapsed_time = 0;
	uint8_t movement_type = 0;
	uint8_t last_movement_type = 0;
	while(1)
	{
		// Copy global variables for local usage
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			release = global_release;
			elapsed_time = global_elapsed_time;
			movement_type = global_movement_type;
		}
		// Check for release to move
		if (release)
		{
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