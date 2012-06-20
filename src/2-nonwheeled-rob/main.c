/*! \file 2-nonwheeled-rob/main.c
    \brief Runtime file with main function for the none-wheeled project 2 and 3
	\author Dennis Hellner
	\author Hans-Peter Wolf
	\copyright GNU Public License V3
	\date 2012
	
	\file 2-nonwheeled-rob/main.c
	
	\details This file ... Show diagram
*/
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

/// Number of motors
#define CONF_NUMBER_OF_MOTORS				6
/// Delay between motor position updates
#define CONF_MOTOR_UPDATE_POSITION_INTERVAL	20
/// Number of different possible movement directions
#define CONF_NUMBER_OF_MOVEMENTS				4

/// ID for movement direction forward
#define CONF_MOVEMENT_FORWARD		0
/// ID for movement direction backward
#define CONF_MOVEMENT_BACKWARD		1
/// ID for movement direction right
#define CONF_MOVEMENT_RIGHT			2
/// ID for movement direction left
#define CONF_MOVEMENT_LEFT			3

/// Port value for sensor in front (long distance)
#define CONF_SENSOR_FRONT			2
/// Port value for sensor at left (ir sensor)
#define CONF_SENSOR_LEFT				6
/// Port value for sensor at right (ir sensor)
#define CONF_SENSOR_RIGHT			1

/// Lower level for distance on front sensor for state "long away from wall"
#define CONF_SENSOR_FRONT_MIN_PROXIMITY		60
/// Upper level for distance on front sensor for state "Close to wall"
#define CONF_SENSOR_FRONT_MAX_PROXIMITY		150 

/// Upper level for distance on left sensor for state "Close to wall"
#define CONF_SENSOR_LEFT_MAX_PROXIMITY		40
/// Upper level for distance on right sensor for state "Close to wall"
#define CONF_SENSOR_RIGHT_MAX_PROXIMITY		40

/// Number of samples for calculating an average of values from front sensor
#define CONF_SENSOR_FRONT_NUMBER_OF_SAMPLES		128
/// Number of samples for calculating an average of values from left sensor
#define CONF_SENSOR_LEFT_NUMBER_OF_SAMPLES		16
/// Number of samples for calculating an average of values from right sensor
#define CONF_SENSOR_RIGHT_NUMBER_OF_SAMPLES		16
		
// Global variables
volatile uint8_t global_release = 0;
volatile uint8_t global_release_autonomous = 0;
volatile uint8_t global_use_zigbee = 0;
volatile uint32_t global_elapsed_time = 0; // elapsed time in ms
volatile uint8_t global_movement_type = CONF_MOVEMENT_FORWARD;
		
// **********************************************
// Configuration 
// **********************************************
/// ID names of the #CONF_NUMBER_OF_MOTORS (6) motors
uint8_t ids[CONF_NUMBER_OF_MOTORS] = {6, 1, 3, 8, 2, 5};
/// Amplitude of oscillations for each of #CONF_NUMBER_OF_MOTORS (6) in the #CONF_NUMBER_OF_MOVEMENTS (4) states.
const uint16_t amplitude[CONF_NUMBER_OF_MOVEMENTS][CONF_NUMBER_OF_MOTORS] ={
																				{120 / 2,       44 / 2,        512/2 - 20,    512/2 - 20,    44 / 2,        120 / 2},
																				{120 / 2,       44 / 2,        512/2 - 20,    512/2 - 20,    44 / 2,        120 / 2},
																				{(995-540)/2,   (512-480)/2,   (512-350)/2,   (512-350)/2,   (512-480)/2,   (995-540)/2},
																				{(995-540)/2,   (512-480)/2,   (512-350)/2,   (512-350)/2,   (512-480)/2,   (995-540)/2} };
// Frequency in Hz for each of #CONF_NUMBER_OF_MOTORS (6) in the #CONF_NUMBER_OF_MOVEMENTS (4) states.
const float frequency[CONF_NUMBER_OF_MOVEMENTS][CONF_NUMBER_OF_MOTORS] ={ 
																				{1,   1,   1,   1,   1,   1},
																				{1,   1,   1,   1,   1,   1},
																				{1,   1,   1,   1,   1,   1},
																				{1,   1,   1,   1,   1,   1} };
/// Phase angle in rad for each of #CONF_NUMBER_OF_MOTORS (6) in the #CONF_NUMBER_OF_MOVEMENTS (4) states.
const float angle[CONF_NUMBER_OF_MOVEMENTS][CONF_NUMBER_OF_MOTORS] = {
																				{M_PI/3,   M_PI/3,   0,        M_PI,     M_PI/3,   M_PI/3},
																				{M_PI/3,   M_PI/3,   M_PI,     0,        M_PI/3,   M_PI/3},
																				{0,        M_PI/3,   M_PI/2,   M_PI/2,   M_PI/3,   M_PI},
																				{M_PI,     M_PI/3,   M_PI/2,   M_PI/2,   M_PI/3,   0} };
																					
/// Offset for each of #CONF_NUMBER_OF_MOTORS (6) in the #CONF_NUMBER_OF_MOVEMENTS (4) states.
const uint16_t offset[CONF_NUMBER_OF_MOVEMENTS][CONF_NUMBER_OF_MOTORS] = {
																				{630,               556 + 50 - 70,     512/2+20,          512/2+20,          556 + 50 - 70,     630},
																				{630,               556 + 50 - 70,     512/2+20,          512/2+20,          556 + 50 - 70,     630 },
																				{(995-540)/2+540,   (512-480)/2+480,   (512-350)/2+350,   (512-350)/2+350,   (512-480)/2+480,   (995-540)/2+540},
																				{(995-540)/2+540,   (512-480)/2+480,   (512-350)/2+350,   (512-350)/2+350,   (512-480)/2+480,   (995-540)/2+540} };

/** Callback function for receiving data. 
  The function uses serial_read to get data from serial buffer. Everytime a new data is received LED RXD is toggled.
  Get data byte:
    - p: Prints positions of all motors
    - o: Prints values from sensors
    - w: Robot go forward
    - s: Robot go backward
    - a: Robot go left
    - d: Robot go right
    - q: Stop or start robot
    - z: Change between Zigbee and wired serial connection
    - r: Change between autonomus or controled driving
 */
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
			for (uint16_t i=0; i<CONF_NUMBER_OF_MOTORS; i++) {
				uint16_t v = motor_get_position(ids[i]);
				printf("Motor %d position is %u\n", ids[i], v);
			}
			break;
		}
		
		// Output sensor positions
		case 'o':
		{
			global_release = 0;
			uint16_t dist_front = sensor_read(CONF_SENSOR_FRONT, DISTANCE);
			uint16_t dist_left = sensor_read(CONF_SENSOR_LEFT, IR);
			uint16_t dist_right = sensor_read(CONF_SENSOR_RIGHT, IR);
			printf("Sensors: Front: %3u, Left: %3u, Right: %3u.\n", dist_front, dist_left, dist_right);
			break;
		}
		
		// Forward
		case 'w':
		{
			global_release = 1;
			global_movement_type = CONF_MOVEMENT_FORWARD;
			printf("Going forward.\n");
			break;
		}
		
		// Backward
		case 's':
		{
			global_release = 1;
			global_movement_type = CONF_MOVEMENT_BACKWARD;
			printf("Going backward.\n");
			break;
		}
		
		// Left
		case 'a':
		{
			global_release = 1;
			global_movement_type = CONF_MOVEMENT_LEFT;
			printf("Going left.\n");
			break;
		}
		
		// Right
		case 'd':
		{
			global_release = 1;
			global_movement_type = CONF_MOVEMENT_RIGHT;
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

/**
  Callback function for pressing start button.
  Is called when start button is pressed.
  Will toggle between robot running  not running mode.
  */
void btn_press_start(void)
{
	if (BTN_START_PRESSED)
		global_release ^= 1;
}

/**
  Updates the positions of motors using the cosinus algorithm A*cos(2*pi*f*t + angle) + offset
  \param[in]	time_in_ms		Is set from current time from the timer, and is used as t in the cosinus algorithm
  \param[in]    movement_type	Decides what values is used for each motors amplitude, frequency, angle and offset.
 */
void update_motor_position(uint16_t time_in_ms, uint8_t movement_type) {
	if (movement_type < CONF_NUMBER_OF_MOVEMENTS)
	{
		uint16_t pos[CONF_NUMBER_OF_MOTORS];
		for (int i=0; i<CONF_NUMBER_OF_MOTORS; i++)
		{
			float c = cos(2 * M_PI * frequency[movement_type][i] * (float)time_in_ms / 1000 + angle[movement_type][i]);
			pos[i] = amplitude[movement_type][i]*c + offset[movement_type][i];
		}
		motor_sync_move(CONF_NUMBER_OF_MOTORS, ids, pos, MOTOR_MOVE_NON_BLOCKING);		
	}
}

/** 
  Callback function for timer 0.
  Timer is running in 1kHz and adds 1 to the global value global_elapsed_time.
  */
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

double calc_simple_moving_avg(uint16_t * value_buffer, uint8_t buffer_size, uint8_t * new_index, uint16_t new_value,
	double old_moving_avg)
{
	// Check for validity of new buffer position
	if (*new_index >= buffer_size)
		*new_index = 0;
	// Calculate new simple moving average
	double new_moving_avg = old_moving_avg - (double)((double)value_buffer[*new_index] - (double)new_value) / buffer_size;
	// Store new value in buffer
	value_buffer[*new_index] = new_value;
	// Increment buffer index
	(*new_index)++;
	// Return new value moving average
	return new_moving_avg; 
}

int execute_autonomous_movement(uint8_t movement_type, uint16_t * dist_front_buffer,
	double dist_front_avg, double dist_left_avg, double dist_right_avg)
{
	// Autonomous movement logic
	if (movement_type == CONF_MOVEMENT_FORWARD && dist_front_avg > CONF_SENSOR_FRONT_MAX_PROXIMITY)
	{
		// Reset sensor buffer with correct values
		for (uint8_t i = 0; i < CONF_SENSOR_FRONT_NUMBER_OF_SAMPLES; i++)
			dist_front_buffer[i] = CONF_SENSOR_FRONT_MAX_PROXIMITY;
		printf("Going right, sensor: %.0f.\n", dist_front_avg);
		return CONF_MOVEMENT_RIGHT;
	}
	else if ((movement_type == CONF_MOVEMENT_LEFT || movement_type == CONF_MOVEMENT_RIGHT) &&
	dist_front_avg < CONF_SENSOR_FRONT_MIN_PROXIMITY)
	{
		// Reset sensor buffer with correct values
		for (uint8_t i = 0; i < CONF_SENSOR_FRONT_NUMBER_OF_SAMPLES; i++)
			dist_front_buffer[i] = CONF_SENSOR_FRONT_MIN_PROXIMITY;
		printf("Going forward, sensor: %.0f.\n", dist_front_avg);
		return CONF_MOVEMENT_FORWARD;
	}
	else if (movement_type == CONF_MOVEMENT_LEFT && dist_left_avg > CONF_SENSOR_LEFT_MAX_PROXIMITY)
	{
		printf("Found left wall, sensor: %.0f.\n", dist_left_avg);
		return CONF_MOVEMENT_RIGHT;
	}
	else if (movement_type == CONF_MOVEMENT_RIGHT && dist_right_avg > CONF_SENSOR_RIGHT_MAX_PROXIMITY)
	{
		printf("Found right wall, sensor: %.0f.\n", dist_right_avg);
		return CONF_MOVEMENT_LEFT;
	}
	else
		return movement_type;
}

/**
  main call function for the none-wheeled robot.
  Initializes serial connection, dynamixel connection, timer0 and 3 sensors. 
  Is running an infinity look:
	- Calculates a mean value for each sensor inputs.
	- When robot is in movement:
	  - Movement control
	    - When robot going forward and close to wall: Go right
	    - When robot is going left or right and is away from wall: Go forward
	    - When robot is going left and wall is close on left side: Go right
	    - When robot is going right and wall is close on right side: Go left
	  - Call update_motor_position function when timer value global_elapsed_time has riched time
  */
int main() {	
	// Initialize motor
	dxl_initialize(0, 1);
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
	sensor_init(CONF_SENSOR_FRONT, DISTANCE);
	sensor_init(CONF_SENSOR_LEFT, IR);
	sensor_init(CONF_SENSOR_RIGHT, IR);
	// Enable global interrupts
	sei();
	// Center motor position
	const uint16_t center_pos[CONF_NUMBER_OF_MOTORS] = {512, 512, 512, 512, 512, 512};
	motor_sync_move(CONF_NUMBER_OF_MOTORS, ids, center_pos, MOTOR_MOVE_BLOCKING);
	// Declare local variables
	uint8_t release = 0;
	uint32_t elapsed_time = 0;
	uint32_t last_elapsed_time = 0;
	uint8_t movement_type = 0;
	uint8_t last_movement_type = 0;
	// Variables to calculate simple moving average
	uint16_t dist_front_buffer[CONF_SENSOR_FRONT_NUMBER_OF_SAMPLES];
	double dist_front_avg = 0;
	uint16_t dist_left_buffer[CONF_SENSOR_LEFT_NUMBER_OF_SAMPLES];
	double dist_left_avg = 0;
	uint16_t dist_right_buffer[CONF_SENSOR_RIGHT_NUMBER_OF_SAMPLES];
	double dist_right_avg = 0;
	uint8_t dist_front_buffer_pointer = 0, dist_left_buffer_pointer = 0, dist_right_buffer_pointer = 0;
	while(1)
	{
		// Copy global variables for local usage to avoid race conditions
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			release = global_release;
			elapsed_time = global_elapsed_time;
			movement_type = global_movement_type;
		}
		
		// Read sensor measurements and calculate simple moving average to remove noise induced by movements
		dist_front_avg = calc_simple_moving_avg(dist_front_buffer, CONF_SENSOR_FRONT_NUMBER_OF_SAMPLES,
			&dist_front_buffer_pointer, sensor_read(CONF_SENSOR_FRONT, DISTANCE), dist_front_avg);
		dist_left_avg = calc_simple_moving_avg(dist_left_buffer, CONF_SENSOR_LEFT_NUMBER_OF_SAMPLES,
			&dist_left_buffer_pointer, sensor_read(CONF_SENSOR_LEFT, DISTANCE), dist_left_avg);
		dist_right_avg = calc_simple_moving_avg(dist_right_buffer, CONF_SENSOR_RIGHT_NUMBER_OF_SAMPLES,
			&dist_right_buffer_pointer, sensor_read(CONF_SENSOR_RIGHT, DISTANCE), dist_right_avg);

		// Check for release to move
		if (release)
		{
			
			// Check if autonomous control is active and execute in case
			if (global_release_autonomous)
			{
				uint8_t new_movement_type = execute_autonomous_movement(movement_type, dist_front_buffer,
					dist_front_avg, dist_left_avg, dist_right_avg);
				// Check for change in movement and update global variable in case
				if (new_movement_type != movement_type)
				{
					movement_type = new_movement_type;
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						global_movement_type = movement_type;
					}
				}
			}
				
			// Check for turn and execute in case
			if (movement_type != last_movement_type)
			{
				// Go to center position
				motor_sync_move(CONF_NUMBER_OF_MOTORS, ids, center_pos, MOTOR_MOVE_BLOCKING);
				// Reset timer
				timer_reset(timer);
				// Update position immediately
				last_elapsed_time = 0;
				// Store current value
				last_movement_type = movement_type;
			}
			
			// Check for position update and execute in case
			if ((elapsed_time - last_elapsed_time) > CONF_MOTOR_UPDATE_POSITION_INTERVAL)
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