/*! \file 2-nonwheeled-rob/main.c
    \brief Application code for the non-wheeled remote-controlled and autonomous robot _Squid_.
	\author Dennis Hellner
	\author Hans-Peter Wolf
	\copyright GNU Public License V3
	\date 2012
	
	\file 2-nonwheeled-rob/main.c
	
	\details This file contains the application code for the _Squid_ robot in both
	remote-controlled (I) and autonomous mode (II). Both versions share the same
	source code. The Squid robot is a robot with four legs
	and six degrees of freedom (DOF). It can move forward, backwards and sidewards 
	(both to the left and right). It is equipped with a long distance sensor in front
	and two short distance sensors on the left and right side. In order to understand
	the following source code it is first important to have a basic understanding of how
	the robot actually moves. This is given as follows.
	
	In order to move the robot one pair of legs lifts the robot and the other pair reaches
	forward. Then the lifting legs drop	the robot again while the forward reaching legs are
	pushing the robot to the front.	Movements to the back and to the side are achieved in the
	exact same manner. In order to let the legs reach forward without touching the ground, the
	lifting legs should be higher. This is achieved by unequal pairs of legs. Two of the four
	legs have two DOF and two have only one DOF. This second degree of freedom allows them
	to lift the robot high enough or bend them strong enough to reach forward while the other
	pair can do its job without any further DOF.
	
	The control of the six robot motors is done via a very simple central pattern generator (CPG),
	a simple harmonic oscillator with an #amplitude, #frequency, #phase shift and #offset. As the
	sinusoidal motor position signals differs depending on the movement direction and the motor position,
	the former arrays contain the values with respect to both of them. Given the former information the
	motor position can be updated with a new position. This is done in the function #update_motor_position.
	In order to limit the traffic on the motor bus the position is only updated every certain time interval
	(#CONF_MOTOR_UPDATE_POSITION_INTERVAL).
	
	In order to set the right position at the right time the timing has to be precise. Thus the 8-bit timer 0
	is used to generate an interrupt at 1 kHz frequency. In that interrupt #timer0_compare_match the global variable
	#global_elapsed_time is incremented. This variable now provides a timestamp in milliseconds precision.
	
	The main application logic is provided in the #main method. At the beginning several firmware functions are called
	to initialize motors, sensors, serial connection, timer and I/O. Then the motors are positioned in a defined center
	position. Now the main control loop starts. In that loop first the global variables for movement release
	(#global_release), movement direction (#global_movement_type), elapsed time (#global_elapsed_time) and autonomous
	mode release (#global_release_autonomous) are copied to local variables to avoid race conditions. Then the sensor
	values are read and a simple moving average (#calc_simple_moving_avg) with a fixed history is calculated to reduce
	the noise induced by movement. In a third step it is checked whether the autonomous mode is activated and the
	movement direction is calculated from the sensor inputs in case (#execute_autonomous_movement). Then it is checked
	whether the movement direction has changed since the last time. If this is the case the robot moves its motors to the
	center position and atomically update the global movement direction. Also a flag is set to make sure that the motor
	positions get updated immediately despite the former mentioned interval. Finally the motor positions are updated
	as former described.
	
	In remote-controlled mode the movement direction is set with commands over the serial communication line. The callback
	function #serial_receive_data is called whenever a new character is received. Depending on the command it sets the
	global movement release, the global movement direction or a few other parameters. See the description of the callback
	function for more information. For safety reasons another way to set the movement release is by pressing the start
	button on the controller. Pressing it executes another callback	function #btn_press_start in order to set or remove
	the release.
	
	In non-autonomous mode the movement direction is set depending on the sensor inputs and a simple logic. By default the
	robot moves forward. In case it comes close to an obstacle in front it changes its movement direction to the right until
	there is no obstacle in front anymore. Then it goes forward again.
	To avoid oscillation due to sensor noise a hysteresis between the two states is implemented. If the robot is going right
	and comes to close to an obstacle on the right side, it inverts its movement. The same goes for the movement to the left
	and an obstacle on the left side. At the moment the robot never goes back and thus has no sensors there as well. More 
	information are found in the description of the specific function #execute_autonomous_movement.
	
	An overview of the main logic with the most important parts is given in the following figure.
	<a name="main_logic"><img src="2_nonwheel_overall.png" width="652" height="1275" alt="Overview of Squid's application logic"></a>
	
*/
#include <stdio.h>
#include <math.h>
#include <util/atomic.h>

#include "../macro.h"
#include "../sensor.h"
#include "../motor.h"
#include <dynamixel.h>
#include "../serial.h"
#include "../serialzigbee.h"
#include "../io.h"
#include "../timer.h"

/// Number of motors.
#define CONF_NUMBER_OF_MOTORS				6
/// Delay between motor position updates in order to reduce motor bus traffic.
#define CONF_MOTOR_UPDATE_POSITION_INTERVAL	20

/// Number of different possible movement directions.
#define CONF_NUMBER_OF_MOVEMENTS				4
/// ID for movement direction forward.
#define CONF_MOVEMENT_FORWARD		0
/// ID for movement direction backward.
#define CONF_MOVEMENT_BACKWARD		1
/// ID for movement direction right.
#define CONF_MOVEMENT_RIGHT			2
/// ID for movement direction left.
#define CONF_MOVEMENT_LEFT			3

/// Port value for sensor in front (DMS sensor).
#define CONF_SENSOR_FRONT			2
/// Port value for sensor on the left side (IR sensor).
#define CONF_SENSOR_LEFT				6
/// Port value for sensor on the right side (IR sensor).
#define CONF_SENSOR_RIGHT			1

/** Minimum necessary proximity towards an obstacle in front in order to start avoidance.
	Together with #CONF_SENSOR_FRONT_MAX_PROXIMITY both variables define a hysteresis
	of proximity in which the vehicle avoids an obstacle in front of the vehicle in autonomous mode. If an
	an obstacle in front is less proximal than #CONF_SENSOR_FRONT_MIN_PROXIMITY the vehicle
	will not try to avoid it. In contrast if an obstacle is more proximal than 
	#CONF_SENSOR_FRONT_MAX_PROXIMITY then obstacle avoidance is active. In between the vehicle keeps
	it current course in order to avoid oscillations due to sensor noise.
 */

#define CONF_SENSOR_FRONT_MIN_PROXIMITY		60
/** Maximum allowed proximity towards an obstacle in front.
	Together with #CONF_SENSOR_FRONT_MIN_PROXIMITY both variables define a hysteresis
	of proximity in which the vehicle avoids an obstacle in front of the vehicle in autonomous mode. If an
	an obstacle in front is less proximal than #CONF_SENSOR_FRONT_MIN_PROXIMITY the vehicle
	will not try to avoid it. In contrast if an obstacle is more proximal than 
	#CONF_SENSOR_FRONT_MAX_PROXIMITY then obstacle avoidance is active. In between the vehicle keeps
	it current course in order to avoid oscillations due to sensor noise.
 */
#define CONF_SENSOR_FRONT_MAX_PROXIMITY		150 

/// Maximum allowed proximity towards an obstacle on the left side.
#define CONF_SENSOR_LEFT_MAX_PROXIMITY		40
/// Maximum allowed proximity towards an obstacle on the right side.
#define CONF_SENSOR_RIGHT_MAX_PROXIMITY		40

/// Number of samples used for calculating an average of values from the front sensor.
#define CONF_SENSOR_FRONT_NUMBER_OF_SAMPLES		128
/// Number of samples used for calculating an average of values from the sensor on the left side.
#define CONF_SENSOR_LEFT_NUMBER_OF_SAMPLES		16
/// Number of samples used for calculating an average of values from the sensor on the right side.
#define CONF_SENSOR_RIGHT_NUMBER_OF_SAMPLES		16

/// Global movement release.
static volatile uint8_t global_release = 0;
/// Global release for autonomous mode.
static volatile uint8_t global_release_autonomous = 0;
/// Global release for using the wireless ZigBee connection.
static volatile uint8_t global_use_zigbee = 0;
/// Global timer value in milliseconds.
static volatile uint32_t global_elapsed_time = 0;
/// Global movement direction.
static volatile uint8_t global_movement_type = CONF_MOVEMENT_FORWARD;
		
/// Array of IDs of the motors to control.
static const uint8_t ids[CONF_NUMBER_OF_MOTORS] = {6, 1, 3, 8, 2, 5};
	
/** Array of amplitudes by movement direction and motor for sinusoidal position signal in position measurement unit.
\details This array stores for each movement direction and motor the amplitude
	\f$ A \f$ of the sinusoidal position signal.
	The motor position signal is a sinusoidal signal of the type
	\f$ position(t) = A * cos(2 \pi f t +  \theta) + off \f$.
 */
static const uint16_t amplitude[CONF_NUMBER_OF_MOVEMENTS][CONF_NUMBER_OF_MOTORS] =
  {
    {120 / 2,       44 / 2,        512/2 - 20,    512/2 - 20,    44 / 2,        120 / 2},
    {120 / 2,       44 / 2,        512/2 - 20,    512/2 - 20,    44 / 2,        120 / 2},
    {(995-540)/2,   (512-480)/2,   (512-350)/2,   (512-350)/2,   (512-480)/2,   (995-540)/2},
    {(995-540)/2,   (512-480)/2,   (512-350)/2,   (512-350)/2,   (512-480)/2,   (995-540)/2}
  };

/** Array of frequencies by movement direction and motor for sinusoidal position signal in Hz.
	\details This array stores for each movement direction and motor the frequency
	\f$ f \f$ of the sinusoidal position signal.
	The motor position signal is a sinusoidal signal of the type
	\f$ position(t) = A * cos(2 \pi f t +  \theta) + off \f$.
 */
static const const float frequency[CONF_NUMBER_OF_MOVEMENTS][CONF_NUMBER_OF_MOTORS] =
  { 
    {1,   1,   1,   1,   1,   1},
    {1,   1,   1,   1,   1,   1},
    {1,   1,   1,   1,   1,   1},
    {1,   1,   1,   1,   1,   1}
  };
																		 
/** Array of phase angles by movement direction and motor for sinusoidal position signal in rad.
	\details This array stores for each movement direction and motor the phase shift
	\f$ \theta \f$ of the sinusoidal position signal.
	The motor position signal is a sinusoidal signal of the type
	\f$ position(t) = A * cos(2 \pi f t +  \theta) + off \f$.
 */
static const const float phase[CONF_NUMBER_OF_MOVEMENTS][CONF_NUMBER_OF_MOTORS] =
  {
    {M_PI/3,   M_PI/3,   0,        M_PI,     M_PI/3,   M_PI/3},
    {M_PI/3,   M_PI/3,   M_PI,     0,        M_PI/3,   M_PI/3},
    {0,        M_PI/3,   M_PI/2,   M_PI/2,   M_PI/3,   M_PI},
    {M_PI,     M_PI/3,   M_PI/2,   M_PI/2,   M_PI/3,   0}
  };

/** Array of offsets by movement direction and motor for sinusoidal position signal in rad.
	\details This array stores for each movement direction and motor the offset
	\f$ off \f$ of the sinusoidal position signal.
	The motor position signal is a sinusoidal signal of the type
	\f$ position(t) = A * cos(2 \pi f t +  \theta) + off \f$.
 */														
const uint16_t offset[CONF_NUMBER_OF_MOVEMENTS][CONF_NUMBER_OF_MOTORS] = 
  {
    {630,               556 + 50 - 70,     512/2+20,          512/2+20,          556 + 50 - 70,     630},
    {630,               556 + 50 - 70,     512/2+20,          512/2+20,          556 + 50 - 70,     630 },
    {(995-540)/2+540,   (512-480)/2+480,   (512-350)/2+350,   (512-350)/2+350,   (512-480)/2+480,   (995-540)/2+540},
    {(995-540)/2+540,   (512-480)/2+480,   (512-350)/2+350,   (512-350)/2+350,   (512-480)/2+480,   (995-540)/2+540}
  };

/** Callback function for receiving serial data.
	\details This callback function is used to react on external commands sent over the serial communication line. It is executed
	whenever a character has been received. The transfered character is read by the _serial_read()_ function and interpreted.
	A reception of a character is indicated by flashing the #LED_RXD LED.
	
	The following commands are currently implemented
    - 'p': Debug command to print current motor positions.
    - 'o': Debug command to print current sensor values.
    - 'w': Change movement direction to forward (set #global_release and #global_movement_type = #CONF_MOVEMENT_FORWARD).
    - 's': Change movement direction to backward (set #global_release and #global_movement_type = #CONF_MOVEMENT_BACKWARD).
    - 'a': Change movement direction to left (set #global_release and #global_movement_type = #CONF_MOVEMENT_LEFT).
    - 'd': Change movement direction to right (set #global_release and #global_movement_type = #CONF_MOVEMENT_RIGHT).
    - 'q': Stop or start movement (toggling #global_release).
    - 'z': Change between Zigbee and wired serial connection (toggling #global_use_zigbee).
    - 'r': Change between autonomous or remote-controlled mode (toggling #global_release_autonomous).
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
			// Stop robot
			global_release = 0;
			// Print positions of all motors for debugging
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
			// Stop robot
			global_release = 0;
			// Print values of all sensors for debugging
			uint16_t dist_front = sensor_read(CONF_SENSOR_FRONT, SENSOR_DISTANCE);
			uint16_t dist_left = sensor_read(CONF_SENSOR_LEFT, SENSOR_IR);
			uint16_t dist_right = sensor_read(CONF_SENSOR_RIGHT, SENSOR_IR);
			printf("Sensors: Front: %3u, Left: %3u, Right: %3u.\n", dist_front, dist_left, dist_right);
			break;
		}
		
		// Forward
		case 'w':
		{
			// Start robot and move forwards
			global_release = 1;
			global_movement_type = CONF_MOVEMENT_FORWARD;
			printf("Going forward.\n");
			break;
		}
		
		// Backward
		case 's':
		{
			// Start robot and move backwards
			global_release = 1;
			global_movement_type = CONF_MOVEMENT_BACKWARD;
			printf("Going backward.\n");
			break;
		}
		
		// Left
		case 'a':
		{
			// Start robot and move to the left side
			global_release = 1;
			global_movement_type = CONF_MOVEMENT_LEFT;
			printf("Going left.\n");
			break;
		}
		
		// Right
		case 'd':
		{
			// Start robot and move to the right side
			global_release = 1;
			global_movement_type = CONF_MOVEMENT_RIGHT;
			printf("Going right.\n");			
			break;
		}
	
		// Start or stop
		case 'q':
		{
			// Start or stop robot
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
				// Enable wired connection
				serial_set_wire();
				printf("Using wired connection.\n");
			}
			break;
		}
		
		// Enable/disable autonomous control
		case 'r':
		{
			// Enable or disable autonomous control mode
			global_release_autonomous ^= 1;
			if (global_release_autonomous)
				printf("Autonomous control mode activated.\n");
			else
				printf("Autonomous control mode deactivated.\n");
			break;
		}
	}
}

/** Callback function for pressing the start button.
	\details This callback function is called whenever the start button is pressed or released. It sets
	the global movement release #global_release in order to start or stop the robot movement.
 */
void btn_press_start(void)
{
	// Start robot in case button is pressed (and not released)
	if (BTN_START_PRESSED)
		global_release ^= 1;
}

/** Update of the motor position with a sinusoidal signal.
	\details \param[in]	time_in_ms		Current time \f$ t \f$.
	\param[in]	movement_type	Definition of the movement direction. 
	\details This functions updates the motor position depending on the current movement direction and time
	with a sinusoidal signal of the type \f[ position(t) = A * cos(2 \pi f t +  \theta) + off. \f]
	Depending on the movement direction different parameter sets for #amplitude \f$ A \f$, #frequency \f$ f \f$,
	#phase shift \f$ \theta \f$ and #offset \f$ off \f$ are used.
	In order to reduce motor bus traffic all motors are controlled at the same time.
 */
void update_motor_position(uint16_t time_in_ms, uint8_t movement_type) {
	if (movement_type < CONF_NUMBER_OF_MOVEMENTS)
	{
		uint16_t pos[CONF_NUMBER_OF_MOTORS];
		// Generate position signal for each motor
		for (int i=0; i<CONF_NUMBER_OF_MOTORS; i++)
		{
			float c = cos(2 * M_PI * frequency[movement_type][i] * (float)time_in_ms / 1000 + phase[movement_type][i]);
			pos[i] = amplitude[movement_type][i]*c + offset[movement_type][i];
		}
		// Send motor position signal to all motors at once
		motor_sync_move(CONF_NUMBER_OF_MOTORS, ids, pos, MOTOR_MOVE_NON_BLOCKING);		
	}
}

/** Output compare callback function for timer 0.
	This output compare callback function of timer 0 is called at 1 kHz and used to generate a precise global timer
	#global_elapsed_time in milliseconds units. Based on this timer the motor positions are accurately controlled
	in #CONF_MOTOR_UPDATE_POSITION_INTERVAL intervals by calling #update_motor_position.
 */
void timer0_compare_match(void)
{
	if (global_release)
	{
		// Increase timer
		global_elapsed_time++;
		LED_ON(LED_PLAY);
	}
	else
		LED_OFF(LED_PLAY);
	// Toggle live bit every second
	if (!(global_elapsed_time % 1000))
		LED_TOGGLE(LED_AUX);
}

/** Helper function to calculate simple moving average.
	\details \param[in,out]	value_buffer	Pointer to a data point array.
	\param[in]			buffer_size			Length of data points array \f$ n \f$.
	\param[in,out]		new_index			Pointer to an array place for the new data point to store.
	\param[in]			new_value			New data point \f$ p_M \f$ to store.
	\param[in]			old_moving_avg		Old moving average value \f$ SMA_{old} \f$.
	
	This helper function calculates the simple moving average \f$ SMA = SMA_{old} - \frac{p_{M-n}-p_M}{n} \f$. In order
	to increase the performance it uses the old average \f$ SMA_{old} \f$ instead of summing up all former data points.
	In this context the function is used to reduce the sensor noise induced by the movement of the vehicle.
	
	\returns The function returns the new simple moving average \f$ SMA \f$.	
 */
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

/**
   Autonomous movement decision maker
	\details 	\param[in]			movement_type			The active movement direction at the moment.
	\param[in,out]		dist_front_buffer		The average buffer for calculating average of front sensor values
	\param[in]			dist_front_avg			The front sensor average value
	\param[in]			dist_left_avg			The left sensor average value
	\param[in]			dist_right_avg			The right sensor average value
	\returns					The chosen direction by autonomous movement decider.
   This function makes decisions based on input values.
   
   A flow chart of the autonomous control:
   <img src="2-c-nonewheel-autonomous.png">
   
   It does:
	 - When robot going forward and close to wall: Go right
	 - When robot is going left or right and is away from wall: Go forward
	 - When robot is going left and wall is close on left side: Go right
	 - When robot is going right and wall is close on right side: Go left
	 
   In short terms it makes the robot goes forward as the robot's main purpose. If there is a wall in front, it will go right until the wall is gone.
   Should the robot go to left and right, and meet a wall, it will go the opposite direction.
  */
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

/** Main application logic and control loop of the Squid robot.
	This function contains the main application logic and the control loop of the robot. At first the used
	firmware functionalities are initialized. This includes motors, the serial connection, the timer 0, the
	I/O and the sensors. Then the motors are placed in the center position for starting and the non-ending
	control loop is executed.
	
	Here to begin with the global variables (e.g. #global_release,
	#global_movement_type and #global_elapsed_time) are copied in an atomic block to local
	variables in order to avoid race conditions. Then the sensor values are read and a simple moving average of
	the recent sensors values is formed (see #calc_simple_moving_avg). This is done to reduce the noise induced
	by the movement. In the next part it is checked whether the robot is actually allowed to move. In autonomous
	mode (Squid II) the sensors are now evaluated in order to generate the necessary movement direction
	(see #execute_autonomous_movement). In non-autonomous mode (Squid I) this procedure is skipped since the
	movement direction is given externally (see #serial_receive_data). In case the position is to be changed
	the robot moves into its center position as at the start. Then in the last step the motor positions are updated
	to form the movement (see #update_motor_position). At the end of each control loop cycle the current motor status
	is printed out to report any motor errors.
	
	A <a href="#main_logic">schematic activity diagram</a> of the function is provided before.
	
	\returns The return value is not used, since the main function never ends.
 */
int main() {	
	// Initialize motor
	dxl_initialize(0, 1);
	// Initialize serial connection and activate ZigBee
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
	sensor_init(CONF_SENSOR_FRONT, SENSOR_DISTANCE);
	sensor_init(CONF_SENSOR_LEFT, SENSOR_IR);
	sensor_init(CONF_SENSOR_RIGHT, SENSOR_IR);
	// Enable global interrupts
	sei();
	// Center motor position
	const uint16_t center_pos[CONF_NUMBER_OF_MOTORS] = {512, 512, 512, 512, 512, 512};
	motor_sync_move(CONF_NUMBER_OF_MOTORS, ids, center_pos, MOTOR_MOVE_BLOCKING);
	// Declare local variables
	uint8_t release = 0, release_autonomous = 0;
	uint32_t elapsed_time = 0, last_elapsed_time = 0;
	uint8_t movement_type = 0, last_movement_type = 0;
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
		// Copy global variables in an atomic blocks to avoid race conditions
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			release = global_release;
			elapsed_time = global_elapsed_time;
			movement_type = global_movement_type;
			release_autonomous = global_release_autonomous;
		}
		
		// Read sensor measurements and calculate simple moving average to remove noise induced by movements
		dist_front_avg = calc_simple_moving_avg(dist_front_buffer, CONF_SENSOR_FRONT_NUMBER_OF_SAMPLES,
			&dist_front_buffer_pointer, sensor_read(CONF_SENSOR_FRONT, SENSOR_DISTANCE), dist_front_avg);
		dist_left_avg = calc_simple_moving_avg(dist_left_buffer, CONF_SENSOR_LEFT_NUMBER_OF_SAMPLES,
			&dist_left_buffer_pointer, sensor_read(CONF_SENSOR_LEFT, SENSOR_DISTANCE), dist_left_avg);
		dist_right_avg = calc_simple_moving_avg(dist_right_buffer, CONF_SENSOR_RIGHT_NUMBER_OF_SAMPLES,
			&dist_right_buffer_pointer, sensor_read(CONF_SENSOR_RIGHT, SENSOR_DISTANCE), dist_right_avg);

		// Check for release to move
		if (release)
		{
			
			// Check if autonomous control is active and execute in case
			if (release_autonomous)
			{
				uint8_t new_movement_type = execute_autonomous_movement(movement_type, dist_front_buffer,
					dist_front_avg, dist_left_avg, dist_right_avg);
				// Check for change in movement and update global variable in case
				if (new_movement_type != movement_type)
				{
					movement_type = new_movement_type;
					// Update the global movement direction in an atomic block to avoid race conditions
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
