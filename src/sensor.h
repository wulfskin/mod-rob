/*! \file sensor.h
    \brief Interface to the sensors in the Robotis Bioloid kit.
	\author Walter Gambelunghe
	\copyright GNU Public License V3
	\date 2012

	\file sensor.h
	
	\details This file provides functions to initialize and read  the values of the various sensors in the kit.
	Each sensor needs to be initialized by calling  #sensor_init and then it can be read by using #sensor_read.
	For both functions the parameters are the port number the sensor is connected to and the type of the sensor,
	which should be one of the defined symbolic types. The sensor type is necessary, since IR sensors, unlike the others, 
	require the IR led to be turned on ond off upon reading.
	
	The following code shows how to use the functions:
		\par 1. Printing the values of two different sensors ~ every second
	\code
	#define SENSOR_IR_PORT 1	//IR sensor is connected to port 1
	#define SENSOR_TOUCH_PORT 2	//TOUCH sensor is connected to port 2
	
	uint8_t ir, touch;	//variables to store the read values
	
	sensor_init(SENSOR_IR_PORT,SENSOR_IR);	//initialize IR sensor
	sensor_init(SENSOR_TOUCH_PORT,SENSOR_TOUCH);	//initialize TOUCH sensor
	
	while(1) {
		
		ir = sensor_read(SENSOR_IR_PORT,SENSOR_IR);	//read the value of the IR sensor
		touch= = sensor_read(SENSOR_TOUCH_PORT,SENSOR_TOUCH);	//read the value of the TOUCH sensor
		
		printf("Value or IR sensor: %u, Value of TOUCH sensor: %u \n,ir,touch");	//print the values
		
		_delay_ms(1000);	//delay 1 second
	}
		
	\endcode
*/
#ifndef __SENSOR_H 
#define __SENSOR_H

/// Symbolic definition of an IR sensor
#define SENSOR_IR 1
/// Symbolic definition of a TOUCH sensor
#define SENSOR_TOUCH 0
/// Symbolic definition of a DISTANCE sensor
#define SENSOR_DISTANCE 0

/** Function to initialize a sensor
 * \param[in]	port		port the sensor is connected to
 * \param[in]	type		type of the sensor, should be one of SENSOR_IR, SENSOR_TOUCH, SENSOR_DISTANCE
 */
void sensor_init(uint8_t port,uint8_t type);

/** Function to read the value of a sensor
 * \param[in]	port		port the sensor is connected to
 * \param[in]	type		type of the sensor, should be one of SENSOR_IR, SENSOR_TOUCH, SENSOR_DISTANCE
 * \returns Returns the current value of the specified sensor
 */
uint8_t sensor_read(uint8_t port,uint8_t type);

#endif //__SENSOR_H