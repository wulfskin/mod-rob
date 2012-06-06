/*
 * Navigationtest.c
 *
 * Created: 05/06/2012 13:29:59
 *  Author: Walter Gambelunghe
 */ 
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "macro.h"
#include "sensor.h"
#include "motor.h"
#include "dynamixel.h"
#include "serial.h"
#include "error.h"

#define SENSOR_LEFT 3
#define SENSOR_RIGHT 4

#define MOTOR_LEFT 11
#define MOTOR_RIGHT 12

	int main() {
		
		int ir_left, ir_right,speed_left,speed_right,touch;
				
		dxl_initialize(0,1);
		serial_initialize(57600);
		sei();
		sensor_init(1,TOUCH);
		sensor_init(SENSOR_LEFT,IR);
		sensor_init(SENSOR_RIGHT,IR);
		motor_set_mode(254,MOTOR_WHEEL_MODE);
		
		while(1) {
			ir_left=sensor_read(SENSOR_LEFT,IR);
			ir_right=sensor_read(SENSOR_RIGHT,IR);
			touch=sensor_read(1,TOUCH);
			if(touch){
				motor_set_speed(MOTOR_LEFT,0);
				motor_set_speed(MOTOR_RIGHT,0);
			}
			else {
			speed_left=1023-(2*ir_left);
			speed_right=2047-(4*ir_right);
			motor_set_speed(MOTOR_LEFT,speed_left);
			motor_set_speed(MOTOR_RIGHT,speed_right);
			}			
			printf("left= %d, right=%d \n",ir_left,ir_right);
		}
		return 0;
}