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
#include "serialzigbee.h"
#include "io.h"

#define SENSOR_FRONT 1
#define SENSOR_FRONTLEFT 3
#define SENSOR_FRONTRIGHT 4
#define SENSOR_BACKLEFT 2
#define SENSOR_BACKRIGHT 5
#define MOTOR_LEFT 11
#define MOTOR_RIGHT 12

	int main() {
		
		int ir_frontleft, ir_frontright,ir_backleft,ir_backright,front;
		//int speed_left,speed_right,	dir_left,dir_right;
		char state = 0;
		dxl_initialize(0,1);
		serial_initialize(57600);
		sei();
		sensor_init(SENSOR_FRONT,DISTANCE);
		sensor_init(SENSOR_FRONTLEFT,IR);
		sensor_init(SENSOR_FRONTRIGHT,IR);
		sensor_init(SENSOR_BACKLEFT,IR);
		sensor_init(SENSOR_BACKRIGHT,IR);
		led_init();
		motor_set_mode(254,MOTOR_WHEEL_MODE);
		serial_set_zigbee();
		dxl_write_word(254,14,256);
		while(1) {
			front=sensor_read(SENSOR_FRONT,DISTANCE);
			ir_frontleft=sensor_read(SENSOR_FRONTLEFT,IR);
			ir_frontright=sensor_read(SENSOR_FRONTRIGHT,IR);
			ir_backleft=sensor_read(SENSOR_BACKLEFT,IR);
			ir_backright=sensor_read(SENSOR_BACKRIGHT,IR);
			
/*
             if (!state) {
					LED_OFF(LED_PLAY);
					if (ir_frontleft < 15)
						 ir_frontleft = 0;
					if (ir_frontright < 15)
						ir_frontright = 0;
					if(ir_backleft >100 || ir_backleft>100)
						state=1;
					
					speed_left= ir_frontright < 50 ? (1023-(16*ir_frontright)) : (1023+(2*ir_frontright));
					speed_right=ir_frontleft  < 50 ? 2047-(16*ir_frontleft) : (2*ir_frontleft);
			 }
			 else if(state){
				 LED_ON(LED_PLAY);
				 speed_left=0;
				 speed_right=0;
				 
			 }			
			
		*/
/*
			if (ir_backright > ir_frontright)
				ir_frontright = 0;
			else
				ir_frontright -= ir_backright;
				
			if (ir_backright > ir_frontright)
				ir_frontright = 0;
			else
				ir_frontright -= ir_backleft;
			
			if (ir_frontleft < 15)
				ir_frontleft = 0;
			if (ir_frontright < 15)
				ir_frontright = 0;
			if(ir_frontright<50){
				speed_left=100-(ir_frontright)
				dir_left=MOTOR_CCW;
			}
			else{
				speed_left=ir_frontright/8;
				dir_left=
			}
			speed_left= ir_frontright < 50 ? (1023-(16*ir_frontright)) : (1023+(2*ir_frontright));
			speed_right=ir_frontleft  < 50 ? 2047-(16*ir_frontleft) : (2*ir_frontleft);
					
			speed_left=*/
			//speed_left=ir_backleft>speed_left? (speed_left-ir_backleft): 0;
			//speed_rightt=ir_backright>speed_right? (speed_left-ir_backleft): 0;
			//speed_right-=ir_backright;
			
			int target_left = (ir_frontleft < 50) ? (1023ul - ir_frontleft) : (-ir_frontleft);
			if (ir_backleft > ir_frontleft)
				target_left = 0;
			else
				target_left -= ir_backleft;
				
			int target_right = ir_frontright;
			if (ir_backright > ir_frontright)
				target_right = 0;
			else
				target_right -= ir_backright;
			
			uint8_t speed_right = (uint8_t) (target_left * 100ul / 1023ul);
			uint8_t speed_left = (uint8_t) (target_right * 100ul / 1023ul);
			
			
			motor_set_speed_dir(MOTOR_LEFT,speed_left,MOTOR_CCW);
			motor_set_speed_dir(MOTOR_RIGHT,speed_right,MOTOR_CW);
			//}			
			//printf("distance= %3d, left= %3d, right=%3d, speed_left=%3d, speed_right=%3d \n",touch,ir_left,ir_right,speed_left,speed_right);
			printf("front= %3d, frontleft= %3d, frontright=%3d, backleft= %3d, backright=%3d,\n",front,ir_frontleft,ir_frontright,ir_backleft,ir_backright);
		}
		return 0;
}

			/*if(touch){
				motor_set_speed(MOTOR_LEFT,0);
				motor_set_speed(MOTOR_RIGHT,0);
			}
			else {*/

	//motor_set_speed(MOTOR_LEFT,speed_left<0 ? (2048-speed_left): (speed_left) );
	//speed_left=1023;
	//speed_right=2047;
	//int diff= (ir_left>ir_right) ? (ir_left-ir_right) : (ir_right-ir_left);
	//if (ir_left<ir_right)
	//{
		//	speed_right-=2*(ir_right-ir_left);
	//}
	//else
	//	{
		//	speed_left-=2*(ir_left-ir_right);
	//	}
	