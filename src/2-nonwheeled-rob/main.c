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

#define NUM_ACTUATOR 3

int8_t turn=0;
		uint8_t ids[3]={4,7,8};
		uint16_t positions[3];//={512+40,512+70,512-70};


	int main() {
		int CommStatus;
				
		dxl_initialize(0,1);
		serial_initialize(57600);
		sei();
		int i=0;
		for(i=0;i<100;i++){
			positions[0]=512+40;
			positions[1]=512+70;
			positions[2]=512+turn-70;
			motor_sync_move(3,ids,positions);
			_delay_ms(200);
			positions[0]=512-40;
			positions[1]=512-70;
			positions[2]=512+turn+70;
			motor_sync_move(3,ids,positions);
			_delay_ms(200);
		}//closes for loop
			
		while(1) {
						
		}
		return 0;
}

