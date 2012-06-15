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


#define pi 3.14

#define NUMBER_OF_MOTORS 6


		//motor id
        uint8_t ids[NUMBER_OF_MOTORS]={1,2,3,5,6,8};
		
		//Time for update
		//uint16_t time=100000;
		
		//Running or not
		char active = 0x00;
		
		//In zigbee mode
		char zigbee = 0x00;
		
		//Simple harmonic oscillator
	    uint16_t amplitude[NUMBER_OF_MOTORS]={300,300,300,600,600,300};//={512+40,512+70,512-70};
		// Frequency in Hz
		uint16_t frequency[NUMBER_OF_MOTORS]={1,1,1,1,1,1};//={512+40,512+70,512-70};
		float angle[NUMBER_OF_MOTORS]={0,0,90*pi/180,0,0,90*pi/180};//={512+40,512+70,512-70};
		uint16_t offset[NUMBER_OF_MOTORS]={530,530,200,200,200,200};//={512+40,512+70,512-70};
			

void something(void){
	//LED_TOGGLE(LED_MANAGE);
	unsigned char data;
	serial_read(&data, 1);
	//printf("%c\n", data);
	
	if(data=='p'){
	    printf("\nPositions of motors:\n");
		for (int i=0;i<6;i++) {
		  uint16_t v = motor_get_position(ids[i]);
		  printf("Motor %d position is %u\n", ids[i], v);
		}		  
	}
	else if(data=='s'){
		active = 0x01;
	}    
	else if(data=='z'){
		if (zigbee == 0x00) {
			zigbee = 0x01;
			serial_set_zigbee();
		}		
		else {
			zigbee = 0x00;
			serial_set_wire();
		}		
	}
}



void update_motor_position(uint16_t time_in_ms) {
	if (active == 0x01) {
		for (int i=0; i<NUMBER_OF_MOTORS; i++)
		{
			float c = cos(2 * pi * frequency[i] * (float)time_in_ms / 1000 + angle[i]);
			uint16_t pos = amplitude[i]*c + offset[i];
	       motor_set_position(ids[i], pos, MOTOR_MOVE_NON_BLOCKING);
		}		   
	}
}

volatile uint32_t global_elapsed_time = 0; // elapsed time in ms

void timer0_callback(void)
{
	global_elapsed_time++;
	if (!(global_elapsed_time % 20))
	{
		update_motor_position((uint16_t)global_elapsed_time);
	}		
}

	int main() {
		int CommStatus;
		uint8_t timer = 0;
		dxl_initialize(0,1);
		serial_initialize(57600);
		serial_set_rx_callback(&something);
		timer_set_interrupt(timer, TIT_OUTPUT_COMPARE_MATCH_A, &timer0_callback);
		timer_set_value(timer, TVT_OUTPUT_COMPARE_A, F_CPU / 64 / 1000 - 1);
		timer_init(timer, TOM_CLEAR_ON_COMPARE, TPS_DIV_64, 0);
		//serial_set_zigbee();
		sei();
		io_init();
		uint32_t g=0;
		while(1) {
			
		}// close infinite loop
		return 0;
}