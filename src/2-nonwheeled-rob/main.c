#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "../macro.h"
#include "../sensor.h"
#include "../motor.h"
#include <dynamixel.h>
#include "../serial.h"
#include "../error.h"
#include "../serialzigbee.h"
#include "../io.h"

#define CENTER 512
#define TWIST_AMPLITUDE 20
#define TWIST_OFFSET 0
#define FORWARD_STEP 150
#define MAX_TURN	80
#define TURN_STEP

volatile int16_t global_turn=0;
volatile uint8_t global_release = 0;
volatile uint8_t global_use_zigbee = 0;

void serial_on_receive_data(void)
{
	LED_TOGGLE(LED_RXD);
	unsigned char data;
	serial_read(&data, 1);
	switch (data)
	{
		// Turn left
		case 'd':
		{
			global_turn = (global_turn >= MAX_TURN) ? MAX_TURN : global_turn + 20;
			printf("turn=%d\n", global_turn);
			break;
		}
		
		// Turn right
		case 'a':
		{
			global_turn = (global_turn <= -MAX_TURN) ? -MAX_TURN : global_turn - 20;
			printf("turn=%d\n", global_turn);
			break;
		}
		
		// Center position
		case 'w':
		{
			global_turn = 0;
			printf("turn=%d\n", global_turn);
			break;
		}
		
		// Stop
		case 's':
		{
			// Toggle release
			global_release ^= 1;
			break;
		}
		
		// Enable zigbee
		case 'z':
		{
			global_use_zigbee ^= 1;
			if (global_use_zigbee)
			{
				// Enable zigbee connection
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

void btn_on_press_start(void)
{
	// Toggle release
	if (BTN_START_PRESSED)
		global_release ^= 1;
}

//uint8_t ids[3]={4,7,8};
//uint8_t ids[2]={7,8};
//uint16_t positions[3];//={512+40,512+70,512-70};
//uint16_t positions[2];//={512+40,512+70,512-70};

int main() {
	// Initialize motors
	dxl_initialize(0,1);
	// Initialize serial connection
	serial_initialize(57600);
	serial_set_rx_callback(&serial_on_receive_data);
	// Enable interrupts
	sei();
	// Enable I/O
	io_init();
	io_set_interrupt(BTN_START, &btn_on_press_start);
	printf("start\n");
	// Move to start position
	motor_move(4, CENTER, MOTOR_MOVE_BLOCKING);
	motor_move(7, CENTER, MOTOR_MOVE_BLOCKING);		
	// Infinity loop
	while(1) {
		// Read global data
		int16_t turn = 0;
		uint8_t release = 0;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			turn = global_turn;
			release = global_release;
		}
		if (release)
		{
			// Turn forward 1
			motor_move(4, CENTER + TWIST_OFFSET + TWIST_AMPLITUDE, MOTOR_MOVE_BLOCKING);
	
			// Turn middle part 1
			motor_move(7, CENTER + turn + FORWARD_STEP, MOTOR_MOVE_BLOCKING);
		
			// Turn forward 2
			motor_move(4, CENTER + TWIST_OFFSET - TWIST_AMPLITUDE, MOTOR_MOVE_BLOCKING);
			
			// Turn middle part 2
			motor_move(7, CENTER + turn - FORWARD_STEP, MOTOR_MOVE_BLOCKING);
			
			// Print error code of motor
			PrintErrorCode();
			
		}					
	}
	return 0;
}