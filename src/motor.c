#include "motor.h"
#include "error.h"
#include <stdio.h>
#include <util/delay.h>
void motor_move (char id, int position) {
	dxl_write_word( id, GOAL_POSITION_L, position);	
}

void motor_set_mode(char id, int mode) {
	dxl_write_word( id, 8, mode);
}

int motor_get_mode(char id){
	return dxl_read_word(id,8);
}

void motor_set_speed(char id, int motor_speed){
	dxl_write_word(id, MOVING_SPEED_L, motor_speed);
}

int motor_get_speed(char id) {
	return dxl_read_word(id, PRESENT_SPEED_L);
}

void motor_set_position(char id, int motor_position){
	dxl_write_word(id, GOAL_POSITION_L, motor_position);
}

int motor_get_position(char id) {
	int value=0;
	value=dxl_read_word(id, PRESENT_POSITION_L);
		return value;	
}

int read_data(char id, char which) {
	
	int value=value=dxl_read_word(id,which);
	//_delay_us(500);
	switch(dxl_get_result()){
		case COMM_TXSUCCESS:{
			printf("Read status: COMM_TXSUCCESS ");
			break;
		}
		case COMM_RXSUCCESS:{
			printf("Read status:  COMM_RXSUCCESS ");
			break;
		}
		case COMM_TXFAIL:{
			printf("Read status:  COMM_TXFAIL ");
			break;
		}
		case COMM_RXFAIL:{
			printf("Read status: COMM_RXFAIL ");
			break;
		}
		case COMM_TXERROR:{
					printf("Read status: COMM_TXERROR ");
			break;
		}
		case COMM_RXWAITING:{
		printf("Read status: COMM_RXWAITING ");
			break;
		}
		case COMM_RXTIMEOUT:{
			printf("Read status:COMM_RXTIMEOUT ");
			break;
		}
		case COMM_RXCORRUPT:{
			printf("Read status:COMM_RXCORRUPT ");
			break;
		}
	}		
	return value;
}
