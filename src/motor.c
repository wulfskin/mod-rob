#include "motor.h"
#include "error.h"
#include "macro.h"
#include <stdio.h>
#include <util/delay.h>


int read_data(char,char);
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

void motor_set_speed_dir(char id, uint8_t percentage, char wise){
	uint16_t v=0;
	v = (uint16_t) percentage*1023ul/100ul;
	if (wise)
		SET(v,10); //bit 10 is the direction bit 0 ccw, 1 cw
	dxl_write_word(id, MOVING_SPEED_L, v);
}

int motor_get_speed(char id) {
	return dxl_read_word(id, PRESENT_SPEED_L);
}

void motor_set_position(char id, int motor_position){
	dxl_write_word(id, GOAL_POSITION_L, motor_position);
}

int motor_get_position(char id) {
	return read_data(id, PRESENT_POSITION_L);
}

void motor_spin(char id, char wise){
	int goal=wise*1023;
	motor_set_position(id,goal);
	motor_set_mode(id,MOTOR_JOINT_MODE);
	while(!(motor_get_position(id)==goal));
	motor_set_mode(id,MOTOR_WHEEL_MODE);
	motor_set_speed(id,1023);
	motor_set_mode(id,MOTOR_JOINT_MODE);
}

int read_data(char id, char which) {
	int value=value=dxl_read_word(id,which);
	int result=dxl_get_result();
	if(result==COMM_RXSUCCESS)
		return value;
	switch(result){
		case COMM_TXSUCCESS:{
			printf("Read status: COMM_TXSUCCESS ");
			break;
		}
		/*case COMM_RXSUCCESS:{
			printf("Read status:  COMM_RXSUCCESS ");
			break;
		}*/
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
	return 5000;
}
