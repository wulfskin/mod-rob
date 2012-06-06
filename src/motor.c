#include "motor.h"
#include "error.h"
#include <stdio.h>

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
	//while(dxl_get_result()!=COMM_RXSUCCESS);
		return value;
	
}
	