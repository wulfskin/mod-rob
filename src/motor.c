#include "motor.h"
#include "error.h"
#include "macro.h"
#include <stdio.h>
#include <util/delay.h>

/// Maximal allowed deviation from goal position.
#define MOTOR_MAX_DEVIATION		10
/// Number of cycles allowed without robot moving.
#define MOTOR_MAX_CYCLES_WITHOUT_MOVING		20

/// Delay in ms between motor commands to avoid flooding of serial port.
#define MOTOR_COMMAND_DELAY		5

// Maximum time in ms to wait for a motor to finish its position
#define MOTOR_MAX_TIMEOUT		2000ul

int read_data(char,char);

void motor_move(char id, uint16_t motor_position, char blocking) {
	motor_set_position(id, motor_position, blocking);
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

void motor_wait_finish(char id, uint16_t goal_position)
{
	uint16_t present_pos = 0, diff = MOTOR_MAX_DEVIATION + 1;
	uint8_t cycles_not_moving = 0;
	uint16_t timeout = 0;
	while (diff > MOTOR_MAX_DEVIATION && cycles_not_moving < MOTOR_MAX_CYCLES_WITHOUT_MOVING && timeout < MOTOR_MAX_TIMEOUT)
	{
		// Read current position
		present_pos = dxl_read_word(id, PRESENT_POSITION_L);
		// Wait to not overload serial motor bus
		_delay_ms(MOTOR_COMMAND_DELAY);
		timeout += MOTOR_COMMAND_DELAY;
		if (dxl_get_result() == COMM_RXSUCCESS)
		{
			if (present_pos > goal_position)
				diff = present_pos - goal_position;
			else
				diff = goal_position - present_pos;
			// Check if motor is not moving for a certain time
			if (diff <= MOTOR_MAX_DEVIATION)
				cycles_not_moving++;
			else
				cycles_not_moving = 0;
		}
	} 
}

void motor_set_position(char id, uint16_t motor_position, char blocking) {
	dxl_write_word(id, GOAL_POSITION_L, motor_position);
	int CommStatus = dxl_get_result();
	if (CommStatus = COMM_RXSUCCESS)
	{
		if (blocking == MOTOR_MOVE_BLOCKING)
			motor_wait_finish(id, motor_position);
	}
	else
		PrintCommStatus(CommStatus);
}

uint16_t motor_get_position(char id) {
	return (uint16_t)read_data(id, PRESENT_POSITION_L);
}

void motor_sync_move(uint8_t size, uint8_t * id, uint16_t * position, char blocking) {
	int i, CommStatus;
	dxl_set_txpacket_id(MOTOR_BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, GOAL_POSITION_L); //memory area to write
	dxl_set_txpacket_parameter(1, 2); //length of the data
	
	for(i=0;i<size;i++){
		//dxl_set_txpacket_parameter(2+(3*i), *(id+i)); //id of the first
		//dxl_set_txpacket_parameter(2+(3*i)+1, dxl_get_lowbyte(*(position+i)));//low byte for first data
		//dxl_set_txpacket_parameter(2+(3*i)+2, dxl_get_highbyte(*(position+i)));//high byte for first data
		dxl_set_txpacket_parameter(2+(3*i), id[i]); //id of the first
		dxl_set_txpacket_parameter(2+(3*i)+1, dxl_get_lowbyte(position[i]));//low byte for first data
		dxl_set_txpacket_parameter(2+(3*i)+2, dxl_get_highbyte(position[i]));//high byte for first data
	}
	
	dxl_set_txpacket_length((2+1)*size+4);
	dxl_txrx_packet();
	CommStatus = dxl_get_result();
	if( CommStatus = COMM_RXSUCCESS ){
		PrintErrorCode();
		if (blocking == MOTOR_MOVE_BLOCKING)
		{
			// Wait for finish of all motors
			for (i = 0; i < size; i++)
				motor_wait_finish(id[i], position[i]);
		}			
	}		
	else
		PrintCommStatus(CommStatus);
}

void motor_spin(char id, char wise){
	int goal=wise*1023;
	motor_set_mode(id,MOTOR_JOINT_MODE);
	motor_set_position(id, goal, 1);
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


// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
		case COMM_TXFAIL:
			printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

		case COMM_TXERROR:
			printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

		case COMM_RXFAIL:
			printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

		case COMM_RXWAITING:
			printf("COMM_RXWAITING: Now receiving status packet!\n");
		break;

		case COMM_RXTIMEOUT:
			printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

		case COMM_RXCORRUPT:
			printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

		default:
			printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if (dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}
