#ifndef __MOTOR_H
#define __MOTOR_H

#define MOTOR_WHEEL_MODE 0
#define MOTOR_JOINT_MODE 1023

#define MOTOR_BROADCAST_ID 254

#define MOTOR_CW 1
#define MOTOR_CCW 0

#include <avr/io.h>
#include <dynamixel.h>
#include "motor_control_table.h"

void motor_move(char id, uint16_t motor_position, char blocking);

void motor_set_mode(char,int);

int motor_get_mode(char);

void motor_set_speed(char, int);

void motor_spin(char,char);

int motor_get_speed(char);

void motor_wait_finish(char id, uint16_t goal_position);

void motor_set_position(char id, uint16_t motor_position, char blocking);

uint16_t motor_get_position(char);

void motor_sync_move(uint8_t size, uint8_t * id, uint16_t * position, char blocking);

void PrintCommStatus(int CommStatus);

void PrintErrorCode();

#endif //__MOTOR_H


