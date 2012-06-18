#ifndef __MOTOR_H
#define __MOTOR_H

#define MOTOR_WHEEL_MODE 0
#define MOTOR_JOINT_MODE 1023

#define MOTOR_BROADCAST_ID 254

#define MOTOR_CW 1
#define MOTOR_CCW 0

#define MOTOR_MOVE_BLOCKING			1
#define MOTOR_MOVE_NON_BLOCKING		0

#include <avr/io.h>
#include <dynamixel.h>
#include "motor_control_table.h"

void motor_move(char id, uint16_t motor_position, char blocking);

void motor_set_mode(char,int);

int motor_get_mode(char);

void motor_set_speed(char, int);

int motor_get_speed(char);

void motor_wait_finish(char id, uint16_t goal_position);

void motor_set_position(char id, uint16_t motor_position, char blocking);

uint16_t motor_get_position(char);

void motor_sync_move(const uint8_t size, const uint8_t * id, const uint16_t * position, const char blocking);

void PrintCommStatus(int CommStatus);

void PrintErrorCode();

#endif //__MOTOR_H


