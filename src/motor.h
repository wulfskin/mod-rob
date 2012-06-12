#ifndef __MOTOR_H
#define __MOTOR_H

#define MOTOR_WHEEL_MODE 0
#define MOTOR_JOINT_MODE 1023

#define MOTOR_BROADCAST_ID 254

#define MOTOR_CW 1
#define MOTOR_CCW 0

#include <dynamixel.h>
#include <stdint.h>
#include "motor_control_table.h"

void motor_move(char,int);

void motor_set_mode(char,int);

int motor_get_mode(char);

void motor_set_speed(char, int);

void motor_set_speed_dir(char, uint16_t, char);

void motor_spin(char,char);

int motor_get_speed(char);

void motor_set_position(char, int);

int motor_get_position(char);
#endif //__MOTOR_H


