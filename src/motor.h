#ifndef __MOTOR_H
#define __MOTOR_H

#define MOTOR_WHEEL_MODE 0
#define MOTOR_JOINT_MODE 1023

#define MOTOR_BROADCAST_ID 254

#include "dynamixel.h"
#include "motor_control_table.h"


enum {CW,CCW};

void motor_move(char,int);

void motor_set_mode(char,int);

int motor_get_mode(char);

void motor_set_speed(char, int);

int read_data(char,char);

int motor_get_speed(char);

void motor_set_position(char, int);

int motor_get_position(char);
#endif //__MOTOR_H


