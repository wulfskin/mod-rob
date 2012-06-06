#ifndef __MOTOR_H
#define __MOTOR_H

#define MOTOR_WHEEL 0
#define MOTOR_JOINT 1023

#include "dynamixel.h"
#include "motor_control_table.h"


enum {CW,CCW};

void motor_move(char,int);

void motor_set_mode(char,int);

int motor_get_mode(char);

void motor_set_speed(char, int);

int motor_get_speed(char);

void motor_set_position(char, int);

int motor_get_position(char);
#endif //__MOTOR_H


