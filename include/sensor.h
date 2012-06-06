#ifndef __SENSOR_H 
#define __SENSOR_H

#define IR 1
#define TOUCH 0
#define DISTANCE 0

void sensor_init(char,char);
int sensor_read(char,char);

#endif //__SENSOR_H