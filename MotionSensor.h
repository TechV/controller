#ifndef _MOTION_SENSOR_H_
#define _MOTION_SENSOR_H_

extern float ypr[3];
extern float gypro[3];

extern int ms_open();

extern int ms_update();

extern int ms_close();

#endif
