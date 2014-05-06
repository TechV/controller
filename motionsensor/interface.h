#ifndef _MOTION_SENSOR_INT_H_
#define _MOTION_SENSOR_INT_H_

struct s_ms {
	float ypr[3];
	float gyro[3];
	float c[3]; //compass
};

extern struct s_ms ms;

extern int ms_open();

extern int ms_update();

extern int ms_close();

#endif
