#ifndef _MOTION_SENSOR_INT_H_
#define _MOTION_SENSOR_INT_H_

#include <pthread.h>

struct s_ms {
	float ypr[3];
	float gypro[3];
	struct timespec ts;
	pthread_mutex_t mutex; 
};

extern struct s_ms ms;

extern int ms_open();

extern int ms_update();

extern int ms_close();

#endif
