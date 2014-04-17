#ifndef _MOTION_SENSOR_H_
#define _MOTION_SENSOR_H_

#include <pthread.h>

#define YAW 0
#define PITCH 1
#define ROLL 2
#define DIM 3


struct s_ms {
	float ypr[3];
	float gypro[3];
	struct timespec t;
	pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
};

extern struct s_ms ms;

extern int ms_open();

extern int ms_update();

extern int ms_close();

#endif
