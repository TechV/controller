#ifndef _BMP_INT_H_
#define _BMP_INT_H_

#include <pthread.h>

struct s_bs {
	float t;
	float p;
	float p0;
	float alt;
        struct timespec ts;
        pthread_mutex_t mutex;
};

extern struct s_bs bs;

extern int bs_open();
extern int bs_update();
extern int bs_close();

#endif 
