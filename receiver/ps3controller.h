#ifndef PS3CONTROLLER_H
#define PS3CONTROLLER_H

#include <pthread.h>
#include "interface.h"

struct s_rec rec = {
	.yprt = {0.0f,0.0f,0.0f,0.0f},
        .ts = {0},
        .mutex = PTHREAD_MUTEX_INITIALIZER
};

int rec_open();
int rec_update();
int rec_close();

#endif

