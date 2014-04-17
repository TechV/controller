#ifndef _RECEIVER_H_
#define _RECEIVER_H_

#include <pthread.h>

#define YAW 0
#define PITCH 1
#define ROLL 2
#define THRUST 3
#define DIM 4


struct s_rec {
        float yprt[4];
        struct timespec t;
        pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
};

extern struct s_rec rec;

extern int rec_open();

extern int rec_update();

extern int rec_close();

#endif
