#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>

#include <stdlib.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

#include "motionsensor/interface.h"
#include "receiver/interface.h"
#include "bmpsensor/interface.h"
#include "speedcontroller/interface.h"

#define delay_ms(a) usleep(a*1000)

pthread_t p_ms;
struct timespec t_ms;
const int ms_fallback_threshold = 50;  //deviation from the rate is ms
int ms_err = 0;

pthread_t p_rec;
struct timespec t_rec;
const int rec_fallback_threshold = 50;  //deviation from the rate is ms
int rec_err = 0;

pthread_t p_bs;
struct timespec t_bs;
const int bs_fallback_threshold = 50;  //deviation from the rate is ms
int bs_err = 0;

const int rate = 100; //every 10ms


volatile struct timespec c_t;

void *sensor_update( void * ptr ) {
	ms_err = ms_update();
	clock_gettime(CLOCK_REALTIME, &t_ms);
	if (t_ms.tv_nsec - ms.ts.tv_nsec > (ms_fallback_threshold*1000) ) {
		ms_err = 1;
		pthread_exit(NULL);
		return NULL;
	}
	delay_ms(1000 / (rate*2)); //twice the rate
	return NULL;
}

void *receiver_update( void *ptr ) {
	while(1) {
	rec_err = rec_update();
	clock_gettime(CLOCK_REALTIME, &t_rec);
	if (t_rec.tv_nsec - rec.ts.tv_nsec > (rec_fallback_threshold*1000) ) {
		rec_err = t_rec.tv_nsec - rec.ts.tv_nsec;
		pthread_exit(NULL);
		return NULL;
	}
	delay_ms(1000 / (rate*2)); //twice the rate
	}
	return NULL;
}

void *baro_update( void *ptr ) {
	while(1) {
	bs_err = bs_update();
	clock_gettime(CLOCK_REALTIME, &t_bs);
	if (t_bs.tv_nsec - bs.ts.tv_nsec > (bs_fallback_threshold*1000) ) {
		bs_err = t_bs.tv_nsec-bs.ts.tv_nsec;
		pthread_exit(NULL);
		return NULL;
	}
	}
	return NULL;
}

int main() {
	int err;

        struct sched_param param;

        /* Declare ourself as a real time task */

        param.sched_priority = 49;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                perror("sched_setscheduler failed");
                exit(-1);
        }

        /* Lock memory */

        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

	/* init and create MotionSensor thread */

//	if (ms_open()<0) return -1;
//	err = pthread_create(&p_ms, NULL, sensor_update, NULL);
//        if (err != 0) {
//		return -1;
//            printf("\ncan't create sensor thread :[%s]", strerror(err));	
//	}
	/* init and create receiver thread */
	if (rec_open()<0) return -1;
	err = pthread_create(&p_rec, NULL, receiver_update, NULL);
        if (err != 0) {
            printf("\ncan't create receiver thread :[%s]", strerror(err));	
	    return -1;
	}

	/* init and create baro thread */
	if (bs_open()<0) return -1;
//	err = pthread_create(&p_bs, NULL, baro_update, NULL);
//        if (err != 0) {
//            printf("\ncan't create BMP thread :[%s]", strerror(err));	
//	    return -1;
//	}
	/* main loop */
	do{
/*
		printf("err = %i%i\tsy = %2.1f\tsp = %2.1f\tsr = %2.1f\try = %2.1f\trp = %2.1f\trr = %2.1f\n",ms_err,rec_err,
			ms.ypr[0],ms.ypr[1],ms.ypr[2],
			rec.yprt[0],rec.yprt[1],rec.yprt[2]);
		delay_ms(1000 / rate);
*/
		printf("err = %i %i\tt = %4.2f\tb = %6.5f\talt = %4.4f\try = %2.1f\trp = %2.1f\trr = %2.1f\n",bs_err,rec_err,
			bs.t,bs.p,bs.alt,
			rec.yprt[0],rec.yprt[1],rec.yprt[2]);
		delay_ms(10000 / rate);
	}while(1);

	return 0;
}
