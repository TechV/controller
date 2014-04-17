#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>

#include "motionsensor.h"
#include "receiver.h"

#define delay_ms(a) usleep(a*1000)

pthread_t p_ms;
struct timespec t_ms;
const int ms_fallback_threshold = 10;  //deviation from the rate is ms
int ms_err = 0;

pthread_t p_rec;
struct timespec t_rec;
const int rec_fallback_threshold = 10;  //deviation from the rate is ms
int rec_err = 0;

const int rate = 100; //every 10ms


volatile struct timespec c_t;

void *sensor_update( void * ptr ) {
	ms_err = ms_update();
	clock_gettime(CLOCK_REALTIME, &t_ms);
	if (t_ms.tv_nsec - ms.t.tv_nsec > (ms_fallback_threshold*1000) ) {
		ms_err = 1;
		pthread_exit(NULL);
		return NULL;
	}
	delay_ms(1000 / (rate*2)); //twice the rate
	return NULL;
}

void *receiver_update( void *ptr ) {
	rec_err = rec_update();
	clock_gettime(CLOCK_REALTIME, &t_rec);
	if (t_rec.tv_nsec - rec.t.tv_nsec > (rec_fallback_threshold*1000) ) {
		rec_err = 1;
		pthread_exit(NULL);
		return NULL;
	}
	delay_ms(1000 / (rate*2)); //twice the rate
	return NULL;
}

int main() {
	int err;
	/* init and create MotionSensor thread */
	if (ms_open()<0) return -1;
	err = pthread_create(&p_ms, NULL, sensor_update, NULL);
        if (err != 0)
            printf("\ncan't create sensor thread :[%s]", strerror(err));	

	/* init and create receiver thread */
	if (rec_open()<0) return -1;
	err = pthread_create(&p_rec, NULL, receiver_update, NULL);
        if (err != 0)
            printf("\ncan't create receiver thread :[%s]", strerror(err));	

	/* main loop */
	do{
		printf("err = %i%i\tsy = %2.1f\tsp = %2.1f\tsr = %2.1f\try = %2.1f\trp = %2.1f\trr = %2.1f\n",ms_err,rec_err,
			ms.ypr[YAW],ms.ypr[PITCH],ms.ypr[ROLL],
			rec.ypr[YAW],rec.ypr[PITCH],rec.ypr[ROLL]);
		delay_ms(1000 / rate);
	}while(1);

	return 0;
}
