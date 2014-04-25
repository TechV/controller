#ifndef MPU6050_H
#define MPU6050_H

#include <pthread.h>

struct s_ms ms = {
        .ypr = {0.0f,0.0f,0.0f},
        .gypro = {0.0f,0.0f,0.0f},
        .ts = {0},
	.mutex = PTHREAD_MUTEX_INITIALIZER
};

int ms_open();
int ms_update();
int ms_close();


uint8_t GetGravity(VectorFloat *v, Quaternion *q);
uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity); 
uint8_t GetGyro(int32_t *data, const uint8_t* packet);
#endif
