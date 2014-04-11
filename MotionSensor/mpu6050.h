#ifndef MPU6050_H
#define MPU6050_H

#define YAW 0
#define PITCH 1
#define ROLL 2
#define DIM 3

int ms_open();
int ms_update();
int ms_close();

uint8_t GetGravity(VectorFloat *v, Quaternion *q);
uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity); 
uint8_t GetGyro(int32_t *data, const uint8_t* packet);
#endif
