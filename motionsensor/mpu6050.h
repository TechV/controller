#ifndef MPU6050_H
#define MPU6050_H

struct s_ms ms = {
        .ypr = {0.0f,0.0f,0.0f},
        .gyro = {0.0f,0.0f,0.0f},
        .c = {0.0f,0.0f,0.0f},
};

int ms_open();
int ms_update();
int ms_close();


uint8_t GetGravity(VectorFloat *v, Quaternion *q);
uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity); 
#endif
