#ifndef MPU6050_H
#define MPU6050_H

struct s_ms ms = {
        .ypr = {0.0f,0.0f,0.0f},
        .gyro = {0.0f,0.0f,0.0f},
        .c = {0.0f,0.0f,0.0f},
};

int ms_open(unsigned int rate);
int ms_update();
int ms_close();

#endif
