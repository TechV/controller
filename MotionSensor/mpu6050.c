#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "helper_3dmath.h"
#include "../MotionSensor.h"
#include "inv_mpu_lib/inv_mpu.h"
#include "inv_mpu_lib/inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// MPU control/status vars
int16_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation
                          //(0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t g[3];              // [x, y, z]            gyro vector
Quaternion q;           // [w, x, y, z]         quaternion container
int32_t _q[4];

VectorFloat gravity;    // [x, y, z]            gravity vector

int r;
int initialized = 0;
int dmpReady = 0;
float lastval[3];
int16_t sensors;

float ypr[3];
float gyro[3];

void _initialize() {
  if (!dmpReady) return;

  printf("Initializing IMU...\n");

  dmp_set_fifo_rate(20);
  mpu_reset_fifo();
  printf("Collecting... ");
  do {
	  sleep(1);
	  while (dmp_read_fifo(g,NULL,_q,&sensors,&fifoCount)!=0);
	  printf("%u ",fifoCount);
  } while (fifoCount<42 || fifoCount>=1024);
  printf("\n");
  
  while (dmp_read_fifo(g,NULL,_q,&sensors,&fifoCount)!=0); //gyro and accel can be null because of being disabled in the efeatures
  q = _q;
  GetGravity(&gravity, &q);
  GetYawPitchRoll(ypr, &q, &gravity);

  printf("IMU init done; offset values are :\n");
  printf("yaw = %f, pitch = %f, roll = %f\n\n",
	 ypr[YAW]*180/M_PI, ypr[PITCH]*180/M_PI,
	 ypr[ROLL]*180/M_PI);
  initialized = 1;
}


int ms_open() {
  dmpReady=1;
  initialized = 0;
  for (int i=0;i<DIM;i++){
    lastval[i]=10;
  }

   // initialize device
    printf("Initializing I2C devices...\n");

    if (mpu_init(NULL) != 0) return -1;
    mpu_set_gyro_fsr(250);
    mpu_set_accel_fsr(2);

    // verify connection
    printf("Testing device connections...\n");
    mpu_get_power_state(&devStatus);
    printf(devStatus ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    printf("Initializing DMP...\n");
    mpu_get_dmp_state(&devStatus);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
 	mpu_set_dmp_state(1);
	dmp_load_motion_driver_firmware();
	//dmp_set_orientation()
	dmp_enable_feature(DMP_FEATURE_LP_QUAT|DMP_FEATURE_SEND_CAL_GYRO);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
	mpu_get_int_status(&mpuIntStatus);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
	//packetSize = MAX_PACKET_LENGTH;
	packetSize = 0;//dmp.packet_length;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }
    
    _initialize();
    return 0;
}

int ms_update() {
  if (!dmpReady) return -1;

  // wait for FIFO count > 42 bits
  do {
    while (dmp_read_fifo(g,NULL,_q,&sensors,&fifoCount)!=0); //gyro and accel can be null because of being disabled in the efeatures
  }while (fifoCount<42);

  if (fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu_reset_fifo();
    printf("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt
    //(this should happen frequently)
  } else  {
    //read packet from fifo
    //mpu.getFIFOBytes(fifoBuffer, packetSize);

    //mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetGravity(&gravity, &q);
    //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    while (dmp_read_fifo(g,NULL,_q,&sensors,&fifoCount)!=0); //gyro and accel can be null because of being disabled in the efeatures
    GetGravity(&gravity, &q);
    GetYawPitchRoll(ypr, &q, &gravity);

    //scaling for degrees output
    for (int i=0;i<DIM;i++){
      ypr[i]*=180/M_PI;
    }

    //unwrap yaw when it reaches 180
    ypr[0] = wrap_180(ypr[0]);

    //change sign of Pitch, MPU is attached upside down
    ypr[1]*=-1.0;

    //mpu.GetGyro(g, fifoBuffer);

    //0=gyroX, 1=gyroY, 2=gyroZ
    //swapped to match Yaw,Pitch,Roll
    //Scaled from deg/s to get tr/s
     for (int i=0;i<DIM;i++){
       gyro[i]   = (float)(g[DIM-i-1])/131.0/360.0;
     }

    // printf("gyro  %7.2f %7.2f %7.2f    \n", (float)g[0]/131.0,
    // 	   (float)g[1]/131.0,
    // 	   (float)g[2]/131.0);

  }
  return 0;
}

int ms_close() {
	return 0;
} 

uint8_t GetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}

