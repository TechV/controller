#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "interface.h"
#include "inv_mpu_lib/inv_mpu.h"
#include "inv_mpu_lib/inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"

#define delay_ms(a)    usleep(a*1000)

static int16_t c[3]; //compass
static int16_t a[3];              // [x, y, z]            accel vector
static int16_t g[3];              // [x, y, z]            gyro vector
static int32_t _q[4];
static float q[4];

static int r=0;
static int initialized = 0;
static int dmpReady = 0;
static int16_t sensors;
static uint8_t devStatus;      // return status after each device operation
static uint8_t fifoCount;     // count of all bytes currently in FIFO

static int err = 0;

static const signed char gyro_orientation[9] = { 1, 0, 0, //changing this means you will need to update QuaternionToEuler too
						 0, 1, 0,
						 0, 0, 1 };

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

#define FSR			2000
#define GYRO_SENS	( 131.0f * 250.0 / (float)FSR )
#define QUAT_SENS	1073741824.f //2^30

#define EPSILON		0.0001f
#define PI			3.14159265358979323846f
#define PI_2		1.57079632679489661923f

static void quaternionToEuler( const float* quat_wxyz, float* x, float* y, float* z )
{
	float test;
	const struct quat { float w, x, y, z; } *q = ( const struct quat* )quat_wxyz;

	float sqy = q->y * q->y;
	float sqz = q->z * q->z;
	float sqw = q->w * q->w;

	test = q->x * q->z - q->w * q->y;

	if( test > 0.5f - EPSILON )
	{
		*x = 2.f * atan2( q->y, q->w );
		*y = PI_2;
		*z = 0;
	}
	else if( test < -0.5f + EPSILON )
	{
		*x = -2.f * atan2( q->y, q->w );
		*y = -PI_2;
		*z = 0;
	}
	else
	{
		*x = atan2( 2.f * ( q->x * q->w + q->y * q->z ), 1.f - 2.f * ( sqz + sqw ) );
		*y = asin( 2.f * test );
		*z = atan2( 2.f * ( q->x * q->y - q->z * q->w ), 1.f - 2.f * ( sqy + sqz ) );
	}
}

static float rad2deg( float rad )
{
	return 180.f * rad / PI;
}



int ms_open(unsigned int rate) {
	dmpReady=1;
	initialized = 0;

	// initialize device
	printf("Initializing MPU...\n");
	r=mpu_init(NULL); 
	if (r != 0) {
		printf("MPU init failed! %i\n",r);
		return -1;
	}
	printf("Setting MPU sensors...\n");
	if (mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) {
		printf("Failed to set sensors!\n");
		return -1;
	}
	printf("Setting GYRO sensitivity...\n");
	if (mpu_set_gyro_fsr(FSR)!=0) {
		printf("Failed to set gyro sensitivity!\n");
		return -1;
	}
	printf("Setting ACCEL sensitivity...\n");
	if (mpu_set_accel_fsr(2)!=0) {
		printf("Failed to set accel sensitivity!\n");
		return -1;
	}
	// verify connection
	printf("Powering up MPU...\n");
	mpu_get_power_state(&devStatus);
	printf(devStatus ? "MPU6050 connection successful\n" : "MPU6050 connection failed %u\n",devStatus);

	//fifo config
	printf("Setting MPU fifo...\n");
	if (mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) {
		printf("Failed to initialize MPU fifo!\n");
		return -1;
	}

	// load and configure the DMP
	printf("Loading DMP firmware...\n");
	if (dmp_load_motion_driver_firmware()!=0) {
		printf("Failed to load DMP firmware!\n");
		return -1;
	}

	printf("Setting GYRO orientation...\n");
	if (dmp_set_orientation( inv_orientation_matrix_to_scalar( gyro_orientation ) )!=0) {
		printf("Failed to set gyro orientation!\n");
		return -1;
	}

	printf("Setting DMP fifo rate to: %i\n",rate);
	if (dmp_set_fifo_rate(rate)!=0) {
		printf("Failed to set dmp fifo rate!\n");
		return -1;
	}

	printf("Activating DMP...\n");
	if (mpu_set_dmp_state(1)!=0) {
		printf("Failed to enable DMP!\n");
		return -1;
	}

	//dmp_set_orientation()
	//if (dmp_enable_feature(DMP_FEATURE_LP_QUAT|DMP_FEATURE_SEND_RAW_GYRO)!=0) {
	printf("Configuring DMP...\n");
	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL)!=0) {
	//if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_RAW_GYRO)!=0) {
		printf("Failed to enable DMP features!\n");
		return -1;
	}

	printf("Resetting fifo queue...\n");
	if (mpu_reset_fifo()!=0) {
		printf("Failed to reset fifo!\n");
		return -1;
	}

	printf("Checking... ");
	do {
		delay_ms(5*1000/rate);  //dmp will habve 4 (5-1) packets based on the fifo_rate
		r=dmp_read_fifo(g,a,_q,&sensors,&fifoCount);
	} while (r!=0 || fifoCount<5); //packtets!!!
	printf("Collected: %i packets.\n",fifoCount);

	ms_update();
	initialized = 1;

	uint16_t lpf,g_fsr,a_sens,dmp_rate;
	uint8_t dmp,a_fsr;
	float g_sens;
	printf("MPU config:\n");
	mpu_get_lpf(&lpf);
	printf("MPU LPF: %u\n",lpf);
	dmp_get_fifo_rate(&dmp_rate);
	printf("DMP Fifo Rate: %u\n",dmp_rate);
	mpu_get_dmp_state(&dmp);
	printf("MPU DMP State: %u\n",dmp);
	mpu_get_gyro_fsr(&g_fsr);
	printf("MPU gyro FSR: %u\n",g_fsr);
	mpu_get_gyro_sens(&g_sens);
	printf("MPU gyro sens: %2.3f\n",g_sens);
	mpu_get_accel_fsr(&a_fsr);
	printf("MPU accel FSR: %u\n",a_fsr);
	mpu_get_accel_sens(&a_sens);
	printf("MPU accel sens: %u\n",a_sens);
	
	return 0;
}

int ms_update() {
	static unsigned int dt = 0;
	float x,y,z;
	if (!dmpReady) {
		printf("Error: DMP not ready!!\n");
		return -1;
	}
	fifoCount = -1;
	err=dmp_read_fifo(g,a,_q,&sensors,&fifoCount); //gyro and accel can be null because of being disabled in the efeatures
	
	ms.count = fifoCount;

	if (fifoCount>1) {
		//printf("MPU fifo queue: %i!!!\n",fifoCount);
		do {
			fifoCount = -1;
			err=dmp_read_fifo(g,a,_q,&sensors,&fifoCount); //gyro and accel can be null because of being disabled in the efeatures
		} while (fifoCount>1);
	}	

	if (err!=0 && fifoCount!=0) return -1;
	if (err && fifoCount==0) return 0; //no new data

	for(int i = 0; i < 4; ++i )
		q[i] = (float)_q[i] / QUAT_SENS;

	quaternionToEuler( q, &x, &y, &z );

	//euler and gyro must be in sync, i.e. MPU pitches left, euler decreases and gyro shows minus, etc 
	//0 = yaw
 	//1 = pitch
	//2 = roll
	ms.ypr[0] = rad2deg(z);
	ms.ypr[1] = rad2deg(y);
	ms.ypr[2] = -rad2deg(x);

	ms.gyro[0] = -(float)g[2] / GYRO_SENS;
	ms.gyro[1] = (float)g[1] / GYRO_SENS;
	ms.gyro[2] = (float)g[0] / GYRO_SENS;
	
	return 1;
}

int ms_close() {
	return 0;
}

