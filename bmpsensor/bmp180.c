/*
	SFE_BMP180.cpp
	Bosch BMP180 pressure sensor library for the Arduino microcontroller
	Mike Grusin, SparkFun Electronics

	Uses floating-point equations from the Weather Station Data Logger project
	http://wmrx00.sourceforge.net/
	http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf

	Forked from BMP085 library by M.Grusin

	version 1.0 2013/09/20 initial version

	Our example code uses the "beerware" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me a (root) beer someday.
*/

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "interface.h"
#include "bmp180.h"
#include "i2cdev/i2cdev.h"

#define OVERSAMPLING 1 //oversampling_setting
#define PRESSURE_DELAY 7500 //ms
#define delay_ms(a)    usleep(a*1000)

#define BMP180_ADDR 0x77 // 7-bit address

#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6

#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0 0x34
#define BMP180_COMMAND_PRESSURE1 0x74
#define BMP180_COMMAND_PRESSURE2 0xB4
#define BMP180_COMMAND_PRESSURE3 0xF4

static struct s_cc {
	short AC1,AC2,AC3,B1,B2,MB,MC,MD;
	unsigned short AC4,AC5,AC6;
	long X1,X2,X3,B3,B5,B6;
	unsigned long B4,B7;
} cc; //calibration coefficients

static int ret = 0;

#define BUF_SIZE 100
static long buffer[BUF_SIZE];
static long *buf_ptr = buffer;


//KalmanCalc() - Calculates new Kalman values from float value "altitude"
 //Kalman Variables 
float f_1=1.00000; 
float kalman_x;
float kalman_x_last;
float kalman_p;
float kalman_p_last;
float kalman_k;
float kalman_q;
float kalman_r;
float kalman_x_temp;
float kalman_p_temp;

void KalmanInit()
{
   kalman_q=4.0001f; //filter parameters, you can play around with them
   kalman_r=.20001f; // but these values appear to be fairly optimal

   kalman_x = 0;
   kalman_p = 0;
   kalman_x_temp = 0;
   kalman_p_temp = 0;
   
   kalman_x_last = 0;
   kalman_p_last = 0;
      
}

float KalmanCalc (float altitude)
 {
   
   //Predict kalman_x_temp, kalman_p_temp
   kalman_x_temp = kalman_x_last;
   kalman_p_temp = kalman_p_last + kalman_r;
   
   //Update kalman values
   kalman_k = (f_1/(kalman_p_temp + kalman_q)) * kalman_p_temp;
   kalman_x = kalman_x_temp + (kalman_k * (altitude - kalman_x_temp));
   kalman_p = (f_1 - kalman_k) * kalman_p_temp;
   
   //Save this state for next time
   kalman_x_last = kalman_x;
   kalman_p_last = kalman_p;
   
      
  return kalman_x;
 } 


int bs_open()
// Initialize library for subsequent pressure measurements
{
	

	// The BMP180 includes factory calibration data stored on the device.
	// Each device has different numbers, these must be retrieved and
	// used in the calculations when taking pressure measurements.

	// Retrieve calibration data from device:
	
	if (bsReadS(0xAA,cc.AC1) &&
		bsReadS(0xAC,cc.AC2) &&
		bsReadS(0xAE,cc.AC3) &&
		bsReadU(0xB0,cc.AC4) &&
		bsReadU(0xB2,cc.AC5) &&
		bsReadU(0xB4,cc.AC6) &&
		bsReadS(0xB6,cc.B1) &&
		bsReadS(0xB8,cc.B2) &&
		bsReadS(0xBA,cc.MB) &&
		bsReadS(0xBC,cc.MC) &&
		bsReadS(0xBE,cc.MD))
	{

/*
		cc.AC1=408;
		cc.AC2=-72;
		cc.AC3=-14383;
		cc.AC4=32741;
		cc.AC5=32757;
		cc.AC6=23153;
		cc.B1=6190;
		cc.B2=4;
		cc.MB=-32768;
		cc.MC=-8711;
		cc.MD=2868;
*/
		// All reads completed successfully!
		// If you need to check your math using known numbers,
		// you can uncomment one of these examples.
		// (The correct results are commented in the below functions.)

		// Example from Bosch datasheet
		// AC1 = 408; AC2 = -72; AC3 = -14383; AC4 = 32741; AC5 = 32757; AC6 = 23153;
		// B1 = 6190; B2 = 4; MB = -32768; MC = -8711; MD = 2868;

		// Example from http://wmrx00.sourceforge.net/Arduino/BMP180-Calcs.pdf
		// AC1 = 7911; AC2 = -934; AC3 = -14306; AC4 = 31567; AC5 = 25671; AC6 = 18974;
		// VB1 = 5498; VB2 = 46; MB = -32768; MC = -11075; MD = 2432;

#ifdef BMP_DEBUG
		printf("AC1: %i\n",cc.AC1);
		printf("AC2: %i\n",cc.AC2);
		printf("AC3: %i\n",cc.AC3);
		printf("AC4: %u\n",cc.AC4);
		printf("AC5: %u\n",cc.AC5);
		printf("AC6: %u\n",cc.AC6);
		printf("VB1: %i\n",cc.B1);
		printf("VB2: %i\n",cc.B2);
		printf("MB: %i\n",cc.MB);
		printf("MC: %i\n",cc.MC);
		printf("MD: %i\n",cc.MD); 
#endif
		
		// Compute floating-point polynominals:

		// Success!
		KalmanInit();
		for (int i=0;i<BUF_SIZE;i++) {
		ret=bs_update();
		if (ret<0) {
			printf("BMP: Failed to get an update (%i)!\n",ret);
			return ret;
		}
		}
		bs.p0 = bs.p;

		return(0);
	}
	else
	{
		printf("BMP: Failed to read calibration valued!\n");
		// Error reading calibration data; bad component or connection?
		return(-1);
	}
}

int bs_update() {
	pthread_mutex_lock( &bs.mutex );
	prepareTemperature();
	usleep(4500);
	ret = getTemperature(bs.t);
	if (ret<0) {
		pthread_mutex_unlock( &bs.mutex );
		printf("Error temp!\n");
		return 100+ret;
	}
	preparePressure(OVERSAMPLING); //need to wait at least 20ms?? see datasheet
	usleep(PRESSURE_DELAY);
	ret = getPressure(bs.p);
	if (ret<0) {
		pthread_mutex_unlock( &bs.mutex );
		printf("Error press!\n");
		return 200+ret;
	}
	bs.alt = altitude(bs.p, bs.p0); 

        clock_gettime(CLOCK_REALTIME, &bs.ts);
	pthread_mutex_unlock( &bs.mutex );
	return 0;
}


static int prepareTemperature(void)
// Begin a temperature reading.
{
	unsigned char data[2], result;
	
	data[0] = BMP180_REG_CONTROL;
	data[1] = BMP180_COMMAND_TEMPERATURE;
	ret = bsWriteBytes(data, 2);
	return ret;
}

static int getTemperature(float &T)
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
{
	unsigned char data[2];
	long tu;
	
	data[0] = BMP180_REG_RESULT;

	ret = bsReadBytes(data, 2);
	if (ret>0) // good read, calculate temperature
	{
		tu = (((long)data[0]) << 8) + ((long)data[1]);
		//tu = 27898;
		cc.X1 = ((tu - cc.AC6) * cc.AC5) >> 15;
		cc.X2 = (cc.MC << 11) / (cc.X1 + cc.MD);
		cc.B5 = cc.X1 + cc.X2;
		T = ((cc.B5 + 8) >> 4)/10.0f;

#ifdef BMP_DEBUG
		printf("tu: %5.3f\ta: %5.3f\tT:%5.3f\n",tu,a,T);
#endif
	}
	return ret;
}

static int preparePressure(int oversampling)
// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
{
	unsigned char data[2];
	
	data[0] = BMP180_REG_CONTROL;

	switch (oversampling)
	{
		case 0:
			data[1] = BMP180_COMMAND_PRESSURE0;
		break;
		case 1:
			data[1] = BMP180_COMMAND_PRESSURE1;
		break;
		case 2:
			data[1] = BMP180_COMMAND_PRESSURE2;
		break;
		case 3:
			data[1] = BMP180_COMMAND_PRESSURE3;
		break;
		default:
			data[1] = BMP180_COMMAND_PRESSURE0;
		break;
	}
	ret=bsWriteBytes(data,2);  // good write?
	return ret;
}


static int getPressure(float &P)
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.

// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
{
	unsigned char data[3];
	long up = 0l;
	long p;
	
	data[0] = BMP180_REG_RESULT;
	data[1] = data[2] = 0;

	if (ret = bsReadBytes(data,3)) // good read, calculate pressure
	{
		up = (((long)(data[0]) << 16) + ((long)(data[1])<<8) + (long)(data[2])) >> (8-OVERSAMPLING);
		//up = 23843;

		cc.B6 = cc.B5-4000;
		cc.X1 = (cc.B2*((cc.B6*cc.B6)>>12))>>11;
		cc.X2 = (cc.AC2*cc.B6)>>11;
		cc.X3 = cc.X1+cc.X2;
		cc.B3 = (((cc.AC1*4+cc.X3)<<OVERSAMPLING)+2)/4;
		cc.X1 = (cc.AC3*cc.B6)>>13;
		cc.X2 = (cc.B1*((cc.B6*cc.B6)>>12))>>16;
		cc.X3 = ((cc.X1+cc.X2)+2)>>2;
		cc.B4 = (cc.AC4*(unsigned long)(cc.X3+32768))>>15;
		cc.B7 = ((unsigned long)up-cc.B3)*(50000>>OVERSAMPLING);
		if (cc.B7<0x80000000) p = (cc.B7*2)/cc.B4;
		else p = (cc.B7/cc.B4)*2;
		cc.X1 = (p>>8) * (p>>8);
		cc.X1 = (cc.X1*3038)>>16;
		cc.X2 = (-7357*p)>>16;
		p = p + ((cc.X1+cc.X2+3791)>>4);
		*buf_ptr = p;
		if (buf_ptr-buffer==(BUF_SIZE-1)) buf_ptr=buffer;
		else buf_ptr++;
		
		long c = 0l;
		for (int i=0;i<BUF_SIZE;i++) c+=buffer[i];
		c/=BUF_SIZE;
		P = (float)c;
		//P = c/100.0f;


#ifdef BMP_DEBUG
		printf("up: %li\n",up); 
		printf("B6: %li\n",cc.B6);
		printf("X1: %li\n",cc.X1);
		printf("X2: %li\n",cc.X2);
		printf("X3: %li\n",cc.X3);
		printf("B3: %li\n",cc.B3);
		printf("P: %li\n",cc);
#endif
		return 0;
	}
	return ret;
}


static float altitude(float P, float P0)
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
	return KalmanCalc(44330.0f*(1.0f-pow(P/P0,1.0f/5.255f)));
}


static int bsReadS(unsigned char address, short &value)
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned short data[2];

	data[0] = address;
	//printf("%#x %#x\n",data[0],data[1]);
	ret=bsReadBytes((unsigned char*)data,2);
	//printf("%#x %#x\n\n\n\n\n\n",data[0],data[1]);
	if (ret>0)
	{
		value = (((short)data[0])<<8)|((short)data[1]);
		//value = (int)((((int)data[0])<<8)|((int)data[1]));
		//if (value & 0x8000) value |= 0xFFFF0000; // sign extend if negative
		return(ret);
	}
	value = 0;
	return(ret);
}


static int bsReadU(unsigned char address, unsigned short &value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned short data[2];

	data[0] = address;
	ret=bsReadBytes((unsigned char*)data,2);
	if (ret>0)
	{
		value = (data[0]<<8)|data[1];
		return(ret);
	}
	value = 0;
	return(ret);
}


static int bsReadBytes(unsigned char *values, char length)
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
{
	return readBytes(BMP180_ADDR,values[0],length,values); //return number of bytes read
}


static int bsWriteBytes(unsigned char *values, char length)
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
{
	ret=writeBytes(BMP180_ADDR,values[0],length-1,values+1);
	return ret;
}



