#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>

#include <stdlib.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

#include "pid.h"
#include "flightlog.h"
#include "config.h"
#include "motionsensor/interface.h"
#include "receiver/interface.h"
#include "bmpsensor/interface.h"
#include "speedcontroller/interface.h"

#include <native/task.h>
#include <native/timer.h>

#define delay_ms(a) usleep(a*1000)

//see speedcontroller for the mapping
#define MOTOR_FL 5 
#define MOTOR_BL 4 
#define MOTOR_FR 1
#define MOTOR_BR 0 

#define TUNE_KP 0.05f
#define TUNE_KI 0.5f
#define TUNE_TRIM 0.5f //used to correct drift

#define SELF_LANDING_THRUST 350 //in case of receiver failure the quad will be set to the following thrust

static int err;
static RTIME t_err;

int ms_err = 0;
int rec_err = 0;
int bs_err = 0;

int armed = 0;
int inflight = 0;
int emergency = 0;

int mode = 0; //0 - flight; 1 - setup

RTIME t; //start time 

static int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

	float yaw_target = 0.0f;
void log() {
	static unsigned long l = 0;
	static RTIME t1=rt_timer_read();
	static RTIME t2=rt_timer_read();
	static unsigned long dt;

	//calculate dt - time between runs in ms
	t2 = rt_timer_read();

	dt = (t2-t1)/1000000;
	t1 = t2;

	if (rec.yprt[3]>5.0f) { //create log only when in flight
		flog_push(28
			,(float)(t2-t)/1000000
			,ms.gyro[0],ms.gyro[1],ms.gyro[2]
			,ms.ypr[0],ms.ypr[1],ms.ypr[2]
			,rec.yprt[0],rec.yprt[1],rec.yprt[2],rec.yprt[3]
			,config.trim[0],config.trim[1],config.trim[2]
			,config.pid_s[1].Kp,config.pid_s[1].Ki,config.pid_s[1].Kd
			,config.pid_r[1].Kp,config.pid_r[1].Ki,config.pid_r[1].Kd
			,config.pid_r[0].value,config.pid_r[1].value,config.pid_r[2].value
			,ms.c[0],ms.c[1],ms.c[2]
			,bs.alt,bs.t
			);
	}

	
	l++;
	if (l%100 == 0) { //every second
		printf("dt = %lu\ts0=%2.2f,yt=%2.2f,p0=%2.2f\tp1=%2.3f\tp2=%2.3f\tdy=%2.1f,dp=%2.1f,dr=%2.1f,RKp=%2.2f,RKd=%2.2f,SKp=%2.2f,SKi=%2.2f\talt=%2.1f\n",
			//dt,ms.ypr[0],yaw_target,config.pid_r[0].value
			dt,config.pid_s[0].value,yaw_target,config.pid_r[0].value,config.pid_r[1].value,config.pid_r[2].value
			,config.trim[0],config.trim[1],config.trim[2],config.pid_r[1].Kp,config.pid_r[1].Kd,config.pid_s[1].Kp,config.pid_s[1].Ki,bs.alt); 

		}
}

void flush() {
	for (int i=0;i<10;i++) {
		fflush(NULL);
		sync();
	}
}

void do_adjustments() {
	//process receiver command
	float *p1a,*p1b,*p2a,*p2b;

	if (mode) {
		p1a = &config.pid_r[1].Kp;
		p2a = &config.pid_r[2].Kp;
		//p1b = &config.pid_r[1].Kd;
		//p2b = &config.pid_r[2].Kd;
	} else {
		p1a = &config.pid_s[1].Kp;
		p2a = &config.pid_s[2].Kp;
		p1b = &config.pid_s[1].Ki;
		p2b = &config.pid_s[2].Ki;
	}

	switch (rec.aux) {
		case 10:
			(*p1a)+=TUNE_KP; (*p2a)+=TUNE_KP; break;
		case 8:
			(*p1a)-=TUNE_KP; (*p2a)-=TUNE_KP; break;
		case 11:
			(*p1b)+=TUNE_KP; (*p2b)+=TUNE_KP; break;
		case 9:
			(*p1b)-=TUNE_KP; (*p2b)-=TUNE_KP; break;
		case 4: //up button trim
			config.trim[1]+=TUNE_TRIM; break;
		case 6: //down button trim
			config.trim[1]-=TUNE_TRIM; break;
		case 7: //left button trim
			config.trim[2]-=TUNE_TRIM; break;
		case 5: //right button trim
			config.trim[2]+=TUNE_TRIM; break;
		case 15: //yaw left trim
			config.trim[0]+=TUNE_TRIM; break;
		case 13: //yaw right trim
			config.trim[0]-=TUNE_TRIM; break;
		case 16: //yaw right trim
			if (mode ==0) mode =1;
			else mode =0;
			printf("mode: %i\n",mode);
			break;
		case 0: 
			if (!inflight)	{ config_save(); flush(); } break;
	}
	rec.aux=-1; //reset receiver command

	for (int i=0;i<3;i++) {
        	if (config.pid_r[i].Kp<0.0f) config.pid_r[i].Kp=0.0f;
        	if (config.pid_r[i].Ki<0.0f) config.pid_r[i].Ki=0.0f;
        	if (config.pid_r[i].Kd<0.0f) config.pid_r[i].Kd=0.0f;
        	if (config.pid_s[i].Kp<0.0f) config.pid_s[i].Kp=0.0f;
        	if (config.pid_s[i].Ki<0.0f) config.pid_s[i].Ki=0.0f;
        	if (config.pid_s[i].Kd<0.0f) config.pid_s[i].Kd=0.0f;
	}

}

void pre_flight() {
	//wait for gyro to stabilize - this will take around 8 sec - see mpu6050 calibration
	if (!armed && ms.gyro[0]>-1.0f && ms.gyro[1]>-1.0f && ms.gyro[2]>-1.0f &&
		ms.gyro[0]<1.0f && ms.gyro[1]<1.0f && ms.gyro[2]<1.0f) {
			sc_update(MOTOR_FR,100);
			sc_update(MOTOR_FL,100);
			sc_update(MOTOR_BR,100);
			sc_update(MOTOR_BL,100);
			yaw_target = ms.ypr[0];
			armed=1;
			printf("ARMED!\n");
	}
}

void handle_issues() {
	//receiver error during flight
	if (inflight && rec_err<0) {
		if (!emergency)
			t_err = rt_timer_read();

		emergency = 1;
		rec_close();
		rec_open();
	}
	
}

void autoflight() {
	if (emergency) { //emergency //TODO: based on altitude
		static float alt = bs.alt;
		RTIME _t = rt_timer_read();
		unsigned long dt = (long)(_t-t_err)/1000000; //calculate time since error in ms;
		
		rec.yprt[0]=rec.yprt[1]=rec.yprt[2]=0.0f;//stabilize
		if (dt<4000) rec.yprt[3] = SELF_LANDING_THRUST; //do self landing for first 7 sec
		else if (dt<10000) rec.yprt[3] = 0; //decrease thrust by half for another 3sec
		else rec.yprt[3] = 0.0f; //switch engines off 
	}

}

void controller_stable( void *ptr ) {
	float m_fl,m_bl,m_fr,m_br; //motor FL, BL, FR, BR
	unsigned long c = 0;

	static int gyro_rate = 100;
	float loop_ms = 1000.0f/gyro_rate;
	float loop_s = loop_ms/1000.0f;
	RTIME t1;

	while (ms_update()!=0);//empty MPU

	for (int i=0;i<3;i++) {
		pid_setmode(&config.pid_r[i],1);
		pid_setmode(&config.pid_s[i],1);
	}
	flush();
	t=rt_timer_read();
	while(1) {
		ms_err = ms_update();
		if (ms_err==0) continue; //dont do anything if gyro has no new data; depends on gyro RATE (100Hz)
		if (ms_err<0) { //something wrong with gyro: i2c issue or fifo full!
			ms.ypr[0]=ms.ypr[1]=ms.ypr[2] = 0.0f; 
			ms.gyro[0]=ms.gyro[1]=ms.gyro[2] = 0.0f;
		}
		c++; //our counter so we know when to read barometer (c increases every 10ms)
		bs_err = bs_update(c*loop_ms);

		rec_err = rec_update();
		
		pre_flight();
		handle_issues();
		autoflight();

		if (armed && rec.yprt[3]>5.0f) { //5% of throttle
			if (!inflight) {//enable pids since we are changing state changes to inflight 
				printf("inflight is on\n");
			}
			inflight = 1; 
		}

		if (rec.yprt[3]<=5.0f) {
			if (inflight) {//disable pids since we are changing state changes to no inflight 
				printf("inflight is off!\n");
			}
			inflight = 0;	
			sc_update(MOTOR_FL,1000);
			sc_update(MOTOR_BL,1000);
			sc_update(MOTOR_FR,1000);
			sc_update(MOTOR_BR,1000);
			yaw_target = ms.ypr[0];	
			bs.p0 = bs.p;
		}
		
		if (rec.yprt[3]<50.0f) {
			for (int i=0;i<3;i++) { 
				config.pid_r[i]._KiTerm = 0.0f;
				config.pid_s[i]._KiTerm = 0.0f;
			}
		}	

		do_adjustments();
	
		//our quad can rotate 360 degree if commanded, it is ok!; learned it the hard way!
		if (yaw_target-ms.ypr[0]<-180.0f) yaw_target*=-1;
		if (yaw_target-ms.ypr[0]>180.0f) yaw_target*=-1;

		//do STAB PID
		for (int i=0;i<3;i++) { 
			if (i==0) pid_update(&config.pid_s[i],yaw_target,ms.ypr[i],loop_s);
			else
				pid_update(&config.pid_s[i],rec.yprt[i]+config.trim[i],ms.ypr[i],loop_s);
			
		}

		//manually set yaw
		if (abs(rec.yprt[0])>2.0f) {
			config.pid_s[0].value = rec.yprt[0];
			yaw_target = ms.ypr[0];	
		}

		if (mode) for (int i=0;i<3;i++) config.pid_s[i].value = 0;
		//do RATE PID
		for (int i=0;i<3;i++) { 
			pid_update(&config.pid_r[i],config.pid_s[i].value,ms.gyro[i],loop_s);
		}

		//calculate motor speeds
		m_fl = rec.yprt[3]+config.pid_r[2].value-config.pid_r[1].value+config.pid_r[0].value;
		m_bl = rec.yprt[3]+config.pid_r[2].value+config.pid_r[1].value-config.pid_r[0].value;
		m_fr = rec.yprt[3]-config.pid_r[2].value-config.pid_r[1].value-config.pid_r[0].value;
		m_br = rec.yprt[3]-config.pid_r[2].value+config.pid_r[1].value+config.pid_r[0].value;

		m_fl = map(m_fl,0,700,1050,1860); //afro esc min: 1060, max 1860
		m_bl = map(m_bl,0,700,1050,1860);
		m_fr = map(m_fr,0,700,1050,1860);
		m_br = map(m_br,0,700,1050,1860);

		//log();

		if (inflight) {
			sc_update(MOTOR_FL,m_fl);
			sc_update(MOTOR_BL,m_bl);
			sc_update(MOTOR_FR,m_fr);
			sc_update(MOTOR_BR,m_br);
		}
	}
}

void catch_signal(int sig)
{
	sc_close();
}


int main() {
        signal(SIGTERM, catch_signal);
        signal(SIGINT, catch_signal);


        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

	err=config_open("/var/local/rpicopter.config");
	if (err<0) {
            	printf("Failed to initiate config! [%s]\n", strerror(err));	
		return -1;
	}

	err=flog_open("/var/local/flight.log");
	if (err<0) {
            	printf("Failed to initiate log! [%s]\n", strerror(err));	
		return -1;
	}

	err = sc_open();
        if (err != 0) {
            printf("Failed to initiate speedcontroller! [%s]\n", strerror(err));	
	    return -1;
	}

	err=ms_open();
        if (err != 0) {
            printf("Failed to initiate gyro! [%s]\n", strerror(err));	
	    return -1;
	}

	err=rec_open();
	if (err<0) {
            	printf("Failed to initiate receiver! [%s]\n", strerror(err));	
		return -1;
	}

	err=bs_open();
	if (err<0) {
            	printf("Failed to initiate pressure sensor! [%s]\n", strerror(err));	
		return -1;
	}

	delay_ms(1000);
	controller_stable(NULL);
	return 0;
}
