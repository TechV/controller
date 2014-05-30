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

#define GYRO_RATE 200 

#define SELF_LANDING_THRUST 1300 //in case of receiver failure the quad will be set to the following thrust

static int err;
static RTIME t_err;

int ms_err = 0;
int rec_err = 0;
int bs_err = 0;

int armed = 0;
int inflight = 0;
int emergency = 0;

int mode = 0; //0 - flight; 1 - setup yaw; 2 - setup pitch; 3 - setup roll
float yaw_target = 0.0f;

RTIME t; //start time 

void log() {
	static unsigned long l = 0;
	static RTIME t1=rt_timer_read();
	static RTIME t2=rt_timer_read();
	static unsigned long dt;

	//calculate dt - time between runs in ms
	t2 = rt_timer_read();

	dt = (t2-t1)/1000000;
	t1 = t2;

	l++;
	if (l%200 == 0) { 
	if (inflight) { //create file log only when in flight
		flog_push(12
			,dt,dt
			,ms.ypr[0],ms.ypr[1],ms.ypr[2]
			,ms.gyro[0],ms.gyro[1],ms.gyro[2]
			,rec.yprt[0],rec.yprt[1],rec.yprt[2],rec.yprt[3]
			);
	}

		printf("c: %i\ty: %2.1f, p: %2.1f, r: %2.1f\tgy: %2.1f,gp: %2.1f,gr: %2.1f\try: %2.1f, rp: %2.1f, rr: %2.1f, rt: %2.1f\n",
			,ms.count
			,ms.ypr[0],ms.ypr[1],ms.ypr[2]
			,ms.gyro[0],ms.gyro[1],ms.gyro[2]
			,rec.yprt[0],rec.yprt[1],rec.yprt[2],rec.yprt[3]
		}
}

void flush() {
	for (int i=0;i<10;i++) {
		fflush(NULL);
		sync();
	}
}

void do_adjustments() {
	if (rec.aux<0) return;

	static float adj1; //for Kp
	static float adj2; //for Ki
	static float adj3 = 0.5f; //for trim

	static float *v1,*v2,*v3,*v4;
	static float _dummy = 0.0f;
	static float *dummy = &_dummy;

	if (mode == 0) { //normal - stabilized flight mode
		adj1 = 0.1f;
		adj2 = 0.5f;
		v1 = &config.pid_s[1].Kp;
		v2 = &config.pid_s[2].Kp;
		v3 = &config.pid_s[1].Ki;
		v4 = &config.pid_s[2].Ki;
	} else if (mode == 1) { //setup mode - setup yaw
		adj1 = 0.025f;
		v1 = &config.pid_r[0].Kp;
		v2 = v3 = v4 = dummy;
	} else if (mode == 2) { //setup pitch
		adj1 = 0.025f;
		v1 = &config.pid_r[1].Kp;
		v2 = v3 = v4 = dummy;
	} else if (mode == 3) { //setup roll
		adj1 = 0.025f;
		v1 = &config.pid_r[2].Kp;
		v2 = v3 = v4 = dummy;
	}

	switch (rec.aux) {
		case 10:
			(*v1)+=adj1; (*v2)+=adj1; break;
		case 8:
			(*v1)-=adj1; (*v2)-=adj1; break;
		case 11:
			(*v3)+=adj2; (*v3)+=adj2; break;
		case 9:
			(*v3)-=adj2; (*v3)-=adj2; break;
		case 4: //up button trim
			config.trim[1]+=adj3; break;
		case 6: //down button trim
			config.trim[1]-=adj3; break;
		case 7: //left button trim
			config.trim[2]+=adj3; break;
		case 5: //right button trim
			config.trim[2]-=adj3; break;
		case 16: 
			mode++;
			if (mode==4) mode=0;
			break;
		case 0: 
			if (!inflight)	{ config_save(); flog_save(); flush(); } break;
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
			printf("Gyro calibrated. \n");
			yaw_target = ms.ypr[0];
			armed=1;
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
		RTIME _t = rt_timer_read();
		unsigned long dt = (long)(_t-t_err)/1000000; //calculate time since error in ms;
		
		rec.yprt[0]=rec.yprt[1]=rec.yprt[2]=0.0f;//stabilize
		if (dt<4000) rec.yprt[3] = SELF_LANDING_THRUST; //do self landing for first 4 sec
		else rec.yprt[3] = 0.0f; //switch engines off after 4 sec 
	}

}

void controller_stable( void *ptr ) {
	unsigned long c = 0;
	float m_fl,m_bl,m_fr,m_br; //motor FL, BL, FR, BR

	float loop_ms = 1000.0f/GYRO_RATE;
	float loop_s = loop_ms/1000.0f;

	while (ms_update()!=0);//empty MPU

	for (int i=0;i<3;i++) {
		pid_setmode(&config.pid_r[i],1);
		pid_setmode(&config.pid_s[i],1);
	}
	flush();
	t=rt_timer_read();
	while(1) {
		ms_err = ms_update();
		if (ms_err==0) continue; //dont do anything if gyro has no new data; depends on gyro RATE
		if (ms_err<0) { //something wrong with gyro: i2c issue or fifo full!
			ms.ypr[0]=ms.ypr[1]=ms.ypr[2] = 0.0f; 
			ms.gyro[0]=ms.gyro[1]=ms.gyro[2] = 0.0f;
		}
		c++; //our counter so we know when to read barometer (c increases every 10ms)
		//bs_err = bs_update(c*loop_ms);

		rec_err = rec_update();
		
		pre_flight();
		handle_issues();
		autoflight();

		if (armed && rec.yprt[3]>(config.esc_min+10)) { 
			inflight = 1; 
		}

		if (rec.yprt[3]<=(config.esc_min+10)) {
			inflight = 0;	
			sc_update(MOTOR_FL,config.esc_min);
			sc_update(MOTOR_BL,config.esc_min);
			sc_update(MOTOR_FR,config.esc_min);
			sc_update(MOTOR_BR,config.esc_min);
			yaw_target = ms.ypr[0];	
			bs.p0 = bs.p;
		}
		
		if (rec.yprt[3]<(config.esc_min+50)) { //use integral part only if there is some throttle
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
			if (i==0) //keep yaw_target 
				pid_update(&config.pid_s[i],yaw_target,ms.ypr[i],loop_s);
			else
				pid_update(&config.pid_s[i],rec.yprt[i]+config.trim[i],ms.ypr[i],loop_s);
			
		}

		//yaw requests will be fed directly to rate pid
		if (abs(rec.yprt[0])>7.5f) {
			config.pid_s[0].value = rec.yprt[0];
			yaw_target = ms.ypr[0];	
		}

		if (mode == 1) { //yaw setup
			config.pid_s[0].value = 0.0f;
			config.pid_s[1].value = ms.gyro[1];
			config.pid_s[2].value = ms.gyro[2];
		} else if (mode == 2) { //pitch setup
			config.pid_s[0].value = ms.gyro[0];
			config.pid_s[1].value = 0.0f;
			config.pid_s[2].value = ms.gyro[2];
		} else if (mode == 3) { //roll setup
			config.pid_s[0].value = ms.gyro[0];
			config.pid_s[1].value = ms.gyro[1];
			config.pid_s[2].value = 0.0f;
		}

		//do RATE PID
		for (int i=0;i<3;i++) { 
			pid_update(&config.pid_r[i],config.pid_s[i].value,ms.gyro[i],loop_s);
		}

		//calculate motor speeds
		m_fl = rec.yprt[3]-config.pid_r[2].value-config.pid_r[1].value+config.pid_r[0].value;
		m_bl = rec.yprt[3]-config.pid_r[2].value+config.pid_r[1].value-config.pid_r[0].value;
		m_fr = rec.yprt[3]+config.pid_r[2].value-config.pid_r[1].value-config.pid_r[0].value;
		m_br = rec.yprt[3]+config.pid_r[2].value+config.pid_r[1].value+config.pid_r[0].value;

		log();

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

#define MAX_SAFE_STACK (MAX_LOG*MAX_VALS + 64*1024)
void stack_prefault(void) {

        unsigned char dummy[MAX_SAFE_STACK];

        memset(dummy, 0, MAX_SAFE_STACK);
        return;
}

int main() {
	struct sched_param param;

        signal(SIGTERM, catch_signal);
        signal(SIGINT, catch_signal);

	param.sched_priority = 49;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                perror("sched_setscheduler failed");
                exit(-1);
        }


        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

	stack_prefault();

	err=config_open("/var/local/rpicopter.config");
	if (err<0) {
            	printf("Failed to initiate config! [%s]\n", strerror(err));	
		return -1;
	}

	err=flog_open("/var/local/");
	if (err<0) {
            	printf("Failed to initiate log! [%s]\n", strerror(err));	
		return -1;
	}

	err = sc_open();
        if (err != 0) {
            printf("Failed to initiate speedcontroller! [%s]\n", strerror(err));	
	    return -1;
	}

	err=ms_open(GYRO_RATE);
        if (err != 0) {
            printf("Failed to initiate gyro! [%s]\n", strerror(err));	
	    return -1;
	}

	err=rec_open();
	if (err<0) {
            	printf("Failed to initiate receiver! [%s]\n", strerror(err));	
		return -1;
	}

/*
	err=bs_open();
	if (err<0) {
            	printf("Failed to initiate pressure sensor! [%s]\n", strerror(err));	
		return -1;
	}
*/
	printf("Waiting for MPU calibration... \n");
	delay_ms(1000);
	controller_stable(NULL);
	return 0;
}
