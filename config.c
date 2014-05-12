#include "config.h"
#include <stdio.h>

struct s_config config;

static FILE *f;
static const char *p;

int config_open(const char *path) {
	int state = 0;
	p = path;
        f = fopen(p,"r");
        if (f == NULL) state = 1; 

	if (state ==0) {

		if (fscanf(f,"%i\t%i\n",&config.esc_min,&config.esc_max)<0) state = 1;
		if (fscanf(f,"%f\t%f\t%f\n",&config.pid_r[0].min,&config.pid_r[1].min,&config.pid_r[2].min)<0) state = 1;
		if (fscanf(f,"%f\t%f\t%f\n",&config.pid_r[0].max,&config.pid_r[1].max,&config.pid_r[2].max)<0) state = 1;
		if (fscanf(f,"%f\t%f\t%f\n",&config.pid_s[0].min,&config.pid_s[1].min,&config.pid_s[2].min)<0) state = 1;
		if (fscanf(f,"%f\t%f\t%f\n",&config.pid_s[0].max,&config.pid_s[1].max,&config.pid_s[2].max)<0) state = 1;
		if (fscanf(f,"%f\t%f\t%f\n",&config.pid_r[0].Kp,&config.pid_r[1].Kp,&config.pid_r[2].Kp)<0) state = 1;
		if (fscanf(f,"%f\t%f\t%f\n",&config.pid_r[0].Ki,&config.pid_r[1].Ki,&config.pid_r[2].Ki)<0) state = 1;
		if (fscanf(f,"%f\t%f\t%f\n",&config.pid_r[0].Kd,&config.pid_r[1].Kd,&config.pid_r[2].Kd)<0) state = 1;
		if (fscanf(f,"%f\t%f\t%f\n",&config.pid_s[0].Kp,&config.pid_s[1].Kp,&config.pid_s[2].Kp)<0) state = 1;
		if (fscanf(f,"%f\t%f\t%f\n",&config.pid_s[0].Ki,&config.pid_s[1].Ki,&config.pid_s[2].Ki)<0) state = 1;
		if (fscanf(f,"%f\t%f\t%f\n",&config.pid_s[0].Kd,&config.pid_s[1].Kd,&config.pid_s[2].Kd)<0) state = 1;

		fclose(f);

		fflush(NULL);
	}
	if (state) {
                printf("No config file or config syntax issue. New will be created!\n");
		for (int i=0;i<3;i++) {
			config.trim[i] = 0.0f;
			pid_init(&config.pid_r[i]);
			pid_init(&config.pid_s[i]);
			config.pid_s[i].min = -180;
			config.pid_s[i].max = 180;
			config.pid_r[i].min = -360;
			config.pid_r[i].max = 360;
		}

		//some default values if no config file
		config.esc_min = 1000;
		config.esc_max = 1900;

		config.pid_r[0].Kp=2.4f;  //yaw
		config.pid_r[1].Kp=0.865f;  //pitch
		config.pid_r[2].Kp=0.865f;  //roll
		config.pid_s[0].Kp=9.0f;  //yaw
		config.pid_s[1].Kp=2.5f;  //pitch
		config.pid_s[2].Kp=2.5f;  //roll

		config.pid_r[1].Ki=0.000f;
		config.pid_r[2].Ki=0.000f;
        }

	config.trim[0] = 0.0f;		
	config.trim[1] = 0.0f;		
	config.trim[2] = 0.0f;		


	printf("Config:\n");
	printf("ESC min (us): %i; max (us): %i;\n",config.esc_min,config.esc_max);
	printf("PID_R_YAW_Kp: %2.3f; PID_R_PITCH_Kp: %2.3f; PID_R_ROLL_Kp: %2.3f;\n",config.pid_r[0].Kp,config.pid_r[1].Kp,config.pid_r[2].Kp);
	printf("PID_R_YAW_Ki: %2.3f; PID_R_PITCH_Ki: %2.3f; PID_R_ROLL_Ki: %2.3f;\n",config.pid_r[0].Ki,config.pid_r[1].Ki,config.pid_r[2].Ki);
	printf("PID_R_YAW_Kd: %2.3f; PID_R_PITCH_Kd: %2.3f; PID_R_ROLL_Kd: %2.3f;\n",config.pid_r[0].Kd,config.pid_r[1].Kd,config.pid_r[2].Kd);
	printf("PID_S_YAW_Kp: %2.3f; PID_S_PITCH_Kp: %2.3f; PID_S_ROLL_Kp: %2.3f;\n",config.pid_s[0].Kp,config.pid_s[1].Kp,config.pid_s[2].Kp);
	printf("PID_S_YAW_Ki: %2.3f; PID_S_PITCH_Ki: %2.3f; PID_S_ROLL_Ki: %2.3f;\n",config.pid_s[0].Ki,config.pid_s[1].Ki,config.pid_s[2].Ki);
	printf("PID_S_YAW_Kd: %2.3f; PID_S_PITCH_Kd: %2.3f; PID_S_ROLL_Kd: %2.3f;\n",config.pid_s[0].Kd,config.pid_s[1].Kd,config.pid_s[2].Kd);

	return 0; 
}

int config_save() {
	f = fopen(p,"w");
        if (f == NULL) {
                printf("Error saving config file.\n");
		return -1;
        }

	fprintf(f,"%i\t%i\n",config.esc_min,config.esc_max);
	fprintf(f,"%f\t%f\t%f\n",config.pid_r[0].min,config.pid_r[1].min,config.pid_r[2].min);
	fprintf(f,"%f\t%f\t%f\n",config.pid_r[0].max,config.pid_r[1].max,config.pid_r[2].max);
	fprintf(f,"%f\t%f\t%f\n",config.pid_s[0].min,config.pid_s[1].min,config.pid_s[2].min);
	fprintf(f,"%f\t%f\t%f\n",config.pid_s[0].max,config.pid_s[1].max,config.pid_s[2].max);
	fprintf(f,"%f\t%f\t%f\n",config.pid_r[0].Kp,config.pid_r[1].Kp,config.pid_r[2].Kp);
	fprintf(f,"%f\t%f\t%f\n",config.pid_r[0].Ki,config.pid_r[1].Ki,config.pid_r[2].Ki);
	fprintf(f,"%f\t%f\t%f\n",config.pid_r[0].Kd,config.pid_r[1].Kd,config.pid_r[2].Kd);
	fprintf(f,"%f\t%f\t%f\n",config.pid_s[0].Kp,config.pid_s[1].Kp,config.pid_s[2].Kp);
	fprintf(f,"%f\t%f\t%f\n",config.pid_s[0].Ki,config.pid_s[1].Ki,config.pid_s[2].Ki);
	fprintf(f,"%f\t%f\t%f\n",config.pid_s[0].Kd,config.pid_s[1].Kd,config.pid_s[2].Kd);

	fflush(NULL);

	fclose(f);
	
	return 0;
}


