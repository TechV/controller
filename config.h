#ifndef QCONFIG_H
#define QCONFIG_H

#include "pid.h"

struct s_config {
	int esc_min, esc_max;
	struct s_pid pid_r[3];
	struct s_pid pid_s[3];
	float trim[3];
};

extern struct s_config config;

int config_open(const char *path);

int config_save();


#endif

