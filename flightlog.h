#ifndef FLIGHTLOG_H
#define FLIGHTLOG_H

#include <stdarg.h>

//#define MAX_LOG 128000
#define MAX_LOG 6000
#define MAX_VALS 32 

struct s_flog {
	float v[MAX_VALS];
};

int flog_open(const char *path);

int flog_push(int n, ...);

int flog_save();


#endif

