#ifndef PS3CONTROLLER_H
#define PS3CONTROLLER_H

#include "interface.h"

struct s_rec rec = {
	.yprt = {0.0f,0.0f,0.0f,0.0f}
};

int rec_open();
int rec_update();
int rec_close();

#endif

