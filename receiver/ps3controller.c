#include <linux/joystick.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <error.h>

#include "ps3controller.h"
#include "../receiver.h"


struct s_rec rec;

static struct js_event js_e;
static int fd = 0;
static int ret;
static int c;

int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int rec_open() {
	fd = open ("/dev/input/js0", O_RDONLY | O_NONBLOCK);	
	if (fd < 0) {
		printf("can't open js0: [%i] [%s]\n", fd, strerror(errno));
		return -1;
	}
	return rec_update();
}

void process_jsevent(struct js_event *e) {
	if (e->type==JS_EVENT_INIT) {
		printf("JS buffer full?\n");	
		return;
	}

	if (e->type==JS_EVENT_BUTTON) {
		printf("B %2u VAL: %4i\n",e->number,e->value);
	}
	if (e->type==JS_EVENT_AXIS) {
		printf("A %2u VAL: %4i\n",e->number,e->value);
		rec.yprt[0] = map(e->value,-255,255,-45,45);
		rec.yprt[1] = map(e->value,-255,255,-45,45);
		rec.yprt[2] = map(e->value,-255,255,-45,45);
		rec.yprt[3] = map(e->value,-255,255,0,100);
	}
	
}

int rec_update() {
        pthread_mutex_lock( &rec.mutex );
	
	c = 0;
	do { //read all event one by one
		ret = read (fd, &js_e, sizeof(js_e));
		process_jsevent(&js_e);
		c++;
	} while (ret > 0);
	
	if (errno != EAGAIN) {
		printf("Error reading js0: [%i] [%s]\n",errno,strerror(errno));
		return -1;
	}

        clock_gettime(CLOCK_REALTIME, &rec.t);

	rec.ypr[0] = 0;
	rec.ypr[1] = 0;
	rec.ypr[2] = 0;

        pthread_mutex_unlock( &rec.mutex );
	return 0;
}

int rec_close() {
	close(fd);
	return 0;
}

