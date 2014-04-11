CPP=g++
LDFLAGS = -lm -lrt 
CFLAGS= -Wall -g -O2

all: controller.o 
	$(CPP) $(LDFLAGS) $(CFLAGS) -o controller \
		main.c \
		MotionSensor/libMotionSensor.a \
		libs/libI2Cdev.a

controller.o: MotionSensor/libMotionSensor.a libs/libI2Cdev.a

MotionSensor/libMotionSensor.a:
	$(MAKE) -C MotionSensor/ 

libs/libI2Cdev.a:
	$(MAKE) -C libs/I2Cdev

install:
	$(INSTALL) -m 755 controller $(DESTDIR)/usr/local/bin/

clean:
	cd MotionSensor && $(MAKE) clean
	cd libs/I2Cdev && $(MAKE) clean
	rm -rf *.o *~ *.mod
	rm -rf $(LIB)
