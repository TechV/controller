CXX=g++
CXXFLAGS= -Wall -g -O2
CXX_OPTS= -Wall -g -O2 $(CXXFLAGS_FOR_BUILD)
LDFLAGS=-lpthread -pthread
LD_OPTS=-lpthread -lxenomai -lrt -lnative

PROG=controller

INSTALL=install

%.o: %.c                                                                         
	$(CXX) -c $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 


all: controller.o 
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $(LD_OPTS) -o $(PROG) \
		main.c \
		pid.o \
		flightlog.o \
		config.o \
		motionsensor/libmotionsensor.a \
		receiver/libreceiver.a \
		speedcontroller/libspeedcontroller.a \
		bmpsensor/libbs.a \
		libs/libi2cdev.a

controller.o: pid.o flightlog.o config.o motionsensor/libmotionsensor.a receiver/libreceiver.a speedcontroller/libspeedcontroller.a bmpsensor/libbs.a libs/libi2cdev.a

speedcontroller/libspeedcontroller.a:
	$(MAKE) -C speedcontroller/ 
	
motionsensor/libmotionsensor.a:
	$(MAKE) -C motionsensor/ 

receiver/libreceiver.a:
	$(MAKE) -C receiver/ 

bmpsensor/libbs.a:
	$(MAKE) -C bmpsensor/ 

libs/libi2cdev.a:
	$(MAKE) -C libs/i2cdev

install:
        $(INSTALL) -d $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 controller $(DESTDIR)/usr/local/bin/

clean:
	cd motionsensor && $(MAKE) clean
	cd receiver && $(MAKE) clean
	cd speedcontroller && $(MAKE) clean
	cd bmpsensor && $(MAKE) clean
	cd libs/i2cdev && $(MAKE) clean
	rm -rf *.o *~ *.mod
	rm -rf $(PROG)
