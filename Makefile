CXX=g++
CXXFLAGS= -Wall -g -O2
CXX_OPTS= -Wall -g -O2

INSTALL=install

%.o: %.c                                                                         
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 


all: controller.o 
	$(CXX) $(LDFLAGS) $(CXXFLAGS) -o controller \
		main.c \
		receiver/libreceiver.a \
		libs/libI2Cdev.a

controller.o: motionsensor/libmotionsensor.a receiver/libreceiver.a libs/libi2cdev.a

motionsensor/libmotionsensor.a:
	$(MAKE) -C motionsensor/ 

receiver/libreceiver.a:
	$(MAKE) -C receiver/ 

libs/libi2cdev.a:
	$(MAKE) -C libs/i2cdev

install:
	$(INSTALL) -m 755 controller $(DESTDIR)/usr/local/bin/

clean:
	cd motionsensor && $(MAKE) clean
	cd receiver && $(MAKE) clean
	cd libs/i2cdev && $(MAKE) clean
	rm -rf *.o *~ *.mod
	rm -rf $(LIB)
