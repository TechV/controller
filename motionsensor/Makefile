CXX=g++
AR=ar
CXXFLAGS=-Wall -g -O2
CXX_OPTS=-c -DMPU9150 -DMPU_DEBUG1 -I../libs/

SOURCES=mpu6050.c inv_mpu_lib/inv_mpu.c inv_mpu_lib/inv_mpu_dmp_motion_driver.c
OBJECTS=$(SOURCES:.c=.o)

LIB=libmotionsensor.a

%.o: %.c
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 
	

$(LIB): $(OBJECTS) 
	$(AR) rcs $(LIB) $(OBJECTS) 

all: $(LIB)

clean:
	rm -rf *.o
	rm -rf inv_mpu_lib/*.o
	rm -rf $(LIB)
