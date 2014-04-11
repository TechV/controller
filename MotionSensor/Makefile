CC=g++
AR=ar
CFLAGS= -Wall -g -O2
CFLAGS+= -DMPU6050 -DMPU_DEBUG -I../libs/
LDFLAGS= -lm -lrt 

LIB=libMotionSensor.a

OBJ=mpu6050.o inv_mpu_lib/inv_mpu.o inv_mpu_lib/inv_mpu_dmp_motion_driver.o

$(LIB): $(OBJ) 
	$(AR) rcs $(LIB) $(OBJ)

clean:
	rm -rf *.o
	rm -rf inv_mpu_lib/*.o
	rm -rf $(LIB)
