CC=g++
AR=ar
CFLAGS= -Wall -g -O2
LDFLAGS= -lm -lrt

LIB=../libI2Cdev.a
OBJ=I2Cdev.o

$(LIB): $(OBJ)
	$(AR) rcs $(LIB) $(OBJ)

clean:
	rm -rf *.o
	rm -rf $(LIB)
