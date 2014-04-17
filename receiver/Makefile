CXX=g++
AR=ar
CXXFLAGS=-Wall -g -O2
CXX_OPTS=-c -DMPU6050 -DMPU_DEBUG -I../libs/

SOURCES=ps3controller.c
OBJECTS=$(SOURCES:.c=.o)

LIB=libreceiver.a

%.o: %.c
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 
	

$(LIB): $(OBJECTS) 
	$(AR) rcs $(LIB) $(OBJECTS) 

all: $(LIB)

clean:
	rm -rf *.o
	rm -rf $(LIB)
