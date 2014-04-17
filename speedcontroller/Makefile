CXX=g++
AR=ar
CXXFLAGS=-Wall -g -O2
CXX_OPTS=-c -I../libs/ -DREVISION=2

SOURCES=blaster.c
OBJECTS=$(SOURCES:.c=.o)

LIB=libspeedcontroller.a

%.o: %.c
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 
	

$(LIB): $(OBJECTS) 
	$(AR) rcs $(LIB) $(OBJECTS) 

all: $(LIB)

clean:
	rm -rf *.o
	rm -rf $(LIB)
