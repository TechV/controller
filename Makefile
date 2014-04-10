CC?=gcc
LIBS = -lm -lrt 

OPT_Debug= -Wall -g -O2

#CXXFLAGS = -DDMP_FIFO_RATE=1

ALL:$(EXEC)

$(EXEC):$(OBJ)
	$(CC) $(CXXFLAGS) $(OPT_Debug) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(CXXFLAGS) $(OPT_Debug) -o $@ -c $^ $(LIBS)

clean:
	rm -rf *.o *~ *.mod
	rm -rf $(EXEC)
