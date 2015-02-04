CC=gcc
#CCFLAGS=-Wall -g
# activate for compiler optimizations:
CCFLAGS=-Wall -O3
LDFLAGS=
 
all: traffic_simulator

traffic_simulator: FIFOQueue.o PriorityQueue.o simulation.o simulationEngine.o
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c %.h
	$(CC) $< $(CCFLAGS) -c

%.o: %.c
	$(CC) $< $(CCFLAGS) -c

clean:
	rm -f *.o traffic_simulator 
