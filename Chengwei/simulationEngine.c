//#include <boost/heap/priority_queue.hpp>
#include <stdlib.h>
#include "simulationEngine.h"
#include "PriorityQueue.h"

//using namespace boost::heap;


// declare simulation time
double simtime = 0;
PriorityQueue *FEL = NULL;      // Future Event List

// define container for simulations events
typedef struct SimEvent {
	double timestamp;					// event time stamp
	void(*callback) (void *e);		// handler callback
	void *data;						// application data
} SimEvent;


// schedule the event at time-stamp, and provide a callback to its handler
void schedule(double timestamp, void* eventData, void(*callback) (void* e)) {
	SimEvent *se = (SimEvent *)malloc(sizeof(SimEvent));

	se->callback = callback;
	se->data = eventData;
	se->timestamp = timestamp;

	// if we have not yet initialize a priority queue, create one
	if (FEL == NULL)
		FEL = pq_create();

	// pass the simulation event into the queue with priority equal to timestamp
	pq_push(FEL, timestamp, se);
}


// returns the current simulation time
double current_time() {
	return simtime;
}


// run the simulation
void run_sim() {

	if (FEL == NULL)
		return;

	// as long as the priority queue is not empty
	// remove its top element, and execute its callback
	// on the event data
	while (pq_size(FEL) > 0) {
		SimEvent *se = (SimEvent *)pq_pop(FEL);
		simtime = se->timestamp;
		se->callback(se->data);
		free(se);
	}
	// free the pq
	pq_free(FEL);
}
