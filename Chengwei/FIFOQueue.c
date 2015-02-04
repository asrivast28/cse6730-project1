


#include "FIFOQueue.h"
#include <stdlib.h>

// define the fifo queue element
typedef struct FIFOElement {
	void* data;
	struct FIFOElement *next;
} FIFOElement;

// define the fifo queue
struct FIFOQueue {
	unsigned int size;
	FIFOElement *front;     // points to the front of the queue
	FIFOElement *end;       // points to the end of the queue
};


// create an empty FIFO queue
FIFOQueue* fifo_create() {
	FIFOQueue *ffq = (FIFOQueue *) malloc(sizeof(FIFOQueue));

	ffq->front = NULL;
	ffq->end = NULL;
	ffq->size = 0;

	return ffq;
}


// Push an element into the queue
void fifo_push(FIFOQueue *ffq, void *data) {
    
    if (ffq == NULL)
        return;
    
	FIFOElement *e = (FIFOElement *) malloc(sizeof(FIFOElement));

	e->data = data;
	e->next = NULL;

	if (ffq->size == 0) {
		ffq->end = e;
		ffq->front = e;
	}
	else {
		ffq->end->next = e;
		ffq->end = ffq->end->next;
	}

	ffq->size++;
}

// Pop an element from the queue
void* fifo_pop(FIFOQueue *ffq) {
	if (ffq->size == 0)
		return NULL;

	FIFOElement *temp = ffq->front;
	ffq->front = ffq->front->next;

	void *data = temp->data;
	free(temp);

	ffq->size--;

	return data;
}


// returns 1 if queue is empty; 0 otherwise
unsigned int fifo_size(FIFOQueue *ffq) {
	return ffq->size;
}


