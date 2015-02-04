
#include "PriorityQueue.h"
#include <stdlib.h>
#include <stdio.h>

// define the priority queue element
typedef struct PQElement {
	double priority;
	void* data;
	struct PQElement *next;
} PQElement;


// define priority queue structure that
// holds the head node and the size
struct PriorityQueue {
	unsigned int size;
	PQElement *head;
};


// create an empty priority queue
PriorityQueue* pq_create() {
	PriorityQueue *pq = (PriorityQueue *) malloc(sizeof(PriorityQueue));

	// create the head node
	pq->head = (PQElement *) malloc(sizeof(PQElement));

	pq->head->next = NULL;
	pq->head->data = NULL;
	pq->head->priority = -1;

	pq->size = 0;

	return pq;
}

// Push an element with a given priority into the queue
void pq_push(PriorityQueue *pq, double priority, void *data) {

	if (pq == NULL)
		return;

	PQElement *p = pq->head->next;
	PQElement *q = pq->head;

	// create a new element with the given data and priority
	PQElement *r = (PQElement *) malloc(sizeof(PQElement));

	r->priority = priority;
	r->data = data;

	// find the correct location in which to put the new element (r)
	for ( ; p != NULL; p=p->next, q=q->next) {
		// check if this is correct location
		if (priority < p->priority)
			break;
	}

	q->next = r;
	r->next = p;

	pq->size += 1;
}

// Pop an element from the priority queue
void* pq_pop(PriorityQueue *pq) {

	if (pq == NULL)
		return NULL;

	// check if queue is empty (i.e. head->next == NULL)
	if (pq->head->next == NULL)
		return NULL;

	// otherwise, remove and return the first element in the list
	PQElement *first = pq->head->next;

	pq->head->next = first->next;		// point head to the second element
	void* data = first->data;	// get the data of the first element

	free(first);				// free the "container" of the first element
	pq->size--;

	// return the data of the first element
	return data;
}


// get the size of the priority queue
unsigned int pq_size(PriorityQueue *pq) {
	if (pq == NULL)
		return 0;

	return pq->size;
}


// free the priority queue (with its elements)
void pq_free (PriorityQueue *pq) {
    if (pq == NULL)
        return;
    
    PQElement *e = pq->head->next;
    
    while (e != NULL) {
        PQElement *temp = e;
        e = e->next;
        free(temp);
    }
    
    free(pq->head);
    
    free(pq);
}