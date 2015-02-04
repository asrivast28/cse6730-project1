
#ifndef PRIORITYQUEUE_H_
#define PRIORITYQUEUE_H_


// Declare type of priority queue
typedef struct PriorityQueue PriorityQueue;

/*
 * Create an empty priority queue.
 *
 * @return a pointer to the priority queue.
 */
PriorityQueue* pq_create();

/*
 * Push an element with a given priority into the queue.
 *
 * @param pq the priority queue
 * @param priority the element's priority
 * @param data the data representing our element
 */
void pq_push(PriorityQueue *pq, double priority, void *data);


/*
 * Pop the top element from the priority queue.
 *
 * @param pq the priority queue
 *
 * @return the data representing the top element
 */
void* pq_pop(PriorityQueue *pq);


/*
 * Get the size of the priority queue
 *
 * @param pq the priority queue
 *
 * @return the size of the queue
 */
unsigned int pq_size(PriorityQueue *pq);


/*
 * Free the memory allocated to the priority queue.
 *
 * @param pq the priority queue
 */
void pq_free(PriorityQueue *pq);



#endif /* PRIORITYQUEUE_H_ */
