
#ifndef FIFOQUEUE_H_
#define FIFOQUEUE_H_

typedef struct FIFOQueue FIFOQueue;



/*
 * Create an empty fifo queue.
 *
 * @return a pointer to the fifo queue.
 */
FIFOQueue* fifo_create();


/*
 * Push an element into the fifo queue.
 *
 * @param ffq the fifo queue
 * @param data the data representing our element
 */
void fifo_push(FIFOQueue *ffq, void *data);


/*
 * Pop the first element from the fifo queue.
 *
 * @param ffq the fifo queue
 *
 * @return the data representing the first element
 */
void* fifo_pop(FIFOQueue *ffq);


/*
 * Get the size of the fifo queue
 *
 * @param ffq the fifo queue
 *
 * @return the size of the queue
 */
unsigned int fifo_size(FIFOQueue *ffq);



#endif /* FIFOQUEUE_H_ */
