#ifndef __QUEUE_H
#define __QUEUE_H
#define QUEUE_SIZE 200

struct Queue {
  int pRD, pWR;
  uint8_t q[QUEUE_SIZE];
};


int Enqueue(struct Queue *q, uint8_t data);
int Dequeue(struct Queue *q, uint8_t *data);
void InitQueue(struct Queue *q);
#endif
