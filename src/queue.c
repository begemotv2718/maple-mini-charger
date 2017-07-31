#include <stdint.h>
#include "queue.h"
#include <string.h>

static inline int QueueFull(struct Queue *q){
  return  (q->pWR+1)%QUEUE_SIZE == q->pRD;
}
static inline int QueueEmpty(struct Queue *q){
  return  q->pWR == q->pRD;
}

int Enqueue(struct Queue *q, uint8_t data){
  if(QueueFull(q)){
    return 0;
  }
  q->q[q->pWR]=data;
  /*Note interrupting this here with another Enqueue will be disastrous!!!! Use cli*/
  q->pWR=(q->pWR>=(QUEUE_SIZE-1))?0:q->pWR+1;
  return 1;
}

int Dequeue(struct Queue *q, uint8_t *data){
  if(QueueEmpty(q)){
    return 0;
  }
  *data=q->q[q->pRD];
  q->pRD =  (q->pRD>= (QUEUE_SIZE -1))? 0: q->pRD+1;
  return 1;
}

void InitQueue(struct Queue *q){
  memset(q,0,sizeof(struct Queue));
}
