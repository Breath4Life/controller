/** Ring buffer used for buffering the debug output stream.
 */
#ifndef RING_BUF_H
#define RING_BUF_H

#include <stdbool.h>

// maximum buffer size
#define RBUF_SIZE 768

// buffer structure
typedef struct ring_buf_s
{
  unsigned char buf[RBUF_SIZE];
  int head;         // new data is written at this position in the buffer
  int tail;         // data is read from this position in the buffer
  int count;        // total number of elements in the queue <= RBUF_SIZE
} rbuf_t;

// ring buffer options
typedef enum
{
  RBUF_CLEAR,
  RBUF_NO_CLEAR
} rbuf_opt_e;

// buffer messages
typedef enum
{
  RBUF_EMPTY = -1,
  RBUF_FULL
} rbuf_msg_e;

// API

// initialise the queue
void ringbuf_init(rbuf_t* _this);

// determine if the queue is empty
bool ringbuf_empty(rbuf_t* _this);

// determine if the queue is full
bool ringbuf_full(rbuf_t* _this);

// fetch a byte from the queue at tail
int ringbuf_get(rbuf_t* _this);

// insert a byte to the queue at head
void ringbuf_put(rbuf_t* _this, const unsigned char item);

#endif
