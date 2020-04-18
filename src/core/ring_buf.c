#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ring_buf.h"

// advance the ring buffer index
static unsigned int ringbuf_adv (const unsigned int value, const unsigned int max_val);

/* Ring bufffer / Circular buffer / Circular Queue implementation
 * The queue grows from the head and shrinks from the tail
 */

void ringbuf_init(rbuf_t* _this)
{
  // clear the _thisfer and init the values
  // and sets head = tail in one go
  memset( _this, 0, sizeof(*_this) );
}

bool ringbuf_empty(rbuf_t* _this)
{
  // test if the queue is empty
  // 0 returns true
  // nonzero false
  return (0 == _this->count);
}

bool ringbuf_full(rbuf_t* _this)
{
  // full when no of elements exceed the max size
  return (_this->count >= RBUF_SIZE);
}

int ringbuf_get(rbuf_t* _this)
{
  int item;
  if (_this->count > 0)
  {
    // get item element
    item        = _this->buf[_this->tail];
    // advance the tail
    _this->tail = ringbuf_adv(_this->tail, RBUF_SIZE);
    // reduce the total count
    --_this->count;
  }
  else {
    // the queue is empty
    item = RBUF_EMPTY;
  }
  return item;
}

static void put_nocheck(rbuf_t* _this, const unsigned char item) {
    // set the item at head position
    _this->buf[_this->head] = item;
    // advance the head
    _this->head = ringbuf_adv(_this->head, RBUF_SIZE);
    // increase the total count
    ++_this->count;
}

void ringbuf_put(rbuf_t* _this, const unsigned char item)
{
  if (_this->count < RBUF_SIZE-3)
  {
      put_nocheck(_this, item);
  } else if (_this->count < RBUF_SIZE-2) {
      put_nocheck(_this, 'B');
      put_nocheck(_this, 'U');
      put_nocheck(_this, 'F');
  }
}

static unsigned int ringbuf_adv(const unsigned int value, const unsigned int max)
{
  unsigned int index = value + 1;
  if (index >= max)
  {
    index = 0;
  }
  return index;
}
