#ifndef VOLUME_H_
#define VOLUME_H_

#include <stdint.h>


/**
 * @volume Current inspired volume in micro-liters
 * access this only with interrupts disabled
 */
//extern volatile int32_t volume;

// Instantaneous flow in standard ml/s
extern volatile int32_t flow;

void init_volume();

// Return 0 if volume updated, otherwise return 1 in non-error cases, and other values for error cases.
uint8_t poll_volume();

void reset_volume();

uint8_t get_volume(int32_t *vol);

#endif // VOLUME_H_
