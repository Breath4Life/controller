#ifndef SFM3000_H_
#define SFM3000_H_

#include <stdint.h>

// in standard ml/s
extern volatile int32_t flow;

// Initialize the sensor
// blocking=1: block until first valid measurement is done.
//  This enures proper valid value for flow.
// blocking=0: do not block, flow value might be invalid.
void sfm3000_init(uint8_t blocking);

uint8_t sfm3000_poll();
#endif // SFM3000_H_
