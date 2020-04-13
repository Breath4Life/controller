#ifndef SFM3000_H_
#define SFM3000_H_

#include <stdint.h>

// Reading of the sfm3300 flow sensor, offset removed
extern volatile int16_t reading_sfm3300;

// Scale in [sensor reading unit]/(sl/min), given in datasheet
#define SCALE_SFM3000 120

// Initialize the sensor
// blocking=1: block until first valid measurement is done.
//  This enures proper valid value for flow.
// blocking=0: do not block, flow value might be invalid.
void sfm3000_init(uint8_t blocking);

uint8_t sfm3000_poll();
#endif // SFM3000_H_
