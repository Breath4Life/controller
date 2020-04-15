/** Interface with the SFM3300 flow sensor.
 *
 * The interface is poll-based: the sfm3000_poll shoud be called frequently and
 * cyclically reads the sensor. A new reading from the sensor is available in
 * the reading_sfm3300 variable every two calls to the function, approximately.
 */
#ifndef SFM3000_H_
#define SFM3000_H_

#include <stdint.h>

/** @reading_sfm3300 The last reading of the sensor, expressed in sensor units.
 * This value should be divided by SCALE_SFM3000 to get the value of the
 * reading in standard liter per minute.
 */
extern volatile int16_t reading_sfm3300;

// Scale in [sensor reading unit]/(sl/min), given in the datasheet
#define SCALE_SFM3000 120

/** @sfm3000_init Initialize the sensor.
 *
 * @param blocking if blocking==1, the initialization performs a first
 * measurement, ensuring the first next measurement is correct.
 * This blocks for about 1 ms.
 * If blocking==0, then the first reading may be incorrect.
 */
void sfm3000_init(uint8_t blocking);

/** @sfm3000_poll Poll the sensor. Updates the reading_sfm3300 variable if new
 * reading is available.
 *
 * @return
 *  0 if a new reading is available
 *  1 if no new reading is available
 *  another value if there was an error with the reading.
 *  Recovery from errors is automatic.
 */
uint8_t sfm3000_poll();

#endif // SFM3000_H_
