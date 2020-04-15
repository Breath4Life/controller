/** Volume measurement.
 * Proceeds by integration of the flow sensor.
 */
#ifndef VOLUME_H_
#define VOLUME_H_

#include <stdint.h>

/* @SENSOR_DIR Configures the flow sensor direction.
 * -1: flow sensor measures inspiration as a negative flow
 *  1: flow sensor measures inspiration as a positive flow
 */
#define SENSOR_DIR -1

/** @volume Current inspired volume in micro-liters
 * access this only with interrupts disabled
 */
//extern volatile int32_t volume;

// Instantaneous flow in standard ml/s
extern volatile int32_t flow;

/** @init_volume Initialize the volume subsystem, including the flow sensor.
 */
void init_volume();

/** @poll_volume This function should be regularly called, polls the flow
 * sensor and updates the volume.
 *
 * @return
 *  0 if the volume was updated
 *  1 if no new data is available
 *  2 if there was an error with the flow sensor (the sensor automatically recovers)
 */
uint8_t poll_volume();

/** @reset_volume Reset the current volume to 0.
 */
void reset_volume();

/** @get_volume Reads the current volume.
 *
 * @param vol: The current volume is written in this variable if there is no
 * error (unit: microliters).
 *
 * @return 0 if there is no error, 1 otherwise
 */
uint8_t get_volume(int32_t *vol);

uint8_t check_volume();

#endif // VOLUME_H_
