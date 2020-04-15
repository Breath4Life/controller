/** Time-keeping for the system
 */
#ifndef TIME_H_
#define TIME_H_

#include <stdint.h>

/** Initialize the time-keeping system.
 */
void init_time();

/** Return the current time in us.
 */
uint32_t time_us();
#endif // TIME_H_
