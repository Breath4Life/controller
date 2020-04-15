/** Reading of error signals coming from the power interface board.
 *
 * There are two signals, both signal an error when they are low.
 * power_aux signal: signals that the backup battery is discharged or missing
 * power_main signal: signals that the main supply is not working, hence the
 * device is not working.
 */
#ifndef POWER_MONITORING_H_
#define POWER_MONITORING_H_

#include <stdbool.h>

/** @init_power_monitoring Initialize the pins for power monitoring.
 */
void init_power_monitoring();

/** @error_power_aux Returns true if there is an error on the auxiliary power
 * supply, false otherwise.
 */
bool error_power_aux();

/** @error_power_main Returns true if there is an error on the main power
 * supply, false otherwise.
 */
bool error_power_main();

#endif // POWER_MONITORING_H_
