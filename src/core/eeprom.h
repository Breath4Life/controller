/** EEPROM management
 */
#ifndef EEPROM_H_
#define EEPROM_H_

#define WRITE_EEPROM_PERIOD_MS 5 * 1000L // Try 5 * 1000L for Testing purpose

#include <stdbool.h>
#include <stdint.h>

#include "motor_control.h"

/** @init_eeprom Initialize the EEPROM task
 *
 * @return true if this is the first boot, false otherwise
 */
bool init_eeprom();

/* Should be call after the init.
 */
void startEeprom();

/** @eepromTask EEPROM writing of running time and number of cycles
 */
void pollEeprom();

uint32_t total_operating_time;

// TODO implement this
/** @GET_CYCLE_COUNT Function that returns the motor cycle count
 */
#define GET_CYCLE_COUNT() (cycleCount)

// TODO implement this
/** @SET_CYCLE_COUNT Function that sets the current cycle count
 */
#define SET_CYCLE_COUNT(x) do { cycleCount = (x); } while (0)
#endif // EEPROM_H_
