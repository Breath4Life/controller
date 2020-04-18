/** EEPROM management
 */
#ifndef EEPROM_H_
#define EEPROM_H_

#define WRITE_EEPROM_PERIOD_MS 300 * 1000L // Try 5 * 1000L for Testing purpose

#include <stdbool.h>
#include <stdint.h>

/** @init_eeprom Initialize the EEPROM task
 *
 * @return true if this is the first boot, false otherwise
 */
bool init_eeprom();

/** @eepromTask EEPROM writing of running time and number of cycles
 */
void eepromTask(void *pvParameters);

// TODO implement this
/** @GET_CYCLE_COUNT Function that returns the motor cycle count
 */
#define GET_CYCLE_COUNT() (0)

// TODO implement this
/** @SET_CYCLE_COUNT Function that sets the current cycle count
 */
#define SET_CYCLE_COUNT(x) do {} while (0)
#endif // EEPROM_H_
