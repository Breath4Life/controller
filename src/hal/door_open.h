#ifndef DOOR_OPEN_H_H
#define DOOR_OPEN_H_H

#include <stdbool.h>

/** HAL for the door opening detection switch.
 *
 * The switch is closed when the door is closed, and open when the door is
 * open. The switch is connected to the Ground and to a DIO pin configured as
 * input with internal Pull-Up.
 */

/** @door_open_init Initialize the door open switch
 */
void door_open_init();

/** @is_door_open Return true if the door is open, false otherwise
 */
bool is_door_open();

#endif // DOOR_OPEN_H_H
