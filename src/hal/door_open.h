#ifndef DOOR_OPEN_H_H
#define DOOR_OPEN_H_H

#include <stdbool.h>

/* @door_open_init Initialize the door open switch
 */
void door_open_init();

/* @is_door_open Return true if the door is open
 */
bool is_door_open();

#endif // DOOR_OPEN_H_H
