/** The limit switches are used to detect the end of course of the
 * bag-compressing lever.
 * There are two limit switches: one corresponding to the lever Up position and
 * one corresponding to the lever Down position.
 * The switches are closed when the lever is not touching them and open if they
 * are touched. They are connected to the Ground and to pins configured as
 * input with internal pull-up resistors.
 *
 * An interrupt is raised when one of the switch states is changed, which in
 * turn notifies the SWITCH_HANDLING_TASK with the LIM_SWITCH_UP and
 * LIM_SWITCH_DOWN notifications when the switches are pushed down.
 */

#ifndef LIMIT_SWITCH_H_
#define LIMIT_SWITCH_H_

#include "core/system.h"
#include "core/motor_control.h"

#define SWITCH_HANDLING_TASK motorControlTaskHandle
#define LIM_SWITCH_UP MOTOR_NOTIF_LIM_UP
#define LIM_SWITCH_DOWN MOTOR_NOTIF_LIM_DOWN

/** @init_limit_switch Initialize the limit switch handling
 * Sets up the pins config and the ISR.
 */
void init_limit_switch();

uint8_t get_lim_down_v();
uint8_t get_lim_up_v();

#endif // LIMIT_SWITCH_H_
