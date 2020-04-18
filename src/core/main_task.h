/** Main task, in charge of
 * - maintaining the global state of the system
 * - maintaining the parameters state
 * - reading the state of the buttons
 * - reading the state of the door open sensor
 * - reading the state of the power supply subsystem
 * - polling the volume subsystem
 * - managing the alarms (triggering, acknowledging and muting)
 *  - TODO other stuff...
 */
#ifndef MAINTASK_H_
#define MAINTASK_H_

#include "FreeRTOS.h"
#include "task.h"

#define ALARM_AUTO_UNMUTE_DELAY 120 * 1000L

// Alarm notification values
#define MAIN_NOTIF_ALARM 0x01

void initMainTask();
void MainTask(void *pvParameters);
/* returns 1 if the global state is stop or run, otherwise 0 */
uint8_t stoppedOrRunning();
/* check if volume is within +- 10% of target volume */
void check_volume(uint32_t actual_vol);

void incrementCyclesNumber();

#endif // MAINTASK_H_
