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

#define BPM_CHECK_PERIOD_MS 60 * 1000L
#define BPM_CHECK_PERIOD_PER_MIN 1
#define BPM_TOL 2

// MAIN wake-up notification
#define MAIN_NOTIF_ALARM 0x01

void initMainTask();
void MainTask(void *pvParameters);

/* check if volume is within +- 10% of target volume */
void check_volume(uint32_t actual_vol);

#endif // MAINTASK_H_
