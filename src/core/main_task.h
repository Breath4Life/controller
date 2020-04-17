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

// tidal vol in tens of mL
#define DEFAULT_TIDAL_VOL 30
#define MAX_TIDAL_VOL 60
#define MIN_TIDAL_VOL 20
#define INC_TIDAL_VOL 2

// breath per min
#define DEFAULT_BPM 15
#define MAX_BPM 30
#define MIN_BPM 12
#define INC_BPM 1

// I:E ratio
#define DEFAULT_IE 1
#define MAX_IE 3
#define MIN_IE 1
#define INC_IE 1

// Pmax
#define DEFAULT_PMAX 38
#define MAX_PMAX 58
#define MIN_PMAX 28
#define INC_PMAX 1

#define N_EXTRA 3

#define ALARM_AUTO_UNMUTE_SEC 120 * 1000L

// Alarm notification values
#define MAIN_NOTIF_ALARM 0x01

// in tens of ml
extern uint8_t tidal_vol;
extern uint8_t bpm;
extern uint8_t ie;
extern uint8_t p_max;
extern uint8_t extra_param;

void initMainTask();
void MainTask(void *pvParameters);
/* returns 1 if the global state is stop or run, otherwise 0 */
uint8_t stoppedOrRunning();
/* check if volume is within +- 10% of target volume */
void check_volume(uint32_t actual_vol);

void incrementCyclesNumber();

#endif // MAINTASK_H_
