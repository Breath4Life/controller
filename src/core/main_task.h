#ifndef MAINTASK_H_
#define MAINTASK_H_

#include "FreeRTOS.h"
#include "task.h"

// tidal vol in tens of mL
#define DEFAULT_TIDAL_VOL 30
#define MAX_TIDAL_VOL 60
#define MIN_TIDAL_VOL 20
#define INC_TIDAL_VOL 2

#define DEFAULT_BPM 15
#define MAX_BPM 30
#define MIN_BPM 6
#define INC_BPM 2

#define ALARM_AUTO_UNMUTE_SEC 120

#define MAIN_NOTIF_TEMPERATURE 0x01
#define MAIN_NOTIF_OVERPRESSURE 0x02
#define MAIN_NOTIF_NOPRESSURE 0x04
#define MAIN_NOTIF_LOWPRESSURE 0x08
#define MAIN_NOTIF_PATIENT_CONNECTED 0x10

// in tens of ml
extern volatile uint8_t tidal_vol;
extern volatile uint8_t bpm;
extern volatile uint8_t p_max;

void initMainTask();
void MainTask(void *pvParameters);
/* returns 1 if the global state is stop or run, otherwise 0 */
uint8_t stoppedOrRunning();

#endif // MAINTASK_H_
