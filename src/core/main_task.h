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

// in tens of ml
extern uint8_t tidal_vol;
extern uint8_t bpm;

void initMainTask();
void MainTask(void *pvParameters);

#endif // MAINTASK_H_
