#ifndef SYSTEM_H_
#define SYSTEM_H_

// System-wide datastructures and non-FreeRTOS configuration

#include "FreeRTOS.h"
#include "task.h"

#define MOCK_VOLUME_SENSOR 1

#define ALL_NOTIF_BITS 0xFFFFFFFF

#define WELCOME_MSG_DUR pdMS_TO_TICKS(2000L)

extern TaskHandle_t mainTaskHandle;
extern TaskHandle_t motorControlTaskHandle;
extern TaskHandle_t buzzerTaskHandle;
extern TaskHandle_t lcdDisplayTaskHandle;
extern TaskHandle_t analogReadTaskHandle;
extern TaskHandle_t eepromTaskHandle;

typedef enum {
    welcome,
    welcome_wait_cal,
    calibration,
    stop,
    run,
    critical_failure
} GlobalState_t;

extern volatile GlobalState_t globalState;

#endif // SYSTEM_H_
