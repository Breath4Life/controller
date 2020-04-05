#ifndef SYSTEM_H_
#define SYSTEM_H_

// System-wide datastructures and non-FreeRTOS configuration

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t mainTaskHandle;
extern TaskHandle_t motorControlTaskHandle;
extern TaskHandle_t userInterfaceTaskHandle;
extern TaskHandle_t lcdDisplayTaskHandle;
extern TaskHandle_t alarmsTaskHandle;
extern TaskHandle_t sfm3000TaskHandle;

typedef enum {
    welcome,
    welcome_wait_cal,
    calibration,
    stop,
    run,
    critical_failure
} GlobalState_t;

extern GlobalState_t globalState;

typedef enum {
    noAlarm
} AlarmState_t;

extern AlarmState_t alarmState;

typedef enum {
    noError
} ErrorCode_t;

extern ErrorCode_t errorCode;

#endif // SYSTEM_H_
