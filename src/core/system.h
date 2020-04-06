#ifndef SYSTEM_H_
#define SYSTEM_H_

// System-wide datastructures and non-FreeRTOS configuration

#include "FreeRTOS.h"
#include "task.h"

#define ALL_NOTIF_BITS 0xFFFFFFFF

#define WELCOME_MSG_DUR pdMS_TO_TICKS(2000L)

extern TaskHandle_t mainTaskHandle;
extern TaskHandle_t motorControlTaskHandle;
extern TaskHandle_t userInterfaceTaskHandle;
extern TaskHandle_t lcdDisplayTaskHandle;
extern TaskHandle_t analogReadTaskHandle;
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

extern volatile GlobalState_t globalState;

typedef enum {
    noAlarm,
    highPriorityAlarm,
    mediumPriorityAlarm,
    cricitalPriorityAlarm
} AlarmState_t;

extern volatile AlarmState_t alarmState;

typedef enum {
    noError,
    overPressure,
    noPressure,
    highPressure,
    highTemperature,
    lowPressure,
    abnVolume,
    abnFreq
} ErrorCode_t;

extern volatile ErrorCode_t errorCode;

extern volatile uint8_t mute_on;

#endif // SYSTEM_H_
