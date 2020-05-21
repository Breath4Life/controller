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

// DEBUG

#ifndef DEBUG
#define DEBUG 1
#endif // DEBUG

#define DEBUG_MOTOR (0 && DEBUG)
#define DEBUG_ALARM (0 && DEBUG)
#define DEBUG_MAIN (0 && DEBUG)
#define DEBUG_ANALOG_READ (1 && DEBUG)
#define DEBUG_BUZZER (0 && DEBUG)
#define DEBUG_DISPLAY (0 && DEBUG)
#define DEBUG_EEPROM (0 && DEBUG)
#define DEBUG_FLOW (0 && DEBUG)
#define DEBUG_LIM_SWITCH (0 && DEBUG)
#define DEBUG_MEASURE_PEEP (1 && DEBUG)

#endif // SYSTEM_H_
