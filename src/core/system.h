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

#define DEBUG_MOTOR (1 && DEBUG)
#define DEBUG_ALARM (1 && DEBUG)
#define DEBUG_MAIN (1 && DEBUG)
#define DEBUG_ANALOG_READ (1 && DEBUG)
#define DEBUG_BUZZER (0 && DEBUG)
#define DEBUG_DISPLAY (1 && DEBUG)
#define DEBUG_EEPROM (1 && DEBUG)
#define DEBUG_FLOW (1 && DEBUG)
#define DEBUG_LIM_SWITCH (1 && DEBUG)
#define DEBUG_MOTOR_LL (1 && DEBUG)
#define DEBUG_MEASURE_PEEP (1 && DEBUG)

// send parameters to plot graph
#define SEND_TO_SERIAL 0

#endif // SYSTEM_H_
