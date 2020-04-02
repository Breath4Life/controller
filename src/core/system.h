#ifndef SYSTEM_H_
#define SYSTEM_H_

// System-wide datastructures and non-FreeRTOS configuration

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t motorControlTaskHandle;
extern TaskHandle_t userInterfaceTaskHandle;
extern TaskHandle_t lcdDisplayTaskHandle;
extern TaskHandle_t alarmsTaskHandle;

#endif // SYSTEM_H_
