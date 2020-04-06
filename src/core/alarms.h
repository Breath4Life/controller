#include "FreeRTOS.h"

void init_alarm();

/**
 * @AlarmsTask Handle the alarms.
 *
 * @param pvParameters Set to NULL.
 */
void AlarmsTask(void *pvParameters);
