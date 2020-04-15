/** Task in charge of generating the buzzer sound.
 */
#include "FreeRTOS.h"

/** @init_alarm Initialize the state of the task.
 * Should be called before starting the real-time OS.
 */
void init_alarm();

/**
 * @AlarmsTask Buzzer sound generation handling task.
 *
 * @param pvParameters Set to NULL.
 */
void AlarmsTask(void *pvParameters);
