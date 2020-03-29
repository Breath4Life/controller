#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "hal/pins.h"

void LCDDisplayTask(void *pvParameters)
{
    while (1)
    {
	//TODO: write this function!
	vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
    }
}

