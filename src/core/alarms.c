#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/tone.h"

void AlarmsTask(void *pvParameters)
{
    tone_init();
    while (1)
    {
        // DEMO
        tone_start(262);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        tone_stop();
        vTaskDelay(120 / portTICK_PERIOD_MS);

        tone_start(440);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        tone_stop();
        vTaskDelay(120 / portTICK_PERIOD_MS);

        tone_start(349);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        tone_stop();
        vTaskDelay(400 / portTICK_PERIOD_MS);

        tone_start(440);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        tone_stop();
        vTaskDelay(120 / portTICK_PERIOD_MS);

        tone_start(349);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        tone_stop();

        //TODO: write this function!
        vTaskDelay(2000 / portTICK_PERIOD_MS); // sleep 100ms
    }
}
