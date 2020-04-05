
#include "FreeRTOS.h"
#include "task.h"

#include "core/debug.h"
#include "core/system.h"
#include "core/buttons.h"

#include "hal/io.h"
#include "hal/pins.h"

GlobalState_t globalState;
AlarmState_t alarmState;
ErrorCode_t errorCode;

void initMainTask()
{
    globalState = welcome;
    alarmState = noAlarm;
    errorCode = noError;
    initButtons();
}

void MainTask(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // TODO notify Buzzer task of Welcome
    while (1) {
        ButtonsState buttons_pressed = poll_buttons();
        //debug_print("polling buttons %u\r\n", buttons_pressed);
        for (uint8_t i=0; i < N_BUTTONS; i++) {
            if (BUTTON_PRESSED(buttons_pressed, i)) {
                debug_print("Button %s pressed\r\n", buttons_descr[i]);
            }
        }
        vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
}
