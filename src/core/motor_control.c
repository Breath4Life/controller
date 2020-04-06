#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/motor.h"
#include "core/debug.h"
#include "core/motor_control.h"
#include "hal/limit_switch.h"

void init_motor() {
    init_limit_switch();
    setup_motor();
}

MotorState_t motorState;

void MotorControlTask(void *pvParameters)
{
    while (1)
    {
        //TODO: write this function!
        vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
        debug_print("Start moving +\r\n");
        set_motor_goto_position_accel_exec(16*200L, 16*80L, 2, 200);
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        while (motor_moving()) {
            vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
            debug_print("moving + pos: %i\r\n", motor_remaining_distance());
        }
        debug_print("Finished moving +\r\n");
        vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
        debug_print("Start moving -\r\n");
        set_motor_goto_position_accel_exec(0, 16*80L, 2, 200);
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        while (motor_moving()) {
            vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
        }
        debug_print("Finished moving -\r\n");
    }
}

