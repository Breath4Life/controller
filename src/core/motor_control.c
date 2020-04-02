#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/motor.h"
#include "core/debug.h"

void MotorControlTask(void *pvParameters)
{
//    dio_init(DIO_PIN_STEP_COUNTER_TN, DIO_INPUT);
//    dio_init(DIO_PIN_MOTOR_STEP, DIO_OUTPUT);
//    dio_init(DIO_PIN_MOTOR_ENABLE, DIO_OUTPUT);
//    dio_init(DIO_PIN_MOTOR_DIRECTION, DIO_OUTPUT);
//    while (1) {
//        vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
//        dio_write(DIO_PIN_MOTOR_STEP, DIO_LOW);
//        dio_write(DIO_PIN_MOTOR_ENABLE, DIO_LOW);
//        dio_write(DIO_PIN_MOTOR_DIRECTION, DIO_LOW);
//        vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
//        dio_write(DIO_PIN_MOTOR_STEP, DIO_HIGH);
//        dio_write(DIO_PIN_MOTOR_ENABLE, DIO_HIGH);
//        dio_write(DIO_PIN_MOTOR_DIRECTION, DIO_HIGH);
//    }
    while (1)
    {
        //TODO: write this function!
        vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
        debug_print("Start moving +\r\n");
        set_motor_goto_position(16*200L, 16*200L);
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        while (motor_moving()) {
            vTaskDelay(1000 / portTICK_PERIOD_MS); // sleep 100ms
            debug_print("moving + pos: %i\r\n", motor_remaining_distance());
        }
        debug_print("Finished moving +\r\n");
        vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
        debug_print("Start moving -\r\n");
        set_motor_goto_position(0, 16*200L);
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        while (motor_moving()) {
            vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
        }
        debug_print("Finished moving -\r\n");
    }
}

