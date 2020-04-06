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
    uint16_t msteps = 1;
    uint32_t target_position_abs = 600 * msteps;
    uint16_t target_speed_insp = 1200*msteps;
    uint16_t target_speed_exp = 1200*msteps;
    uint16_t step_num_base = 8;
    uint16_t step_freq_base = 200*msteps;

    vTaskDelay(500 / portTICK_PERIOD_MS); // sleep 100ms
    while (1) {
        debug_print("Start INSP\r\n");
        motor_enable();
        set_motor_goto_position_accel_exec(target_position_abs, target_speed_insp, step_num_base, step_freq_base);
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        debug_print("Finished INSP %u\r\n", motor_inmotion);
        vTaskDelay(70 / portTICK_PERIOD_MS); // sleep 100ms
        debug_print("Start EXP\r\n");
        set_motor_goto_position_accel_exec(0, target_speed_exp, step_num_base, step_freq_base);
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        debug_print("Finished EXP%u\r\n", motor_inmotion);
        motor_disable();
        vTaskDelay(500 / portTICK_PERIOD_MS); // sleep 100ms
    }
}
#if 0
void MotorControlTask(void *pvParameters)
{
    uint16_t msteps = 1;
    uint32_t target_position_abs = 600 * msteps;
    uint16_t target_speed;
    uint16_t step_num_base;
    uint16_t step_freq_base;

    for (target_speed=100; target_speed < 600; target_speed+=99) {
        for (step_num_base=16; step_num_base<30; step_num_base+=3) {
            for (step_freq_base=50; step_freq_base < 200; step_freq_base += 79) {

                target_speed = 1200 * msteps;
                step_num_base = 8;
                step_freq_base = 200 * msteps;


                debug_print("case:\r\n");
                debug_print("target_speed: %u\r\n", target_speed);
                debug_print("step_num_base: %u\r\n", step_num_base);
                debug_print("step_freq_base: %u\r\n", step_freq_base);
                vTaskDelay(500 / portTICK_PERIOD_MS); // sleep 100ms
                debug_print("Start moving +\r\n");
                set_motor_goto_position_accel_exec(target_position_abs, target_speed, step_num_base, step_freq_base);
                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
                //while (motor_inmotion);
                debug_print("Finished moving + %u\r\n", motor_inmotion);

                vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 100ms
                debug_print("Start moving -\r\n");
                set_motor_goto_position_accel_exec(0, target_speed, step_num_base, step_freq_base);
                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
                debug_print("Finished moving - %u\r\n", motor_inmotion);

            }
        }
    }
    debug_print("Finished everything\r\n");

    while (1)
    {
        //TODO: write this function!
        vTaskDelay(1000 / portTICK_PERIOD_MS); // sleep 100ms
    }
}
#endif

