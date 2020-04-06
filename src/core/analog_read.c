#include <stdint.h>
#include "core/analog_read.h"
#include "hal/io.h"
#include "hal/pins.h"
#include "core/utils.h"
#include "core/system.h"
#include "core/main_task.h"
#include "core/motor_control.h"
#include "core/display.h"
#include "core/debug.h"

#include "FreeRTOS.h"
#include "task.h"

#define N_ANALOG_READS 3

#define DEBUG_ANALOG_READ 1

volatile int16_t p;
volatile int16_t p_peak;
volatile int16_t cycle_p_peak;

volatile uint8_t temp0;
volatile uint8_t temp1;

static int16_t mes2pres(uint16_t mes);
static uint8_t mes2temp(uint16_t mes);

static enum {
    pressure,
    temperature0,
    temperature1
} curr_mes;

static const uint8_t aio_pins[N_ANALOG_READS] = {
    AIO_PIN_PRESSURE_SENSOR_0,
    AIO_PIN_TEMP_SENSOR_0,
    AIO_PIN_TEMP_SENSOR_1
};

void init_analog_read() {
    p = 0;
    p_peak = 0;
    cycle_p_peak = 0;
    temp0 = 0;
    temp1 = 0;
    curr_mes = pressure;
    aio_read_start(aio_pins[curr_mes]);
}

void AnalogReadTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
#if DEBUG_ANALOG_READ
    debug_print("[ANALOG-READ] Starting.\r\n");
#endif //DEBUG_ANALOG_READ

    while (1) {
        if (aio_ready()) {
            uint16_t res = aio_read_result();
            switch (curr_mes) {
                case pressure:
                    p = mes2pres(res);
                    debug_print("res: %u\r\n", res);
                    debug_print("P: %i\r\n", p);
                    if (stoppedOrRunning()) {
                        if (p > p_max) {
                            xTaskNotify(mainTaskHandle, ALARM_NOTIF_OVERPRESSURE, eSetBits);
                            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_OVER_PRESSURE, eSetBits);
                        }

                        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_INST_P, eSetBits);

                        if (breathState == insp && p > cycle_p_peak) {
                            cycle_p_peak = p;
                        }

                        if (p < NO_PRESSURE_THRESHOLD) {
                            // TODO: only in insp ?
                            xTaskNotify(mainTaskHandle, ALARM_NOTIF_NO_PRESSURE, eSetBits);
                        }

                        if (p < LOW_PRESSURE_THRESHOLD) {
                            // TODO: only in insp ?
                            xTaskNotify(mainTaskHandle, ALARM_NOTIF_LOW_PRESSURE, eSetBits);
                        }

                        if (breathState == plateau && cycle_p_peak > 0) {
                                p_peak = cycle_p_peak;
                                cycle_p_peak = 0;
                                xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PEAK_P, eSetBits);
                        }
                    }
                    if (globalState == calibration) {
                        if (p > CALIBRATION_MAX_P) {
                            // FIXME: same notification to the motor for patient connected and overpressure?
                            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_OVER_PRESSURE, eSetBits);
                            xTaskNotify(mainTaskHandle, ALARM_NOTIF_PATIENT_CONNECTED, eSetBits);
                        }
                    }
                    curr_mes = temperature0;
                    break;
                case temperature0:
                    temp0 = mes2temp(res);
                    if (temp0 > MAX_TEMP0) {
                        xTaskNotify(mainTaskHandle, ALARM_NOTIF_HIGH_TEMP, eSetBits);
                    }
                    curr_mes = temperature1;
                    break;
                case temperature1:
                    temp1 = mes2temp(res);
                    debug_print("[ANALOG_READ] Temp1: %u\r\n", temp1);
                    if (temp1 > MAX_TEMP1) {
                        xTaskNotify(mainTaskHandle, ALARM_NOTIF_HIGH_TEMP, eSetBits);
                    }
                    curr_mes = pressure;
                    break;
            }
            aio_read_start(aio_pins[curr_mes]);
        }

        vTaskDelayUntil(&xLastWakeTime, MAX(1, ANALOG_SENSOR_PERIOD/N_ANALOG_READS));
    }
}

static int16_t mes2pres(uint16_t mes) {
    //static int32_t scale_MPX5010DP = 1110;
    //static int32_t offset_MPX5010DP = -45320;

    //int32_t tmp = MAX(scale_MPX5010DP * mes + offset_MPX5010DP, 0);

    return 25; // (int16_t) tmp / 10000;
}

static uint8_t mes2temp(uint16_t mes) {
    // TODO
    return 12;
}
