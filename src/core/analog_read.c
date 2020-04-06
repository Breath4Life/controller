
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

volatile uint8_t p;
volatile uint8_t p_peak;
volatile uint8_t cycle_p_peak;

volatile uint8_t temp0;
volatile uint8_t temp1;

static uint8_t mes2pres(uint16_t mes);
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
    while (1) {
        if (aio_ready()) {
            uint16_t res = aio_read_result();
            switch (curr_mes) {
                case pressure:
                    p = mes2pres(res);
#if DEBUG_ANALOG_READ
                    debug_print("Pressure: %u\r\n", p);
#endif //DEBUG_ANALOG_READ
                    if (stoppedOrRunning()) {
                        if (p > p_max) {
                            xTaskNotify(mainTaskHandle, MAIN_NOTIF_OVERPRESSURE, eSetBits);
                            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_OVER_PRESSURE, eSetBits);
                        }
                        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_INST_P, eSetBits);
                        if (breathState == insp && p > cycle_p_peak) {
                            cycle_p_peak = p;
                        }
                        if (p < NO_PRESSURE_THRESHOLD) {
                            // TODO: only in insp ?
                            xTaskNotify(mainTaskHandle, MAIN_NOTIF_NOPRESSURE, eSetBits);
                        }
                        if (p < LOW_PRESSURE_THRESHOLD) {
                            // TODO: only in insp ?
                            xTaskNotify(mainTaskHandle, MAIN_NOTIF_LOWPRESSURE, eSetBits);
                        }
                        if (breathState == plateau && cycle_p_peak > 0) {
                                p_peak = cycle_p_peak;
                                cycle_p_peak = 0;
                                xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PEAK_P, eSetBits);
                        }
                    }
                    if (globalState == calibration) {
                        if (p > CALIBRATION_MAX_P) {
                            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_OVER_PRESSURE, eSetBits);
                            xTaskNotify(mainTaskHandle, MAIN_NOTIF_PATIENT_CONNECTED, eSetBits);
                        }
                    }
                    curr_mes = temperature0;
                    break;
                case temperature0:
                    temp0 = mes2temp(res);
#if DEBUG_ANALOG_READ
                    debug_print("Temp0: %u\r\n", temp0);
#endif //DEBUG_ANALOG_READ
                    if (temp0 > MAX_TEMP0) {
                        xTaskNotify(mainTaskHandle, MAIN_NOTIF_TEMPERATURE, eSetBits);
                    }
                    curr_mes = temperature1;
                    break;
                case temperature1:
                    temp1 = mes2temp(res);
#if DEBUG_ANALOG_READ
                    debug_print("Temp1: %u\r\n", temp1);
#endif //DEBUG_ANALOG_READ
                    if (temp1 > MAX_TEMP1) {
                        xTaskNotify(mainTaskHandle, MAIN_NOTIF_TEMPERATURE, eSetBits);
                    }
                    curr_mes = pressure;
                    break;
            }
            aio_read_start(aio_pins[curr_mes]);
        }
        vTaskDelayUntil(&xLastWakeTime, MIN(1, ANALOG_SENSOR_PERIOD/N_ANALOG_READS));
    }
}

static uint8_t mes2pres(uint16_t mes) {
    // TODO
    return (mes >> 8);
}

static uint8_t mes2temp(uint16_t mes) {
    // TODO
    return (mes >> 8);
}
