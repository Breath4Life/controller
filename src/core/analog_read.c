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

volatile int16_t temp0;
volatile int16_t temp1;

static int16_t mes2pres(uint16_t mes);
static int16_t mes2temp(uint16_t mes);

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
#endif

    while (1) {
        if (aio_ready()) {
            uint16_t res = aio_read_result();
            switch (curr_mes) {
                case pressure:
                    p = mes2pres(res);
                    if (stoppedOrRunning()) {
                        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_INST_P, eSetBits);
                     }

                    // FIXME: was stoppedOrRunning before. Doesn't make sense in stop state right?
                    if (globalState == run) {
                        // FIXME: added errorCode != overPressure, makes sense?
                        if (p > p_max && errorCode != overPressure) {
                            xTaskNotify(mainTaskHandle, ALARM_NOTIF_OVERPRESSURE, eSetBits);
                            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_OVER_PRESSURE, eSetBits);
                        }


                        if (breathState == insp && p > cycle_p_peak) {
                            cycle_p_peak = p;
                        }

                        // FIXME: added errorCode != noPressure, makes sense?
                        if (p < NO_PRESSURE_THRESHOLD && errorCode != noPressure) {
                            // TODO: only in insp ?
                            xTaskNotify(mainTaskHandle, ALARM_NOTIF_NO_PRESSURE, eSetBits);

#if DEBUG_ANALOG_READ
                            debug_print("[ANALOG-READ] NOPSR notif to MAIN.\r\n");
#endif
                        }
                        // FIXME: added errorCode != lowPressure, makes sense?
                        else if (p < LOW_PRESSURE_THRESHOLD && errorCode != lowPressure) {
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
                            //xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_OVER_PRESSURE, eSetBits);
                            xTaskNotify(mainTaskHandle, ALARM_NOTIF_PATIENT_CONNECTED, eSetBits);
                        }
                    }
                    curr_mes = temperature0;
                    break;
                case temperature0:
                    temp0 = mes2temp(res);
                    // FIXME: added errorCode != highTemperature, makes sense?
                    if (temp0 > MAX_TEMP0 && errorCode != highTemperature) {
                        xTaskNotify(mainTaskHandle, ALARM_NOTIF_HIGH_TEMP, eSetBits);
                    }
                    curr_mes = temperature1;
                    break;
                case temperature1:
                    temp1 = mes2temp(res);
                    // FIXME: added errorCode != highTemperature, makes sense?
                    if (temp1 > MAX_TEMP1 && errorCode != highTemperature) {
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
    /*
     * # Transfer function of MPX5010DP
     * Vout[V] = Vs * (0.09*p[kPa] + 0.04) with Vs = 5V
     * -> p[kPa] = (Vout/Vs - 0.04) / 0.09
     *
     * ADC resolution is 10 bit: 1 = 4.9mV.
     * Unit conversion: 1kPa = 101.972mmH2O
     *
     * Scale for mmH2O      = 0.0049/5.0/ 0.09 * 101.971    = 1.110351
     * Offset for mmH2O     = -0.04 / 0.09 * 101.971        = -45.320444
     * Scale for cmH2O      = scale for mmH2O / 10          = 0.1110351
     * Offset for cmH2O     = offset for mmH2O / 10         = -4.5320444
     * Scale for 1/64cmH2O  = scale for cmH2O * 64          = 7.106 ~ 7
     * Offset for 1/64cmH2O = offset for cmH2O * 64         = -290.050 ~ -290
     *
     * Max value in 1/64cmH2O = 1023 * 7 - 290 = 6871
     * Min value in 1/64cmH2O = 0 * 7 - 290 = -290
     * -> fits in int16_t
     *
     * Value obtained in cmH2O by dividing by 64: >> 6.
     */

    static int16_t scale_MPX5010DP = 7;
    static int16_t offset_MPX5010DP = -290;

    // FIXME mes in uint16_t, could that cause any problem?
    int16_t tmp = scale_MPX5010DP * mes + offset_MPX5010DP;

    // FIXME +1 added empirically here by observing that default value is -1...
    // TODO check scale and offset calculation above
    return (tmp >> 6) + 1;
}

static int16_t mes2temp(uint16_t mes) {
    /*
     * # Transfer function of LMT87LP
     * See http://www.ti.com/lit/ds/symlink/lmt87.pdf, page 10)
     *
     * V - V1 = (V2 - V1) / (T2 - T1) * (T - T1)
     * with voltage in mV and temperature in °C.
     * -> T = (V - V1)/(V2 - V1) * (T2 - T1) + T1
     *
     * Choosing an operating range of 0 - 150°C, voltage ranges from
     * 2633mV to 538mV
     * T1 = 0   , V1 = 2633mV (537)
     * T2 = 150 , V2 = 538mV (110)
     *
     * Scale for °C         = (T2 - T1)/(V2 - V1)               = -150/427
     * Offset for °C        = -V1 * (T2 - T1)/(V2 - V1) + T1    = 80550/427
     *
     * Max value = 188.642
     * Min value = -170.726
     * -> fits in int16_t
     *
     */

    // FIXME: this is super dirty but I'm tired, fix that
    int32_t tmp = (-150 * ((int32_t) mes) + 80550)/427;
#if DEBUG_ANALOG_READ
//    debug_print("[ANALOG-READ] T: %i.\r\n", tmp);
#endif

    return (int16_t) tmp;
}
