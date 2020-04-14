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

#if DEBUG_ANALOG_READ
#define DEBUG_PRINT debug_print
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_PRINT

// Instantaneous pressure in cmH2O
volatile int16_t p;
// Peak pressure over the last cycle in cmH2O
volatile int16_t p_peak;
// Peak pressure over the current cycle in cmH2O
volatile int16_t cycle_p_peak;
// Plateau pressure last measurement in cmH2O
volatile int16_t p_plateau;
// PEEP last measurement in cmH2O
volatile int16_t peep;

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
    // Initialize pressure measurements
    p = 0;
    p_peak = 0;
    cycle_p_peak = 0;
    p_plateau = 0;
    peep = 0;

    // Initialize temperature measurements
    temp0 = 0;
    temp1 = 0;

    curr_mes = pressure;
    aio_read_start(aio_pins[curr_mes]);
}

void AnalogReadTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    DEBUG_PRINT("[ANALOG-READ] Starting.\r\n");

    while (1) {
        if (aio_ready()) {
            uint16_t res = aio_read_result();
            switch (curr_mes) {
                case pressure:
                    // Instantaneous pressure
                    p = mes2pres(res);

                    if (globalState == run) {
                        if (p > p_max) {
                            // Overpressure
                            DEBUG_PRINT("[P-SENS] OVERPRESSURE.\r\n");
                            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_OVER_PRESSURE, eSetBits);
                            xTaskNotify(mainTaskHandle, ALARM_NOTIF_OVERPRESSURE, eSetBits);
                        }

                        // Track max. instantaneous pressure during inspiration
                        if (breathState == insp && p > cycle_p_peak) {
                            cycle_p_peak = p;
                        }

                        // Inspiration is over, set p_peak, notify LCD and check thresholds
                        if (breathState == plateau && cycle_p_peak > 0) {
                            cli();
                            p_peak = cycle_p_peak;
                            sei();
                            cycle_p_peak = 0;
                            DEBUG_PRINT("[P-SENS] Updated p_peak.\r\n");
                            xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PEAK_P, eSetBits);

                            if (p_peak < NO_PRESSURE_THRESHOLD) {
                                // No pressure
                                DEBUG_PRINT("[P-SENS] NO PRESSURE.\r\n");
                                xTaskNotify(mainTaskHandle, ALARM_NOTIF_NO_PRESSURE, eSetBits);
                            } else if (p_peak < LOW_PRESSURE_THRESHOLD) {
                                // Low pressure
                                DEBUG_PRINT("[P-SENS] LOW PRESSURE.\r\n");
                                xTaskNotify(mainTaskHandle, ALARM_NOTIF_LOW_PRESSURE, eSetBits);
                            }
                        }
                    } else if (globalState == calibration) {
                        if (p > CALIBRATION_MAX_P) {
                            // Pressure increased during self-calibration
                            DEBUG_PRINT("[P-SENS] PATIENT CONNECTED?\r\n");
                            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_OVER_PRESSURE, eSetBits);
                            xTaskNotify(mainTaskHandle, NOTIF_PATIENT_CONNECTED, eSetBits);
                        }
                    }

                    curr_mes = temperature0;
                    break;
                case temperature0:
                    temp0 = mes2temp(res);

                    if (temp0 > MAX_TEMP0 && errorCode != highTemperature) {
                        DEBUG_PRINT("[T0-SENS] HIGH TEMP: %i.\r\n", temp0);
                        xTaskNotify(mainTaskHandle, ALARM_NOTIF_HIGH_TEMP, eSetBits);
                    }
                    curr_mes = temperature1;
                    break;
                case temperature1:
                    temp1 = mes2temp(res);

                    if (temp1 > MAX_TEMP1 && errorCode != highTemperature) {
                        DEBUG_PRINT("[T1-SENS] HIGH TEMP: %i.\r\n", temp1);
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

    return (int16_t) tmp;
}

void measure_p_plateau() {
    cli();
    p_plateau = p;
    sei();
    // TODO: alarm on p_plateau?
    DEBUG_PRINT("[P-SENS] p_plateau = %i.\r\n", p_plateau);
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PLATEAU_P, eSetBits);
}

void measure_peep() {
    cli();
    peep = p;
    sei();
    DEBUG_PRINT("[P-SENS] peep = %i.\r\n", peep);
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PARAM, eSetBits);
}
