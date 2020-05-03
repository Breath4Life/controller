#include <stdint.h>
#include <avr/interrupt.h>
#include "core/analog_read.h"
#include "hal/io.h"
#include "hal/pins.h"
#include "hal/time.h"
#include "core/utils.h"
#include "core/system.h"
#include "core/main_task.h"
#include "core/motor_control.h"
#include "core/display.h"
#include "core/alarm.h"
#include "core/parameters.h"

#include "FreeRTOS.h"
#include "task.h"

#define CURR_DEBUG_PREFIX analogRead
#include "core/debug.h"

#define N_ANALOG_READS 4

// Absurd value than can never happen
#define INIT_CYCLE_P_PEAK -100

#if DEBUG_ANALOG_READ
#define DEBUG_PRINT debug_print_prefix
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_ANALOG_READ

#define SEND_TO_SERIAL 0
#define CALIBRATE_FLOW 1

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

// Flow measured with differential pressure sensor
volatile int32_t flow;

volatile int16_t temp_machine;
volatile int16_t temp_motor;

static int16_t mes2pres(uint16_t mes);
static int32_t mes2flow(uint16_t mes);
static int16_t mes2temp(uint16_t mes);

static enum {
    mesPressure,
    mesFlow,
    mesTempMachine,
    mesTempMotor
} curr_mes;

static const uint8_t aio_pins[N_ANALOG_READS] = {
    AIO_PIN_PRESSURE_SENSOR_0,
    AIO_PIN_PRESSURE_SENSOR_1,
    AIO_PIN_TEMP_SENSOR_0,
    AIO_PIN_TEMP_SENSOR_1
};

void init_analog_read() {
    // Initialize pressure measurements
    p = 0;
    p_peak = 0;
    cycle_p_peak = INIT_CYCLE_P_PEAK;
    p_plateau = 0;
    peep = 0;

    // Initialize temperature measurements
    temp_machine = 0;
    temp_motor = 0;

    curr_mes = mesPressure;
    aio_read_start(aio_pins[curr_mes]);
}

void reset_pressure() {
    cli();
    cycle_p_peak = INIT_CYCLE_P_PEAK;
    sei();
}

// Inspiration is over, set p_peak, notify LCD and check thresholds
void publish_p_peak() {
    cli();
    p_peak = cycle_p_peak;
    sei();
    DEBUG_PRINT("Upd p_peak");
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PEAK_P, eSetBits);

    if (p_peak < NO_PRESSURE_THRESHOLD) {
        // No pressure
        DEBUG_PRINT("NO PRESSURE");
        sendNewAlarm(noPressure);
    } else if (p_peak < LOW_PRESSURE_THRESHOLD) {
        // Low pressure
        DEBUG_PRINT("LOW PRESSURE");
        sendNewAlarm(lowPressure);
    }
}

void AnalogReadTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    DEBUG_PRINT("Starting");

    while (1) {
        if (aio_ready()) {
            uint16_t res = aio_read_result();
            switch (curr_mes) {
                case mesPressure:
                    // Instantaneous pressure
                    p = mes2pres(res);

                    if (globalState == run) {
                        if (p > p_max) {
                            // Overpressure
                            DEBUG_PRINT("OVERPRESSURE");
                            sendNewAlarm(overPressure);
                            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_OVER_PRESSURE, eSetBits);
                        }

                        // Track max. instantaneous pressure during inspiration
                        if (p > cycle_p_peak) {
                            cycle_p_peak = p;
                        }

                    } else if (globalState == calibration) {
                        if (p > CALIBRATION_MAX_P) {
                            // Pressure increased during self-calibration
                            DEBUG_PRINT("PATIENT CONNECTED?");
                            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_OVER_PRESSURE, eSetBits);
                            sendNewAlarm(calibPatientConnected);
                        }
                    }
                    curr_mes = mesFlow;
                    break;
                case mesFlow:
                    flow = mes2flow(res);
                    curr_mes = mesTempMachine;
                    break;
                case mesTempMachine:
                    temp_machine = mes2temp(res);

                    if (temp_machine > MAX_TEMP_MACHINE) {
                        DEBUG_PRINT("HIGH TEMP MACHINE: %i", temp_machine);
                        sendNewAlarm(highTemperature);
                    }
                    curr_mes = mesTempMotor;
                    break;
                case mesTempMotor:
                    temp_motor = mes2temp(res);

                    if (temp_motor > MAX_TEMP_MOTOR) {
                        DEBUG_PRINT("HIGH TEMP MOTOR: %i", temp_motor);
                        sendNewAlarm(motorHot);
                    }
                    curr_mes = mesPressure;
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

#if SEND_TO_SERIAL
    // Send curr_time and in µs, pressure in 1/8cmH2O
    debug_print("%lu:%i\r\n", time_us(), (tmp >> 4) + 1);
#endif

    // TODO check scale and offset calculation above
    // FIXME: +1 for auto-zero (i.e., set measured pressure to 0
    // when pressured difference is supposed to be 0)
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

static int32_t mes2flow(uint16_t mes) {
    /*
     * MPXV7002DP pressure sensor
     * --------------------------
     * Unit conversion: 1kPa = 101.972mmH2O
     *
     * # Sensitivity
     * 1 V/kPa ~= 9.8mV/mmH2O
     * analogRead() precision is 10 bit on 5V: 1 = 4.9mV
     * -> 1 corresponds to 0.5 mmH2O
     *
     * # Transfer function
     * Vout[V] = Vs * (0.2 * p[kPa] + 0.5) with Vs = 5V
     * = p[kPa] + 2.5
     * -> p[kPa] = Vout[V] - 2.5
     *
     * analogRead() precision is 10 bit: 1 = 4.9mV.
     *
     * Scale for mmH2O = 0.0049 * 101.971 = 0.4996579
     * Offset for mmH2O = -2.5 * 101.971 = -254.9275
     * Scale for 1/2mmH2O = 0.0049 * 101.971 = 0.9993158 ≃ 1
     * Offset for 1/2mmH2O = -2.5 * 101.971 = -509.855 ≃ -510
     * -> p[1/2mmH2O] = r[reading] - 510
     *
     * No overflow can happen on int16_t.
    */

    // pressure in units of 1/2mmH2O
    // -510 - 37 to zero output at zero pressure difference
    // FIXME: needs proper calibration for auto-zeroing
    int32_t pressure = ((int32_t) mes) - 547L;

    // FIXME: needs proper calibration: currently obtained
    // from a rough diff. pressure/flow piecewise linear
    // curve fitting
    /*
    int32_t tmp;
    if (pressure < -11) {
        tmp = 1298L * pressure - 26201L;
    } else if (pressure < 18) {
        tmp = 2648L * pressure - 11349L;
    } else {
        tmp = 1455L * pressure + 11125L;
    }
    */

    // Assume flow/pressure relation is linear
    int32_t tmp = 1839L * pressure;

#if CALIBRATE_FLOW
    debug_print("%lu:%li:%li:%li\r\n", time_us(), (int32_t) mes, pressure, tmp);
#endif

    return tmp;
}

void measure_p_plateau() {
    cli();
    p_plateau = p;
    sei();
    // TODO: alarm on p_plateau?
    DEBUG_PRINT("p_plateau = %i", p_plateau);
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PLATEAU_P, eSetBits);
}

void measure_peep() {
    cli();
    peep = p;
    sei();
    DEBUG_PRINT("peep = %i", peep);
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PARAM, eSetBits);
}
