/** Task in charge of reading the analog sensors:
 * the pressure and temperature sensors.
 * This tasks update regularly the various state variables.
 */
#ifndef ANALOG_READ_H_
#define ANALOG_READ_H_

#include "FreeRTOS.h"

// Period at which each sensor is read.
#define ANALOG_SENSOR_PERIOD pdMS_TO_TICKS(100)

// Threshold (in cm H2O) to generate a NO_PRESSURE alarm.
#define NO_PRESSURE_THRESHOLD 10

// Threshold (in cm H2O) to generate a LOW_PRESSURE alarm.
#define LOW_PRESSURE_THRESHOLD 20

// Threshold (in cm H2O) to generate a PATIENT_CONNECTED alarm during the
// calibration phase.
#define CALIBRATION_MAX_P 10

// Threshold temperature in °C for the environment sensor temperature above
// which a MACHINE_HIGH_TEMPERATURE alarm.
#define MAX_TEMP_MACHINE 60

// Threshold temperature in °C for the motor sensor temperature above
// which a MOTOR_HIGH_TEMPERATURE alarm.
#define MAX_TEMP_MOTOR 120

/** @p Instantaneous pressure in cm H2O
 */
extern volatile int16_t p;

/** @p_peak Peak pressure over the last cycle in cm H2O.
 */
extern volatile int16_t p_peak;

extern volatile int16_t cycle_p_peak;

/** @p_plateau Plateau pressure for the last cycle in cm H2O.
 */
extern volatile int16_t p_plateau;

/** @peep PEEP for the last cycle in cm H2O.
 */
extern volatile int16_t peep;

/** @temp0 Last machine temperature read in °C
 */
extern volatile int16_t temp0;

/** @temp1 Last motor temperature read in °C
 */
extern volatile int16_t temp1;

/** @init_analog_read Initialize the state of the analog reading task.
 */
void init_analog_read();

/** @AnalogReadTask Analog sensors reading task.
 */
void AnalogReadTask(void *pvParameters);

/** @measure_p_plateau Function to be called at the end of the plateau (when
 * the plateau pressure is reached).
 */
void measure_p_plateau();

/** @measure_peep Function to be called at the end of the expiratory phase
 * (when the PEEP is reached).
 */
void measure_peep();

void reset_pressure();
void publish_p_peak();

/** @get_flow Reads the current flow measured with the differential pressure sensor.
 *
 * @param f: The current flow is written in this variable (unit: ml/min).
 */
void get_flow(int32_t *f);

#endif // ANALOG_READ_H_
