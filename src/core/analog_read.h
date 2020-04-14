#ifndef ANALOG_READ_H_
#define ANALOG_READ_H_

#include "FreeRTOS.h"

#define ANALOG_SENSOR_PERIOD pdMS_TO_TICKS(100)

#define NO_PRESSURE_THRESHOLD 10
#define LOW_PRESSURE_THRESHOLD 20
#define CALIBRATION_MAX_P 10

#define MAX_TEMP0 50
#define MAX_TEMP1 50

extern volatile int16_t p;
extern volatile int16_t p_peak;
extern volatile int16_t cycle_p_peak;
extern volatile int16_t p_plateau;
extern volatile int16_t peep;

extern volatile int16_t temp0;
extern volatile int16_t temp1;

void init_analog_read();

void AnalogReadTask(void *pvParameters);

void measure_p_plateau();

void measure_peep();

#endif // ANALOG_READ_H_
