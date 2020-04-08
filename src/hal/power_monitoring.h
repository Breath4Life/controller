#ifndef POWER_MONITORING_H_
#define POWER_MONITORING_H_

#include <stdint.h>

void init_power_monitoring();
uint8_t error_power_aux();
uint8_t error_power_main();

#endif // POWER_MONITORING_H_
