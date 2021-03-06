
#include <stdbool.h>
#include "power_monitoring.h"
#include "hal/pins.h"
#include "hal/io.h"

void init_power_monitoring() {
    dio_init(DIO_PIN_AUX_POWER_MONITORING,    DIO_INPUT);
    dio_init(DIO_PIN_MAIN_POWER_MONITORING,   DIO_INPUT);
}

bool error_power_aux() {
    return (dio_read(DIO_PIN_AUX_POWER_MONITORING) == DIO_LOW);
}
bool error_power_main() {
    return (dio_read(DIO_PIN_MAIN_POWER_MONITORING) == DIO_LOW);
}
