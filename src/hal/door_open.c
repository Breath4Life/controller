
#include <stdbool.h>
#include "hal/door_open.h"
#include "hal/pins.h"
#include "hal/io.h"

void door_open_init() {
    dio_init(DIO_PIN_DOOR_OPEN, DIO_INPUT_PULLUP);
}

bool is_door_open() {
    return dio_read(DIO_PIN_DOOR_OPEN) == DIO_HIGH;
}
