
#include <stdint.h>
#include "core/buttons.h"
#include "hal/pins.h"
#include "hal/io.h"
#include "core/debug.h"

static ButtonsState read_buttons();

static const enum dio_pin button_pins[N_BUTTONS] = {
    DIO_PIN_BUTTON_NEXT,
    DIO_PIN_BUTTON_CONFIRM,
    DIO_PIN_BUTTON_UP,
    DIO_PIN_BUTTON_DOWN,
    DIO_PIN_BUTTON_VTIDAL_UP,
    DIO_PIN_BUTTON_VTIDAL_DOWN,
    DIO_PIN_BUTTON_FREQ_RESPI_UP,
    DIO_PIN_BUTTON_FREQ_RESPI_DOWN,
    DIO_PIN_BUTTON_ALARM_MUTE,
    DIO_PIN_BUTTON_ALARM_ACK,
    DIO_PIN_BUTTON_STARTSTOP,
};

const char *buttons_descr[N_BUTTONS] = {
    "NEXT",
    "CONFIRM",
    "UP",
    "DOWN",
    "VTIDAL_UP",
    "VTIDAL_DOWN",
    "BPM_UP",
    "BPM_DOWN",
    "ALRAM_MUTE",
    "ALRAM_ACK",
    "STARTSTOP"
};

static ButtonsState buttons_state;

void initButtons() {
    for (uint8_t i=0; i < N_BUTTONS; i++) {
        dio_init(button_pins[i], DIO_INPUT_PULLUP);
    }
    buttons_state = read_buttons();
}

static ButtonsState read_buttons() {
    ButtonsState res = 0;
    for (uint8_t i=0; i < N_BUTTONS; i++) {
        res |= (dio_read(button_pins[i]) << i);
    }
    return res;
}

ButtonsState poll_buttons() {
    ButtonsState new_state = read_buttons();
    ButtonsState res = (~new_state) & buttons_state;
    //debug_print("polling state %x", buttons_state);
    //debug_print(" new_state %x", new_state);
    //debug_print(" res %x \r\n", res);
    buttons_state = new_state;
    return res;
}

