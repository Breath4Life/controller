#ifndef BUTTONS_H_
#define BUTTONS_H_

#define N_BUTTONS 11

typedef enum {
    button_right,
    button_left,
    button_up,
    button_down,
    button_vtidal_up,
    button_vtidal_down,
    button_freq_respi_up,
    button_freq_respi_down,
    button_alarm_mute,
    button_alarm_ack,
    button_startstop,
} Buttons;

extern const char *buttons_descr[N_BUTTONS];

typedef uint16_t ButtonsState;

void initButtons();
ButtonsState poll_buttons();

#define BUTTON_PRESSED(state, button) ((uint8_t) (((state) >> (button)) & 0x1))

#endif // BUTTONS_H_
