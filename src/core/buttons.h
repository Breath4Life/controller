/** Reading the state of the interface buttons
 */
#ifndef BUTTONS_H_
#define BUTTONS_H_

#define N_BUTTONS 11

typedef enum {
    button_next,
    button_confirm,
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

/** @buttons_descr Human-readable description of each of the buttons (for debug
 * purposes).
 */
extern const char *buttons_descr[N_BUTTONS];

typedef uint16_t ButtonsState;

/** @initButtons Initialize the buttons state.
 */
void initButtons();

/** @poll_buttons Read the state of the buttons
 *
 * @return The list of buttons that have been pressed.
 * This result can be queried with the BUTTON_PRESSED macro.
 */
ButtonsState poll_buttons();

/** @BUTTON_PRESSED Extract a result in a ButtonsState.
 *
 * @param state The ButtonsState where data should be extracted.
 * @param button The Buttons that determine which button to extract from the state.
 *
 * @return 1 is the button was pressed, otherwise 0
 */
#define BUTTON_PRESSED(state, button) ((uint8_t) (((state) >> (button)) & 0x1))

#endif // BUTTONS_H_
