/** Handling of user settings
 */
#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stdint.h>
#include <stdbool.h>

#include "core/buttons.h"

// tidal vol in tens of mL
#define DEFAULT_TIDAL_VOL 30
#define MAX_TIDAL_VOL 60
#define MIN_TIDAL_VOL 20
#define INC_TIDAL_VOL 2

// breath per min
#define DEFAULT_BPM 15
#define MAX_BPM 30
#define MIN_BPM 12
#define INC_BPM 1

// I:E ratio
#define DEFAULT_IE 1
#define MAX_IE 3
#define MIN_IE 1
#define INC_IE 1

// Pmax
#define DEFAULT_PMAX 38
#define MAX_PMAX 58
#define MIN_PMAX 28
#define INC_PMAX 1

#define PARAM_AUTO_REVERT_DELAY 120 * 1000L

#define N_EXTRA_PARAM 3
typedef enum {
    ExtraParamIE,
    ExtraParamPmax,
    ExtraParamPEEP,
} ExtraParam_t;

// current parameters
extern volatile uint8_t tidal_vol; // tens of mL
extern volatile uint8_t bpm;
extern volatile uint8_t ie;
extern volatile uint8_t p_max;
extern volatile ExtraParam_t extra_param;

/** @initParameters Initialize parameters state.
 */
void initParameters();

/** @upd_params Update parameters state
 * Notifies LCD if parameters state changed.
 *
 * @param buttons_pressed Last presses on the buttons.
 */
void upd_params(ButtonsState buttons_pressed);

/** @unsaved_tidal_vol Check if the tidal volume is not yet confirmed
 *
 * @return true if the tidal volume is not yet confirmed
 */
bool unsaved_tidal_vol();

/** @unsaved_bpm Check if the BPM is not yet confirmed
 *
 * @return true if the BPM is not yet confirmed.
 */
bool unsaved_bpm();

/** @unsaved_bpm Check if the extra parameter is not yet confirmed
 *
 * @return true if the currently displayed extra parameter is not yet
 * confirmed.
 */
bool unsaved_extra_param();

/** @unsaved_bpm Check if any parameter is not yet confirmed
 *
 * @return true if any parameter is not yet confirmed.
 */
bool unsaved_parameters();

#endif //PARAMETERS_H_
