
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "core/parameters.h"
#include "core/system.h"
#include "core/buttons.h"
#include "core/utils.h"
#include "core/display.h"

#define CURR_DEBUG_PREFIX parametersDbg
#include "core/debug.h"

#if DEBUG_PARAMETERS
#define DEBUG_PRINT debug_print_prefix
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_PARAMETERS

// current parameters
volatile uint8_t tidal_vol; // tens of mL
volatile uint8_t bpm;
volatile uint8_t ie;
volatile uint8_t p_max;
volatile ExtraParam_t extra_param;

// last confirmed parameters
volatile uint8_t saved_tidal_vol; // tens of mL
volatile uint8_t saved_bpm;
volatile uint8_t saved_ie;
volatile uint8_t saved_p_max;

static void save_parameters();
static void revert_parameters();
TickType_t param_last_update_time;

static bool handle_button_press(ButtonsState buttons_pressed);
static void save_parameters();
static void revert_parameters();

// return true if a parameter changed
static bool handle_button_press(ButtonsState buttons_pressed) {
    bool updated_setting = false;
    if (BUTTON_PRESSED(buttons_pressed, button_vtidal_up)) {
        tidal_vol = MIN(MAX_TIDAL_VOL, tidal_vol + INC_TIDAL_VOL);
        updated_setting = true;
    }
    if (BUTTON_PRESSED(buttons_pressed, button_vtidal_down)) {
        tidal_vol = MAX(MIN_TIDAL_VOL, tidal_vol - INC_TIDAL_VOL);
        updated_setting = true;
    }
    if (BUTTON_PRESSED(buttons_pressed, button_freq_respi_up)) {
        bpm = MIN(MAX_BPM, bpm + INC_BPM);
        updated_setting = true;
    }
    if (BUTTON_PRESSED(buttons_pressed, button_freq_respi_down)) {
        bpm = MAX(MIN_BPM, bpm - INC_BPM);
        updated_setting = true;
    }
    if (BUTTON_PRESSED(buttons_pressed, button_next)) {
        if (unsaved_extra_param()) {
            // TODO SPEC: signal error ?
        } else {
            extra_param = (extra_param + 1) % N_EXTRA_PARAM;
            updated_setting = true;
        }
    }
    if (BUTTON_PRESSED(buttons_pressed, button_confirm)) {
        DEBUG_PRINT("Current parameters saved");
        save_parameters();
        updated_setting = true;
    }
    if (BUTTON_PRESSED(buttons_pressed, button_up)) {
        switch (extra_param) {
            case ExtraParamIE:
                ie = MIN(MAX_IE, ie + INC_IE);
                updated_setting = true;
                break;
            case ExtraParamPmax:
                p_max = MIN(MAX_PMAX, p_max + INC_PMAX);
                updated_setting = true;
                break;
            case ExtraParamPEEP:
                // PEEP, not settable
                break;
        }
    }
    if (BUTTON_PRESSED(buttons_pressed, button_down)) {
        switch (extra_param) {
            case ExtraParamIE:
                ie = MAX(MIN_IE, ie - INC_IE);
                updated_setting = true;
                break;
            case ExtraParamPmax:
                p_max = MAX(MIN_PMAX, p_max - INC_PMAX);
                updated_setting = true;
                break;
            case ExtraParamPEEP:
                // PEEP, not settable
                break;
        }
    }

    return updated_setting;
}

void initParameters() {
    tidal_vol = DEFAULT_TIDAL_VOL;
    bpm = DEFAULT_BPM;
    ie = DEFAULT_IE;
    p_max = DEFAULT_PMAX;
    save_parameters();

    extra_param = ExtraParamIE;
}

void upd_params(ButtonsState buttons_pressed) {
    bool updated_setting = handle_button_press(buttons_pressed);
    bool upd_display = false;
    if (updated_setting) {
        DEBUG_PRINT("Parameters changed");
        param_last_update_time = xTaskGetTickCount();
        upd_display = true;
    } else if (
            unsaved_parameters() &&
            (xTaskGetTickCount() - param_last_update_time > pdMS_TO_TICKS(PARAM_AUTO_REVERT_DELAY))
            ) {
        revert_parameters();
        upd_display = true;
    }
    if (upd_display) {
        DEBUG_PRINT("NOTIF_PARAM -> LCD");
        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PARAM, eSetBits);
    }
}

bool unsaved_tidal_vol() {
    return tidal_vol != saved_tidal_vol;
}

bool unsaved_bpm() {
    return bpm != saved_bpm;
}

bool unsaved_extra_param() {
    return  (ie != saved_ie) || (p_max != saved_p_max);
}

bool unsaved_parameters() {
    return unsaved_bpm() || unsaved_tidal_vol() || unsaved_extra_param();
}

// Saves/confirms the current parameters.
static void save_parameters() {
    saved_tidal_vol = tidal_vol;
    saved_bpm = bpm;
    saved_ie = ie;
    saved_p_max = p_max;
}

// Reverts the current parameters to their latest saved state.
static void revert_parameters() {
    tidal_vol = saved_tidal_vol;
    bpm = saved_bpm;
    ie = saved_ie;
    p_max = saved_p_max;
}
