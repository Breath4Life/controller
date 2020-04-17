#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/lcd.h"
#include "core/system.h"
#include "core/display.h"
#include "core/main_task.h"
#include "core/debug.h"
#include "core/analog_read.h"
#include "core/alarm.h"

#define DEBUG_DISPLAY 0
#if DEBUG_DISPLAY
#define DEBUG_PRINT debug_print
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_DISPLAY

// Time in ticks
#define MUTE_MSG_PERIOD pdMS_TO_TICKS(2000L)
#define PARAM_BLINK_PERIOD pdMS_TO_TICKS(250)

static void initDisplay();
static void displayWelcomeMsg();

// Pressure measurements display
static void displayPlateauPressure();
static void displayPeakPressure();

// Information zone display (state, error, mute)
static void refreshInfoZone();
static void displayState();
static void displayErrorCode();
static void toggleMuteMsg();

// Used for the alternating muted indication
static TickType_t muteMsgLastVisible = 0;
static bool muteMsgVisible = false;

// Parameters display
static void refreshParametersZone();
static void toggleUnsavedParameters();
static void displayTidalVolume(bool visible);
static void displayRespFreq(bool visible);
static void displayExtraParam(bool visible);

// Used for unconfirmed parameters blinking
static TickType_t paramLastVisible = 0;
static bool paramVisible = false;

static void pollAlarmMutedDisplay();
static void pollUnsavedParamDisplay();

static void processDisplayNotification(uint32_t notification);

void LCDDisplayTask(void *pvParameters)
{
    initDisplay();
    displayWelcomeMsg();

    while (true) {
        pollAlarmMutedDisplay();
        pollUnsavedParamDisplay();

        uint32_t notification = 0;
        BaseType_t notif_recv = xTaskNotifyWait( 0x0,
                                                 ALL_NOTIF_BITS,
                                                 &notification,
                                                 pdMS_TO_TICKS(100));

        if (notif_rcv == pdTRUE) {
            processDisplayNotification(notification);
        }

        lcd_refreshLCD();
    }
}

/*
 * Initializes the display.
 */
static void initDisplay() {
    DEBUG_PRINT("[LCD] Init.\r\n");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

    lcd_initLCD();
}

/*
 * Displays the welcome message.
 */
static void displayWelcomeMsg() {
    lcd_write_string(WELCOME_MSG1, 1, 1, NO_CR_LF);
    lcd_write_string(WELCOME_MSG2, 2, 1, NO_CR_LF);
}

/*
 * If alarm is muted, LCD information zone must alternate between
 * a muted alarm indication and the current state/error information
 */
static void pollAlarmMutedDisplay() {
    if (alarmMuted) {
        if (xTaskGetTickCount() - muteMsgLastVisible > MUTE_MSG_PERIOD) {
            toggleMuteMsg();
         }
    } else if (muteMsgVisible) {
        // Ensures the state/error information is visible when alarm has been unmuted
        refreshInfoZone();
    }
}

/*
 * Toggle the appearance of the "MUTED" message in
 * the information zone of the LCD.
 */
static void toggleMuteMsg() {
    if (muteMsgVisible) {
        refreshInfoZone();
    } else {
        lcd_write_string(" MUTED ", 1, 10, NO_CR_LF);
        muteMsgLastVisible = xTaskGetTickCount();
    }

    muteMsgVisible = !muteMsgVisible;
}

/*
 * Displays either the current state or the current
 * alarm error code, depending on the alarm state.
 */
static void refreshInfoZone() {
    if (alarmLevel == noAlarm) {
        disp_state();
    } else {
        disp_alarm();
    }
}

/*
 * Unsaved parameters must blink while unconfirmed
 */
static void pollUnsavedParamDisplay() {
    if (unsaved_parameters()) {
        if (xTaskGetTickCount() - paramLastVisible > PARAM_BLINK_PERIOD) {
            toggleUnsavedParameters();
        }
    } else if (!paramVisible) {
        // Ensure parameters are visible when confirmed
        refreshParametersZone();
    }
}

/*
 * Makes unsaved parameters blink.
 */
static void toggleUnsavedParameters() {
    if (tidal_vol != saved_tidal_vol) {
        displayTidalVolume(!tidalVolumeVisible);
    }

    if (bpm != saved_bpm) {
        displayRespFreq(!respFreqVisible);
    }

    if (unsaved_extra_param()) {
        displayExtraParam(!extraParamVisible);
    }

    paramVisible = !paramVisible;
}

/*
 * Displays all parameters and update the paramLastVisible
 */
static void refreshParametersZone() {
    displayTidalVolume(true);
    displayRespFreq(true);
    displayExtraParam(true);

    paramVisible = true;
    paramLastVisible = xTaskGetTickCount();
}


/*
 * Display tidal volume if visible is true, a blank
 * space of the corresponding size otherwise.
 */
static void displayTidalVolume(bool visible) {
    char tidalVolumeBuffer[8];
    if (!visible) {
        sprintf(tidalVolumeBuffer, "    ");
    } else {
        sprintf(tidalVolumeBuffer, "%2u0 ", tidal_vol);
    }

    lcd_write_string(tidalVolumeBuffer, 2, 1, NO_CR_LF);
}

/*
 * Display respiratory frequency if visible is true, a blank
 * space of the corresponding size otherwise.
 */
static void displayRespFreq(bool visible) {
    char respFreqBuffer[8];

    if (!visible) {
        sprintf(respFreqBuffer, "   ");
    } else {
        sprintf(respFreqBuffer, "%2u ", bpm);
    }

    lcd_write_string(respFreqBuffer, 2, 5, NO_CR_LF);
}

/*
 * Display extra parameter if visible is true, a blank
 * space of the corresponding size otherwise.
 */
static void displayExtraParam(bool visible) {
    char extraParamBuffer[12];

    if (!visible) {
        sprintf(extraParamBuffer, "         ");
    } else {
        if (extra_param == 0) {
            sprintf(extraParamBuffer, "I:E = 1:%1u", ie);
        } else if (extra_param == 1) {
            sprintf(extraParamBuffer, "Pmax = %2u", p_max);
        } else if (extra_param == 2) {
            sprintf(extraParamBuffer, "PEEP = %2u", peep);
        }
    }

    lcd_write_string(extraParamBuffer, 2, 8, NO_CR_LF);
}

/*
 * Display plateau pressure.
 */
static void displayPlateauPressure() {
    char plateauPressureBuffer[5];
    sprintf(plateauPressureBuffer, "PL%2i", p_plateau);
    lcd_write_string(plateauPressureBuffer, 1, 1, NO_CR_LF);
}

/*
 * Display peak pressure.
 */
static void displayPeakPressure() {
    char peakPressureBuffer[6];
    sprintf(peakPressureBuffer, "PIC%2i", p_peak);
    lcd_write_string(peakPressureBuffer, 1, 5, NO_CR_LF);
}

/*
 * Display current state in the LCD information zone.
 */
static void displayState() {
    if (globalState == critical_failure) {
        DEBUG_PRINT("[LCD] rcvd crit_fail.\r\n");
        switch (alarmCause) {
            case doorOpen:
                lcd_write_string(DOOR_OPEN_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(CRITICAL_FAILURE_MSG, 2, 1, NO_CR_LF);
                break;
            case cfMotorError:
                lcd_write_string(MOTOR_ERROR_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(CRITICAL_FAILURE_MSG, 2, 1, NO_CR_LF);
                break;
            case powerError:
                lcd_write_string(POWER_ERROR_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(CRITICAL_FAILURE_MSG, 2, 1, NO_CR_LF);
                break;
             default:
                // Should never happen
                DEBUG_PRINT("[LCD] Unknown critical failure.\r\n");
                lcd_write_string(UNKNOWN_ERROR_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(CRITICAL_FAILURE_MSG, 2, 1, NO_CR_LF);
        }
    } else if (globalState == welcome_wait_cal) {
        switch (alarmCause) {
            case noError:
                lcd_write_string(WAIT_CALI_MSG1, 1, 1, NO_CR_LF);
                lcd_write_string(WAIT_CALI_MSG2, 2, 1, NO_CR_LF);
                break;
            case calibPatientConnected:
                lcd_write_string(PAT_CONNECTED_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(RETRY_CALI_MSG, 2, 1, NO_CR_LF);
                break;
            case calibIncorrectFlow:
                lcd_write_string(INC_FLOW_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(RETRY_CALI_MSG, 2, 1, NO_CR_LF);
                break;
            default:
                // Should never happen
                DEBUG_PRINT("[LCD] Unknown calib error.\r\n");
                lcd_write_string(UNKNOWN_ERROR_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(WAIT_CALI_MSG2, 2, 1, NO_CR_LF);
        }
    } else if(globalState == calibration) {
        lcd_write_string(CALI_MSG1, 1, 1, NO_CR_LF);
        lcd_write_string(CALI_MSG2, 2, 1, NO_CR_LF);
    } else if (globalState == stop) {
        lcd_write_string(" STOP  ",1,10,NO_CR_LF);
    } else if (globalState == run) {
        lcd_write_string(" RUN   ",1,10,NO_CR_LF);
    }
}

/*
 * Displays alarm error code in the LCD information zone.
 */
static void displayErrorCode() {
    // FIXME: buffer useless here
    char alarm_buffer[9];

    // FIXME: char* [] indexed by alarm Cause containing error codes
    switch(alarmCause) {
        case overPressure:
            sprintf(alarm_buffer, " MXPSR ");
            break;
        case noPressure:
            sprintf(alarm_buffer, " NOPSR ");
            break;
        case highPressure:
            sprintf(alarm_buffer, " HOPSR ");
            break;
        case highTemperature:
            sprintf(alarm_buffer, " MXTPM ");
            break;
        case lowPressure:
            sprintf(alarm_buffer, " LOPSR ");
            break;
        case abnVolume:
            sprintf(alarm_buffer, " TIDALV");
            break;
        case abnFreq:
            sprintf(alarm_buffer, " RESPR ");
            break;
        case auxPower:
            sprintf(alarm_buffer, " AUXPWR");
            break;
        case noError:
            // FIXME: should never happen if above code well design
            displayState()
            break;
        default:
            sprintf(alarm_buffer, " ALARM ");
    }

    lcd_write_string(alarm_buffer, 1, 10, NO_CR_LF);
}

static void processDisplayNotification(uint32_t notification) {
    // Update parameters
    if (notification & DISP_NOTIF_PARAM) {
        DEBUG_PRINT("[LCD] Rcvd NOTIF_PARAM.\r\n");
        refreshParamZone();
    }

    // Update plateau pressure
    if (notification & DISP_NOTIF_PLATEAU_P) {
        DEBUG_PRINT("[LCD] Rcvd NOTIF_PLATEAU_P.\r\n");
        displayPlateauPressure();
    }

    // Update peak pressure
    if (notification & DISP_NOTIF_PEAK_P) {
        DEBUG_PRINT("[LCD] Rcvd NOTIF_PEAK_P.\r\n");
        displayPeakPressure();
    }

    // Update LCD information zone
    if (notification & (DISP_NOTIF_STATE | DISP_NOTIF_ALARM)) {
        DEBUG_PRINT("[LCD] Rcvd NOTIF_STATE|NOTIF_ALARM.\r\n");
        refreshInfoZone();
    }
}
