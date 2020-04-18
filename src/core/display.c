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
#include "core/utils.h"
#include "core/parameters.h"

#if DEBUG_DISPLAY
#define DEBUG_PRINT debug_print
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_DISPLAY

// Time in ticks
#define MUTE_MSG_PERIOD pdMS_TO_TICKS(2000L)
#define PARAM_BLINK_PERIOD pdMS_TO_TICKS(250)

static const char *errorCode[] = {
    "       ",
    "MXPSR  ",
    "NOPSR  ",
    "HOPSR  ",
    "MXTPM  ",
    "LOPSR  ",
    "TIDALV ",
    "RESPR  ",
    "BATTERY",
    "FLOW   ",
    "PATIENT",
    "DOOR   ",
    "MOTOR  ",
    "POWER  "
};

static void lcdWriteTwoLines(char * firstLine, char * secondLine);

static void initDisplay();
static void displayWelcomeMsg();

// Pressure measurements display
static void displayPlateauPressure();
static void displayPeakPressure();

// Information zone display (state, error, mute)
static void refreshInfoZone();
static void displayState();
static void displayErrorCode();

// Used for the alternating muted indication
static TickType_t muteMsgLastToggle = 0;
static bool muteMsgVisible = false;

// Parameters display
static void refreshParametersZone();
static void toggleUnsavedParameters();
static void displayTidalVolume(bool visible);
static void displayRespFreq(bool visible);
static void displayExtraParam(bool visible);

// Used for unconfirmed parameters blinking
static TickType_t paramLastToggle = 0;
static bool paramVisible = true;

static TickType_t pollAlarmMutedDisplay();
static TickType_t pollUnsavedParamDisplay();

static void processDisplayNotification(uint32_t notification);

void LCDDisplayTask(void *pvParameters)
{
    initDisplay();
    displayWelcomeMsg();

    uint32_t notification = 0;
    BaseType_t notif_recv = pdFALSE;

    while (true) {
        TickType_t nextMuteMsgToggle = pollAlarmMutedDisplay();
        TickType_t nextParamToggle = pollUnsavedParamDisplay();

        if (notif_recv == pdTRUE) {
            processDisplayNotification(notification);
        }

        lcd_refreshLCD();

        notification = 0;
        notif_recv = xTaskNotifyWait( 0x0, ALL_NOTIF_BITS, &notification,
                                      MIN(nextMuteMsgToggle, nextParamToggle));

    }
}

/*
 * Initializes the display.
 */
static void initDisplay() {
    lcd_initLCD();
    DEBUG_PRINT("[LCD] Init.\r\n");
    // Note: this delay is needed for correct operation of the LCD afterwards
    vTaskDelay(pdMS_TO_TICKS(100));
}

/*
 * Displays the welcome message.
 */
static void displayWelcomeMsg() {
    lcdWriteTwoLines(WELCOME_MSG1, WELCOME_MSG2);
}

/*
 * If alarm is muted, LCD information zone must alternate between
 * a muted alarm indication and the current state/error information
 */
static TickType_t pollAlarmMutedDisplay() {
    TickType_t currentTime = xTaskGetTickCount();

    if (alarmMuted) {
        if (currentTime - muteMsgLastToggle > MUTE_MSG_PERIOD) {
            if (muteMsgVisible) {
                refreshInfoZone();
            } else {
                lcd_write_string(" MUTED ", 1, 10, NO_CR_LF);
            }

            muteMsgLastToggle = currentTime;
            muteMsgVisible = !muteMsgVisible;

            return MUTE_MSG_PERIOD;
         } else {
            // FIXME: overflow risk? Use FreeRTOS TimeOut?
            return MAX(MUTE_MSG_PERIOD - (currentTime - muteMsgLastToggle), 0);
         }
    } else if (muteMsgVisible) {
        // Ensures the state/error information is visible when alarm has been unmuted
        refreshInfoZone();
    }

    return portMAX_DELAY;
}

/*
 * Displays either the current state or the current
 * alarm error code, depending on the alarm state.
 */
static void refreshInfoZone() {
    if (alarmLevel == noAlarm) {
        displayState();
    } else {
        displayErrorCode();
    }
}

/*
 * Unsaved parameters must blink while unconfirmed
 */
static TickType_t pollUnsavedParamDisplay() {
    TickType_t currentTime = xTaskGetTickCount();

    if (unsaved_parameters()) {
        if (currentTime - paramLastToggle > PARAM_BLINK_PERIOD) {
            toggleUnsavedParameters();
            return PARAM_BLINK_PERIOD;
        } else {
            // FIXME: overflow risk? Use FreeRTOS TimeOut?
            return MAX(PARAM_BLINK_PERIOD - (currentTime - paramLastToggle), 0);
        }
    } else if (!paramVisible) {
        // Ensure parameters are visible when confirmed
        refreshParametersZone();
    }

    return portMAX_DELAY;
}

static void toggleUnsavedParameters() {
    if (unsaved_tidal_vol()) {
        displayTidalVolume(!paramVisible);
    }

    if (unsaved_bpm()) {
        displayRespFreq(!paramVisible);
    }

    if (unsaved_extra_param()) {
        displayExtraParam(!paramVisible);
    }

    paramLastToggle = xTaskGetTickCount();
    paramVisible = !paramVisible;
}


/*
 * Displays all parameters and update the paramLastToggle
 */
static void refreshParametersZone() {
    displayTidalVolume(true);
    displayRespFreq(true);
    displayExtraParam(true);

    paramVisible = true;
    paramLastToggle = xTaskGetTickCount();
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
        switch (extra_param) {
            case 0:
                sprintf(extraParamBuffer, "I:E = 1:%1u", ie);
                break;
            case 1:
                sprintf(extraParamBuffer, "Pmax = %2u", p_max);
                break;
            case 2:
                sprintf(extraParamBuffer, "PEEP = %2u", peep);
                break;
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
                lcdWriteTwoLines(DOOR_OPEN_MSG, CRITICAL_FAILURE_MSG);
                break;
            case cfMotorError:
                lcdWriteTwoLines(MOTOR_ERROR_MSG, CRITICAL_FAILURE_MSG);
                break;
            case powerError:
                lcdWriteTwoLines(POWER_ERROR_MSG, CRITICAL_FAILURE_MSG);
                break;
             default:
                // Should never happen
                DEBUG_PRINT("[LCD] Unknown critical failure.\r\n");
                lcdWriteTwoLines(UNKNOWN_ERROR_MSG, CRITICAL_FAILURE_MSG);
        }
    } else if (globalState == welcome_wait_cal) {
        switch (alarmCause) {
            case noError:
                lcdWriteTwoLines(WAIT_CALI_MSG1, WAIT_CALI_MSG2);
                break;
            case calibPatientConnected:
                lcdWriteTwoLines(PAT_CONNECTED_MSG, RETRY_CALI_MSG);
                break;
            case calibIncorrectFlow:
                lcdWriteTwoLines(INC_FLOW_MSG, RETRY_CALI_MSG);
                break;
            default:
                // Should never happen
                DEBUG_PRINT("[LCD] Unknown calib error.\r\n");
                lcdWriteTwoLines(UNKNOWN_ERROR_MSG, RETRY_CALI_MSG);
        }
    } else if(globalState == calibration) {
        lcdWriteTwoLines(CALI_MSG1, CALI_MSG2);
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
    lcd_write_string(errorCode[alarmCause], 1, 10, NO_CR_LF);
}

static void processDisplayNotification(uint32_t notification) {
    // Update parameters
    if (notification & DISP_NOTIF_PARAM) {
        DEBUG_PRINT("[LCD] Rcvd NOTIF_PARAM.\r\n");
        refreshParametersZone();
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

static void lcdWriteTwoLines(char * firstLine, char * secondLine) {
    lcd_write_string(firstLine, 1, 1, NO_CR_LF);
    lcd_write_string(secondLine, 2, 1, NO_CR_LF);
}
