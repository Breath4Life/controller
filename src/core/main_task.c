#include "FreeRTOS.h"
#include "task.h"

#include "core/debug.h"
#include "core/system.h"
#include "core/buttons.h"
#include "core/main_task.h"
#include "core/motor_control.h"
#include "core/utils.h"
#include "core/display.h"
#include "core/volume.h"
#include "core/buzzer.h"
#include "core/alarm.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/power_monitoring.h"
#include "hal/door_open.h"

volatile GlobalState_t globalState;

// current parameters
uint8_t tidal_vol; // tens of mL
uint8_t bpm;
uint8_t ie;
uint8_t p_max;
uint8_t extra_param;

// last confirmed parameters
uint8_t saved_tidal_vol; // tens of mL
uint8_t saved_bpm;
uint8_t saved_ie;
uint8_t saved_p_max;

static void save_parameters();
static void revert_parameters();
TickType_t last_update_time;

#if DEBUG_MAIN
#define DEBUG_PRINT debug_print
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_MAIN

#define SIM_MOTOR 0             // "simulate" motor to debug the rest
#define ALARM_CHECK 1          // active/deactivate alarm check for debug
#define CALIB_ERROR_CHECK 1     // active/deactivate calib error check during calib for debug

#define POWER_AUX_CHECK 0       // active/deactivate power aux check for debug
#define POWER_MAIN_CHECK 0      // active/deactivate power main check for debug
#define DOOR_CHECK 0            // active/deactivate door check for debug

void initMainTask()
{
    globalState = welcome;

    tidal_vol = DEFAULT_TIDAL_VOL;
    bpm = DEFAULT_BPM;
    ie = DEFAULT_IE;
    p_max = DEFAULT_PMAX;
    save_parameters();

    extra_param = 0;

    initButtons();
}

void MainTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    DEBUG_PRINT("[MAIN] -> welcome.\r\n");

    // TODO SPEC and adjust this
    play_tone(440, 500);

    vTaskDelayUntil(&xLastWakeTime, WELCOME_MSG_DUR);
    /*
     * 4. If globalState is welcome and WELCOME_MSG_DUR s have elapsed,
     * set globalState to welcome_wait_cal
     */
    globalState = welcome_wait_cal;
    DEBUG_PRINT("[MAIN] -> welcome_wait_cal.\r\n");
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
    DEBUG_PRINT("[MAIN] NOTIF_STATE -> LCD.\r\n");

    // Indicate if the state has changed
    uint8_t updated_state;
    // Indicate if settings have changed
    uint8_t updated_setting;
    // Received notifications from other tasks
    uint32_t notification = 0;
    BaseType_t notif_recv = pdFALSE;

#if SIM_MOTOR
    uint32_t calib_start = 0;
#endif

    while (1) {
        // 0. If globalState is critical failure, full restart required, do nothing.
        if (globalState == critical_failure) {
            DEBUG_PRINT("[MAIN] Critically failed.\r\n");
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
            continue;
        }
        if (alarmLevel == criticalPriorityAlarm && 
                alarmCause != calibPatientConnected && 
                alarmCause != calibIncorrectFlow ) {
            DEBUG_PRINT("[MAIN] -> critical_failure.\r\n");
            globalState = critical_failure;
            xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
            DEBUG_PRINT("[MAIN] NOT_STATE -> LCD. \r\n");
        }
        updated_state = 0;
        updated_setting = 0;
        /*
         * TODO:
         * - notif_LCD_to_send variable that is |=ed at different point
         *   in the code, with a single xTaskNotify at the end of the loop
         */

        // 1. Read buttons
        ButtonsState buttons_pressed = poll_buttons();

        // 2. If the MUTE button was pressed down: toggle mute_on and record time
        if (BUTTON_PRESSED(buttons_pressed, button_alarm_mute)) {
            mutePressed();
        }

        // 3. If the ACK button was pressed down: new alarm state is noError
        if (BUTTON_PRESSED(buttons_pressed, button_alarm_ack)) {
            ackAlarm();
        }

        /*
         * 5. If globalState is welcome_wait_cal and start/stop button was
         * pressed down: set globalState to calibration and notify MotorTask
         * to start calibration
         */
        if (globalState == welcome_wait_cal) {
           if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                globalState = calibration;
                // reset to noAlarm
                ackAlarm();

                DEBUG_PRINT("[MAIN] -> calibration.\r\n");
                updated_state = 1;

                xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);

#if SIM_MOTOR
                calib_start = xTaskGetTickCount();
#endif
            }
        }
        // 6. If globalState is calibration:
        else if (globalState == calibration) {
            /*
             * 6a. If motorState is motorStopped, set globalState to stop and notify
             * LCD that calibration is done (display settings + measured parameters
             * + state).
             */
            if (alarmLevel == criticalPriorityAlarm) {
                globalState = welcome_wait_cal;
                DEBUG_PRINT("[MAIN] -> welcome_wait_cal.\r\n");

                xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
                DEBUG_PRINT("[MAIN] NOT_STATE -> LCD. \r\n");
            }

#if SIM_MOTOR
            if (xTaskGetTickCount() - calib_start > 2*WELCOME_MSG_DUR) {
#else
            if (motorState == motorStopped) {
#endif
                globalState = stop;
                DEBUG_PRINT("[MAIN] -> stop.\r\n");

                xTaskNotify(lcdDisplayTaskHandle,
                        DISP_NOTIF_STATE | DISP_NOTIF_PARAM |
                        DISP_NOTIF_PLATEAU_P | DISP_NOTIF_PEAK_P, eSetBits);
            }
            /*
             * 6b. If motorState is calibrating and the start/stop button was pressed
             * down, set globalState to welcome_wait_cal and notify motor of HALT
             */

#if SIM_MOTOR
            else {
#else
            else if (motorState == motorCalibrating) {
#endif
                if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                    globalState = welcome_wait_cal;
                    DEBUG_PRINT("[MAIN] -> welcome_wait_cal.\r\n");
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
                }
            }
        }
       /*
        * 7. If globalState is stop or run:
        */
        else if (globalState == stop || globalState == run) {
            /*
             * 7a. If any setting button was pressed down, update the corresponding
             * state and notify LCD
             */
            if (BUTTON_PRESSED(buttons_pressed, button_vtidal_up)) {
                tidal_vol = MIN(MAX_TIDAL_VOL, tidal_vol + INC_TIDAL_VOL);
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_vtidal_down)) {
                tidal_vol = MAX(MIN_TIDAL_VOL, tidal_vol - INC_TIDAL_VOL);
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_freq_respi_up)) {
                bpm = MIN(MAX_BPM, bpm + INC_BPM);
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_freq_respi_down)) {
                bpm = MAX(MIN_BPM, bpm - INC_BPM);
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_next) && !unsaved_extra_param()) {
                extra_param = (extra_param + 1) % N_EXTRA;
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_confirm)) {
                DEBUG_PRINT("[MAIN] Current parameters saved.\r\n");
                save_parameters();
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_up)) {
                if (extra_param == 0) {
                    ie = MIN(MAX_IE, ie + INC_IE);
                    updated_setting = 1;
                } else if (extra_param == 1) {
                    p_max = MIN(MAX_PMAX, p_max + INC_PMAX);
                    updated_setting = 1;
                } else if (extra_param == 2) {
                    // PEEP, not settable
                }
            }
            if (BUTTON_PRESSED(buttons_pressed, button_down)) {
                if (extra_param == 0) {
                    ie = MAX(MIN_IE, ie - INC_IE);
                    updated_setting = 1;
                } else if (extra_param == 1) {
                    p_max = MAX(MIN_PMAX, p_max - INC_PMAX);
                    updated_setting = 1;
                } else if (extra_param == 2) {
                    // PEEP, not settable
                }
             }

            if (updated_setting) {
                DEBUG_PRINT("[MAIN] NOTIF_PARAM -> LCD.\r\n");
                DEBUG_PRINT("[MAIN] Parameters changed.\r\n");
                last_update_time = xTaskGetTickCount();
                xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PARAM, eSetBits);
            }

            /*
             * 7b. If stop/start button was pressed down, update globalState
             * accordingly
             */
            if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                if (globalState == stop) {
                    globalState = run;
                    DEBUG_PRINT("[MAIN] -> run.\r\n");
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
                } else if (globalState == run) {
                    globalState = stop;
                    DEBUG_PRINT("[MAIN] -> stop.\r\n");
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
                }
            }
        }

        // 8. Update the alarm state if needed (muting and new alarms)
        pollAlarm();

        /*
         * If PARAM_AUTO_REVERT_DELAY has elapsed, revert current parameters to their last saved state
         */
        if (unsaved_parameters() && xTaskGetTickCount() - last_update_time > pdMS_TO_TICKS(PARAM_AUTO_REVERT_DELAY)) {
            revert_parameters();
            xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PARAM, eSetBits);
        }

        /*
         * Common to any changing state, notify LCD of a state change
         */
        if (updated_state == 1) {
            xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
            DEBUG_PRINT("[MAIN] NOTIF_STATE -> LCD. \r\n");
        }

        /*
         * 11. Check BATTERY_LOW signal
         */
        if (error_power_aux()) {
#if POWER_AUX_CHECK
            DEBUG_PRINT("[MAIN] POWER AUX ERROR.\r\n");
            sendNewAlarm(auxPower);
#endif
        }

        /*
         * 12. Check POWER_FAIL signal
         */
        if (error_power_main()) {
#if POWER_MAIN_CHECK
            DEBUG_PRINT("[MAIN] POWER MAIN ERROR.\r\n");
            sendNewAlarm(powerError);
#endif
        }

        /*
         * 13. Check if casing door is open
         */
        if (is_door_open()) {
#if DOOR_CHECK
            DEBUG_PRINT("[MAIN] DOOR OPEN.\r\n");
            sendNewAlarm(doorOpen);
#endif
        }

        /*
         * 14. EEPROM total operating writing
         */
        // TODO

        /*
         * 15. Poll volume sensing.
         */
        poll_volume();

        /*
         * 16. Bounded wait for notification (10ms)
         */
        notif_recv = xTaskNotifyWait(0x0, ALL_NOTIF_BITS, &notification, pdMS_TO_TICKS(10));
    }
}

uint8_t stoppedOrRunning() {
    return (globalState == stop || globalState == run);
}

// TODO: move
void check_volume(uint32_t actual_vol) {
    // Convert target tidal volume to tens µl
    int32_t target_vol = tidal_vol * 1000L;

    // Check that measured volume is within +-10% of the target volume
    if((actual_vol > 11*target_vol) || (actual_vol < 9*target_vol)) {
        sendNewAlarm(abnVolume);
    }
}

// TODO: move what's below in parameters.c

/*
 * Returns 1 if at least one parameter is unsaved/unconfirmed,
 * 0 otherwise.
 */
uint8_t unsaved_parameters() {
    return  (tidal_vol != saved_tidal_vol) ||
            (bpm != saved_bpm) ||
            (ie != saved_ie) ||
            (p_max != saved_p_max);
}

/*
 * Returns 1 if tidal_volume is unsaved/unconfirmed, 0 otherwise.
 */
uint8_t unsaved_tidal_vol() {
    return tidal_vol != saved_tidal_vol;
}

/*
 * Returns 1 if bpm is unsaved/unconfirmed, 0 otherwise.
 */
uint8_t unsaved_bpm() {
    return bpm != saved_bpm;
}

/*
 * Returns 1 if at least one of the extra parameters is
 * unsaved/unconfirmed, 0 otherwise.
 */
uint8_t unsaved_extra_param() {
    return  (ie != saved_ie) ||
            (p_max != saved_p_max);
}


/*
 * Saves/confirms the current parameters.
 */
static void save_parameters() {
    saved_tidal_vol = tidal_vol;
    saved_bpm = bpm;
    saved_ie = ie;
    saved_p_max = p_max;
}

/*
 * Reverts the current parameters to their latest saved state.
 */
static void revert_parameters() {
    tidal_vol = saved_tidal_vol;
    bpm = saved_bpm;
    ie = saved_ie;
    p_max = saved_p_max;
}
