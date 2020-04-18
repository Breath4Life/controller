#include "FreeRTOS.h"
#include "task.h"

#include "core/system.h"
#include "core/buttons.h"
#include "core/main_task.h"
#include "core/motor_control.h"
#include "core/utils.h"
#include "core/display.h"
#include "core/volume.h"
#include "core/buzzer.h"
#include "core/alarm.h"
#include "core/parameters.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/power_monitoring.h"
#include "hal/door_open.h"

#define CURR_DEBUG_PREFIX mainTask
#include "core/debug.h"

volatile GlobalState_t globalState;


#if DEBUG_MAIN
#define DEBUG_PRINT debug_print_prefix
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
    initButtons();
    initParameters();
}

void MainTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    DEBUG_PRINT("-> welcome");

    // TODO SPEC and adjust this
    play_tone(440, 500, false);

    vTaskDelayUntil(&xLastWakeTime, WELCOME_MSG_DUR);
    /*
     * 4. If globalState is welcome and WELCOME_MSG_DUR s have elapsed,
     * set globalState to welcome_wait_cal
     */
    globalState = welcome_wait_cal;
    DEBUG_PRINT("-> welcome_wait_cal");
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
    DEBUG_PRINT("NOTIF_STATE -> LCD");

    // Indicate if the state has changed
    uint8_t updated_state;
    // Received notifications from other tasks
    uint32_t notification = 0;
    BaseType_t notif_recv = pdFALSE;

#if SIM_MOTOR
    uint32_t calib_start = 0;
#endif

    while (1) {
        // 0. If globalState is critical failure, full restart required, do nothing.
        if (globalState == critical_failure) {
            DEBUG_PRINT("Critically failed");
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
            continue;
        }
        if (alarmLevel == criticalPriorityAlarm && 
                alarmCause != calibPatientConnected && 
                alarmCause != calibIncorrectFlow ) {
            DEBUG_PRINT("-> critical_failure");
            globalState = critical_failure;
            xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
            DEBUG_PRINT("NOT_STATE -> LCD");
        }
        updated_state = 0;
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

                DEBUG_PRINT("-> calibration");
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
                DEBUG_PRINT("-> welcome_wait_cal");

                xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
                DEBUG_PRINT("NOT_STATE -> LCD");
            }

#if SIM_MOTOR
            if (xTaskGetTickCount() - calib_start > 2*WELCOME_MSG_DUR) {
#else
            if (motorState == motorStopped) {
#endif
                globalState = stop;
                DEBUG_PRINT("-> stop");

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
                    DEBUG_PRINT("-> welcome_wait_cal");
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
                }
            }
        }
       /*
        * 7. If globalState is stop or run:
        */
        else if (globalState == stop || globalState == run) {
            upd_params(buttons_pressed);

            /*
             * 7b. If stop/start button was pressed down, update globalState
             * accordingly
             */
            if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                if (globalState == stop) {
                    globalState = run;
                    DEBUG_PRINT("-> run");
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
                } else if (globalState == run) {
                    globalState = stop;
                    DEBUG_PRINT("-> stop");
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
                }
            }
        }

        // 8. Update the alarm state if needed (muting and new alarms)
        pollAlarm();

        /*
         * Common to any changing state, notify LCD of a state change
         */
        if (updated_state == 1) {
            xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
            DEBUG_PRINT("NOTIF_STATE -> LCD");
        }

        /*
         * 11. Check BATTERY_LOW signal
         */
        if (error_power_aux()) {
#if POWER_AUX_CHECK
            DEBUG_PRINT("POWER AUX ERROR");
            sendNewAlarm(auxPower);
#endif
        }

        /*
         * 12. Check POWER_FAIL signal
         */
        if (error_power_main()) {
#if POWER_MAIN_CHECK
            DEBUG_PRINT("POWER MAIN ERROR");
            sendNewAlarm(powerError);
#endif
        }

        /*
         * 13. Check if casing door is open
         */
        if (is_door_open()) {
#if DOOR_CHECK
            DEBUG_PRINT("DOOR OPEN");
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
