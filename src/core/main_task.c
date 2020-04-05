
#include "FreeRTOS.h"
#include "task.h"

#include "core/debug.h"
#include "core/system.h"
#include "core/buttons.h"
#include "core/main_task.h"
#include "core/motor_control.h"
#include "core/utils.h"

#include "hal/io.h"
#include "hal/pins.h"

GlobalState_t globalState;
AlarmState_t alarmState;
ErrorCode_t errorCode;

uint8_t mute_on;
TickType_t mute_time;

uint8_t tidal_vol;
uint8_t bpm;

void initMainTask()
{
    globalState = welcome;
    alarmState = noAlarm;
    errorCode = noError;
    mute_on = 0;
    tidal_vol = DEFAULT_TIDAL_VOL;
    bpm = DEFAULT_BPM;
    initButtons();
}

void MainTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

    // TODO notify Buzzer task of Welcome
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(4000));
    globalState = welcome_wait_cal;

    while (1) {
        ////////////// 1. READ BUTTONS ///////////
        ButtonsState buttons_pressed = poll_buttons();
        //debug_print("polling buttons %u\r\n", buttons_pressed);
        for (uint8_t i=0; i < N_BUTTONS; i++) {
            if (BUTTON_PRESSED(buttons_pressed, i)) {
                debug_print("Button %s pressed\r\n", buttons_descr[i]);
            }
        }
        ////////////// 2. MUTE ///////////
        if (BUTTON_PRESSED(buttons_pressed, button_alarm_mute)) {
            mute_on = !mute_on;
            mute_time = xTaskGetTickCount();
            // TODO notify buzzer task of mute
        }
        ////////////// 3. ACK ///////////
        if (BUTTON_PRESSED(buttons_pressed, button_alarm_ack)) {
            alarmState = noAlarm;
            errorCode = noError;
            // TODO notify buzzer task of ack
        }
        ////////////// 4. Welcome ///////////
        // Done before the loop.
        ////////////// 5. Welcome wait cal ///////////
        if (globalState == welcome_wait_cal && BUTTON_PRESSED(buttons_pressed, button_startstop)) {
            globalState = calibration;
            // TODO notify motor task of calibration
        }
        ////////////// 6. End of calibration ///////////
        if (globalState == calibration && motorState == motorStopped) {
            globalState = stop;
        }
        ////////////// 7. Settings  & Start/Stop ///////////
        if (globalState == stop || motorState == run) {
            uint8_t updated_setting = 0;
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
            // TODO other settings
            if (updated_setting) {
                // TODO notify LCD setting
            }
            if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                if (globalState == stop) {
                    globalState = run;
                    // TODO notify motor start
                    // TODO notify LCD start
                }
                if (globalState == run) {
                    globalState = stop;
                    // TODO notify motor halt
                    // TODO notify LCD stop
                }
            }
        }
        ////////////// 8. Buzzer auto-unmute ///////////
        if (mute_on && xLastWakeTime - mute_time > pdMS_TO_TICKS(ALARM_AUTO_UNMUTE_SEC)) {
            mute_on = 0;
            // TODO notify buzzer task of mute
        }
        // TODO 9. - 12.

        vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
}


