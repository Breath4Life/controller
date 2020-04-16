#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/tone.h"
#include "hal/time.h"
#include "core/buzzer.h"
#include "core/system.h"
#include "core/debug.h"
#include "core/main_task.h"
#include "core/utils.h"

#define N_TONES_CRIT 5
#define N_TONES_HIGH 5
#define N_TONES_MED 3

static const uint16_t tones_freq_crit[N_TONES_CRIT] = { 523, 262, 262, 523, 262 };
static const uint16_t tones_dur_crit[N_TONES_CRIT] = { 150, 150, 150, 150, 150 };
static const uint16_t tones_pause_crit[N_TONES_CRIT] = { 120, 120, 400, 120, 2000 };

static const uint16_t tones_freq_high[N_TONES_HIGH] = { 262, 440, 349, 440, 349 };
static const uint16_t tones_dur_high[N_TONES_HIGH] = { 150, 150, 150, 150, 150 };
static const uint16_t tones_pause_high[N_TONES_HIGH] = { 120, 120, 400, 120, 2000 };

static const uint16_t tones_freq_med[N_TONES_MED] = { 262, 440, 349 };
static const uint16_t tones_dur_med[N_TONES_MED] = { 250, 250, 250 };
static const uint16_t tones_pause_med[N_TONES_MED] = { 120, 120, 2000 };

static const uint16_t *tones_freq[] = { NULL, tones_freq_high, tones_freq_med, tones_freq_crit};
static const uint16_t *tones_dur[] = { NULL, tones_dur_high, tones_dur_med, tones_dur_crit};
static const uint16_t *tones_pause[] = { NULL, tones_pause_high, tones_pause_med, tones_pause_crit};
static const uint8_t tones_size[] = { 0, N_TONES_HIGH, N_TONES_MED, N_TONES_CRIT};

static AlarmState_t buzzer_alarm_state;
static uint8_t muted;

// Doing a pause between two sounds ?
static uint8_t pausing;

// Time of the last update
static uint32_t last_upd_us;

// Offset in the tone sequence
static uint8_t seq_offset;

#define TONE_DUR(alarm, offset) (((int32_t) tones_dur[(alarm)][(offset)])*1000)
#define TONE_PAUSE(alarm, offset) (((int32_t) tones_pause[(alarm)][(offset)])*1000)
// upper bound on the duration of any alarm beep/pause
#define MAX_ALARM_DUR (4000L*1000L)

#define DEBUG_ALARM 0

void init_buzzer() {
    tone_init();
    buzzer_alarm_state = noAlarm;
    muted = 0;
    last_upd_us = 0;
    seq_offset = 0;
    pausing = 0;
}

void BuzzerTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

#if DEBUG_ALARM
    debug_print("[ALARM] Starting.\r\n");
#endif

    while (1) {
        uint32_t curr_time = time_us();
        if ((buzzer_alarm_state != alarmState) || (muted != mute_on)) {
#if DEBUG_ALARM
            debug_print("[ALARM] New alarm/mute.\r\n");
#endif
            tone_stop();
            buzzer_alarm_state = alarmState;
            muted = mute_on;
            // Do as if we were at the end of the sequence
            last_upd_us = curr_time - MAX_ALARM_DUR;
            pausing = 1;
            seq_offset = tones_size[buzzer_alarm_state];
        }
        if ((buzzer_alarm_state != noAlarm) && !muted) {
            if (pausing) {
               if (curr_time-last_upd_us > TONE_PAUSE(buzzer_alarm_state, seq_offset)) {
#if DEBUG_ALARM
                    debug_print("[ALARM] Unpausing.\r\n");
#endif
                    pausing = 0;
                    last_upd_us = curr_time;
                    seq_offset += 1;
                    if (seq_offset >= tones_size[buzzer_alarm_state]) {
                        seq_offset = 0;
                    }
                    tone_start(tones_freq[buzzer_alarm_state][seq_offset]);
               }
            } else {
               if ((curr_time-last_upd_us) > TONE_DUR(buzzer_alarm_state, seq_offset)) {
#if DEBUG_ALARM
                    debug_print("[ALARM] Pausing.\r\n");
#endif
                    pausing = 1;
                    last_upd_us = curr_time;
                    tone_stop();
               }
            }
        }

        // TODO replace this with a wait for notify
        vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
}
