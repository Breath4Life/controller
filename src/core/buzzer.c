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

#define BUZZER_NOTIF_ALARM_STATE_CHANGE 0x1

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

// Offset in the tone sequence
static uint8_t seq_offset;

#define TONE_DUR(alarm, offset) (((int32_t) tones_dur[(alarm)][(offset)])*1000)
#define TONE_PAUSE(alarm, offset) (((int32_t) tones_pause[(alarm)][(offset)])*1000)
// upper bound on the duration of any alarm beep/pause
#define MAX_ALARM_DUR (4000L*1000L)

#define DEBUG_BUZZER 0
#if DEBUG_BUZZER
#define BUZZER_DEBUG_PRINT debug_print
#else
#define BUZZER_DEBUG_PRINT fake_debug_print
#endif

void init_buzzer() {
    tone_init();
    buzzer_alarm_state = noAlarm;
    muted = 0;
    seq_offset = 0;
    pausing = 0;
}

static TimeOut_t timeOutBoundedWait;
static TickType_t boundedWaitTime;

void notify_buzzer() {
    xTaskNotify(buzzerTaskHandle, BUZZER_NOTIF_ALARM_STATE_CHANGE, eSetBits);
}

static void set_time_next_wakeup(uint16_t time) {
    vTaskSetTimeOutState(&timeOutBoundedWait);
    boundedWaitTime = pdMS_TO_TICKS(time);
}

void play_tone(uint16_t frequency, uint16_t time) {
    tone_start(frequency);
    set_time_next_wakeup(time);
}

void BuzzerTask(void *pvParameters)
{
    BUZZER_DEBUG_PRINT("[ALARM] Starting.\r\n");

    while (1) {
        if ((buzzer_alarm_state != alarmState) || (muted != mute_on)) {
            BUZZER_DEBUG_PRINT("[ALARM] New alarm/mute.\r\n");
            tone_stop();
            buzzer_alarm_state = alarmState;
            muted = mute_on;
            // Do as if we were at the end of the sequence
            pausing = 1;
            seq_offset = tones_size[buzzer_alarm_state];
            set_time_next_wakeup(0);
        }
        if (xTaskCheckForTimeOut(&timeOutBoundedWait, &boundedWaitTime)) {
            tone_stop();
            if ((buzzer_alarm_state != noAlarm) && !muted) {
                if (pausing) {
                    BUZZER_DEBUG_PRINT("[ALARM] Unpausing.\r\n");
                    pausing = 0;
                    seq_offset += 1;
                    if (seq_offset >= tones_size[buzzer_alarm_state]) {
                        seq_offset = 0;
                    }
                    tone_start(tones_freq[buzzer_alarm_state][seq_offset]);
                    set_time_next_wakeup(tones_dur[buzzer_alarm_state][seq_offset]);
                } else {
                    BUZZER_DEBUG_PRINT("[ALARM] Pausing.\r\n");
                    pausing = 1;
                    set_time_next_wakeup(tones_pause[buzzer_alarm_state][seq_offset]);
                }
            } else {
                // no wake-up needed
                set_time_next_wakeup(1000);
            }
        }

        uint32_t notif_recv;
        xTaskNotifyWait(0x0,ALL_NOTIF_BITS,&notif_recv,boundedWaitTime);
    }
}
