#ifndef BUZZER_H_
#define BUZZER_H_
/** Task in charge of generating the buzzer sound.
 */
#include "FreeRTOS.h"


/** @init_buzzer Initialize the state of the task.
 * Should be called before starting the real-time OS.
 */
void init_buzzer();

/** @notify_buzzer To be called when the alarm state changes.
 */
void notify_buzzer();

/** @play_tone Play a tone once.
 *
 * @param frequency Frequency of the tone (in Hz).
 * @param time Duration of the tone (in ms).
 */
void play_tone(uint16_t frequency, uint16_t time);

/**
 * @BuzzerTask Buzzer sound generation handling task.
 *
 * @param pvParameters Set to NULL.
 */
void BuzzerTask(void *pvParameters);

#endif // BUZZER_H_
