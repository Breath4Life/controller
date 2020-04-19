/** Alarm management
 */
#ifndef ALARM_H_
#define ALARM_H_

#include <stdbool.h>

#define ALARM_CHECK 1           // active/deactivate alarm check for debug
#define CALIB_ERROR_CHECK 1     // active/deactivate calib error check during calib for debug

#define POWER_AUX_CHECK 0       // active/deactivate power aux check for debug
#define POWER_MAIN_CHECK 0      // active/deactivate power main check for debug
#define DOOR_CHECK 1            // active/deactivate door check for debug

typedef enum {
    noAlarm,
    mediumPriorityAlarm,
    highPriorityAlarm,
    criticalPriorityAlarm
} AlarmLevel_t;

#define ERROR_TABLE \
    X(noError, "noError", noAlarm) \
    X(overPressure, "overPressure", highPriorityAlarm) \
    X(noPressure, "noPressure", highPriorityAlarm) \
    X(highTemperature, "highTemperature", highPriorityAlarm) \
    X(lowPressure, "lowPressure", mediumPriorityAlarm) \
    X(abnVolume, "abnVolume", mediumPriorityAlarm) \
    X(abnFreq, "abnFreq", mediumPriorityAlarm) \
    X(auxPower, "auxPower", mediumPriorityAlarm) \
    X(calibPatientConnected, "calibPatientConnected", highPriorityAlarm) \
    X(calibIncorrectFlow, "calibIncorrectFlow", highPriorityAlarm) \
    X(doorOpen, "doorOpen", criticalPriorityAlarm) \
    X(cfMotorError, "cfMotorError", criticalPriorityAlarm) \
    X(powerError, "powerError", criticalPriorityAlarm)

#define X(a, b, c) a,
typedef enum {
    ERROR_TABLE
} AlarmCause_t;
#undef X

extern volatile AlarmLevel_t alarmLevel;
extern volatile AlarmCause_t alarmCause;
extern volatile bool alarmMuted;

void initAlarm();
void mutePressed();
void pollAlarm();
void sendNewAlarm(AlarmCause_t alarm);
void ackAlarm();
void set_alarm_led(AlarmLevel_t state);


#endif // ALARM_H_
