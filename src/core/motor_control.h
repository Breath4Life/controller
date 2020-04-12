#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "FreeRTOS.h"

#define MOTOR_NOTIF_LIM_DOWN 0x01
#define MOTOR_NOTIF_LIM_UP 0x02
#define MOTOR_NOTIF_MOVEMENT_FINISHED 0x08
#define MOTOR_NOTIF_GLOBAL_STATE 0x10
#define MOTOR_NOTIF_OVER_PRESSURE 0x80

#define MOTOR_USTEPS 1

// Threshold used in the flow-check calibration [ml]
#define VOLUME_CHECK_THRESHOLD 500

typedef enum {
    motorInit,
    motorCalibrating,
    motorStopped,
    motorRunning,
    motorError,
} MotorState_t;

typedef enum {
    calibStart,
    calibDown,
    calibDownWaitStop,
    calibUp,
    calibUpWaitStop,
    calibPosEnd,
    calibVol,
    calibVolEnd
} CalibState_t;

typedef enum {
    insp,
    plateau,
    exp,
    cycleEnd,
    startNewCycle,
    stopping,
    reCalibUp,
    reCalibUpWaitStop,
    reCalibHome
} BreathState_t;

extern volatile MotorState_t motorState;
extern volatile CalibState_t calibState;
extern volatile BreathState_t breathState;

void init_motor();

/**
 * @MotorControlTask Control the motor.
 *
 * @param pvParameters Set to NULL.
 */
void MotorControlTask(void *pvParameters);


#endif // MOTOR_CONTROL_H_
