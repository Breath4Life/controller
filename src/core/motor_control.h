#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "FreeRTOS.h"

#define MOTOR_NOTIF_LIM_DOWN 0x01
#define MOTOR_NOTIF_LIM_UP 0x02
#define MOTOR_NOTIF_MOVEMENT_FINISHED 0x08
#define MOTOR_NOTIF_GLOBAL_STATE 0x10
#define MOTOR_NOTIF_OVER_PRESSURE 0x80

#define MOTOR_USTEPS 1

// Threshold used in the flow-check calibration [ul]
// FIXME put 600000
#define VOLUME_CHECK_THRESHOLD 400000L

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
    calibVolEnd,
    calibWaitStopAbort
} CalibState_t;

typedef enum {
    // inspiration
    insp,
    // plateau
    plateau,
    // expiration
    expiration,
    // resting before inspiration
    cycleEnd,
    // set parameters just before inspiration
    startNewCycle,
    // moving to rest position after a stop command
    stopping,
    // moving up for re-calibration
    reCalibUp,
    // touched the upper limit switch while reCalibUp, now waiting for motor to stop
    reCalibUpWaitStop,
    // reCalibration: after touching the limit switch up, moving to rest position
    reCalibHome,
    // after touching the down limit swith during expiration, new waiting for the motor to stop
    expStopping,
    // for recalibration, moving a little bit down to distance from the up limit switch
    reCalibDown,
    // got a stop command, waiting for the motor to stop before going in the stopping state
    preStopping,
    // touched limit switch down while insp, stopping before expiration
    inspStopping,
} BreathState_t;

typedef enum {
    errorStopping,
    errorStopped,
} MotorErrorState_t;

extern volatile MotorState_t motorState;
extern volatile CalibState_t calibState;
extern volatile BreathState_t breathState;
extern volatile MotorErrorState_t motorErrorState;

void init_motor();

/**
 * @MotorControlTask Control the motor.
 *
 * @param pvParameters Set to NULL.
 */
void MotorControlTask(void *pvParameters);


#endif // MOTOR_CONTROL_H_
