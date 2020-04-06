#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "FreeRTOS.h"

typedef enum {
    motorStopped,
    motorRunning,
} MotorState_t;

extern MotorState_t motorState;


/**
 * @MotorControlTask Control the motor.
 *
 * @param pvParameters Set to NULL.
 */
void MotorControlTask(void *pvParameters);

#endif // MOTOR_CONTROL_H_
