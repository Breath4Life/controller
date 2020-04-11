#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "core/main_task.h"
#include "core/system.h"
#include "core/utils.h"
#include "hal/pins.h"
#include "hal/motor.h"
#include "core/debug.h"
#include "core/motor_control.h"
#include "core/volume.h"
#include "hal/limit_switch.h"


// TODO
// * delay motor disable to prevent overrunning
// * fix timeout values

#define DEBUG_MOTOR 0
#if DEBUG_MOTOR
#define MOTOR_DEBUG_PRINT debug_print
#else
#define MOTOR_DEBUG_PRINT fake_debug_print
#endif // DEBUG_MOTOR

#define MOTOR_NOTIF_LIM (MOTOR_NOTIF_LIM_UP | MOTOR_NOTIF_LIM_DOWN)
#define MOTOR_NOTIF_CYCLE (MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE | MOTOR_NOTIF_MOVEMENT_FINISHED)

/////////////////////////////////////
uint32_t TCT;
uint32_t Ti;
uint32_t Te;

uint32_t n_steps;
uint32_t tot_pulses;

uint32_t plateau_pulses;
uint32_t insp_pulses;
uint32_t exp_pulses;

uint32_t T_tot_plateau;
uint32_t T_tot_Ti;
uint32_t T_tot_Te;

uint32_t tmp_f_insp;
uint32_t tmp_f_plateau;
uint32_t tmp_f_exp;

uint32_t f_insp;
uint32_t f_plateau;
uint32_t f_exp;

const uint32_t vol_ml = 700;

///////////////////////////////////


volatile MotorState_t motorState;
volatile BreathState_t breathState;
volatile CalibState_t calibState;

uint32_t currentPosition;
uint32_t targetPosition;
uint32_t homePosition;
uint32_t posOffset;

TickType_t previousWakeTime;
uint32_t ticksTctTime;

// TODO reset proper value!
const uint32_t f_home = 100*MOTOR_USTEPS; // steps/s (not µsteps/s)

const int32_t steps_calib_down = 2000;
const int32_t steps_calib_up = 1000;
const int32_t steps_calib_end = 50;
const int32_t steps_calib_vol = 500;
const int32_t steps_recalib = 1000;

const uint32_t thresh_calib_vol_mil = 600;

uint8_t recalibrateFlag;

// Threshold for volume calibraiton

// 0 replaces the whole task by an empty loop for testing purposes
#define MOTOR_ACTIVE 1

// TODO something real
uint32_t vol2steps(uint8_t  tidal_vol){
    return 10*tidal_vol;
}

void compute_config(){
    // Breathing cycles parameter
    TCT = (60000L / bpm); // in ms
    // TODO check for each possibilities
    Ti = (TCT/(1+ie)); // in ms
    Te = TCT - Ti;

    ticksTctTime = pdMS_TO_TICKS(TCT);

    n_steps = vol2steps(tidal_vol);
    tot_pulses = n_steps * MOTOR_USTEPS;

    plateau_pulses = MOTOR_USTEPS*5; //tot_pulses/20;
    insp_pulses = tot_pulses - plateau_pulses;
    exp_pulses = tot_pulses;

    T_tot_plateau = Ti / 5;
    T_tot_Ti = Ti - T_tot_plateau;
    // TODO make this depend on number of pulses ? / "as fast as possible" means speed should not depend on volume
    T_tot_Te = 750; // As fast as possible. Should be less than 1s (for 30 BPM + I/E+1 case)

    // 1000 factor since time in ms and freq in Hz
    tmp_f_insp = insp_pulses * 1000;
    tmp_f_plateau = plateau_pulses * 1000;
    tmp_f_exp = tot_pulses * 1000;

    f_insp = tmp_f_insp / T_tot_Ti;
    f_plateau = tmp_f_plateau / T_tot_plateau;
    f_exp = tmp_f_exp / T_tot_Te;

    MOTOR_DEBUG_PRINT("ie %u \r\n",ie);
    MOTOR_DEBUG_PRINT("bpm %u \r\n",bpm);
    MOTOR_DEBUG_PRINT("TCT %u \r\n",TCT);
    MOTOR_DEBUG_PRINT("Ti %u \r\n",Ti);
    MOTOR_DEBUG_PRINT("nsteps %u \r\n",n_steps);
    MOTOR_DEBUG_PRINT("pulse_lateau %u \r\n",plateau_pulses);
    MOTOR_DEBUG_PRINT("Ttot plate %u \r\n",T_tot_plateau);
}

void init_motor() {
    init_limit_switch();
    setup_motor();

    motorState = motorInit;
    breathState = startNewCycle;
    calibState = calibDown;
    currentPosition = 0;
    targetPosition = 0;
    homePosition = 0;
    posOffset = 0;
    recalibrateFlag = 0;
}

static void genMotorError() {
    motor_anticipated_stop();
    motor_disable();
    motorState = motorError;
    xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
}

static void motorHaltCalib(char *state) {
    motor_anticipated_stop();
    motor_disable();
    motorState = motorInit;
    MOTOR_DEBUG_PRINT("[MOTOR] halting in %s\r\n", state);
}

static void motorHalt(char *state) {
    motor_anticipated_stop();
    breathState = stopping;
    set_motor_goto_position_accel_exec(homePosition, f_exp, 2, 200);
    MOTOR_DEBUG_PRINT("[MOTOR] halting in %s\r\n", state);
}

static uint32_t boundedWaitNotification(uint32_t admissible_notifications, TickType_t ticksToWait) {
    uint32_t notif_recv;
    if (xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,ticksToWait) == pdTRUE) {
        // received notification
        if (notif_recv & ~admissible_notifications) {
            // invalid notification
            genMotorError();
            MOTOR_DEBUG_PRINT("[MOTOR] Unexp notif %x\r\n", notif_recv);
        } else {
            return notif_recv;
        }
    } else {
        // no notification received, timeout
        genMotorError();
        MOTOR_DEBUG_PRINT("[MOTOR] TIMEOUT\r\n");
    }
    return notif_recv;
}

static uint32_t move_and_wait(uint32_t targetPosition, uint32_t max_freq, uint32_t admissible_notifications) {
    // max speed: 1200 steps/s. Acceleration: 200 steps/s/10ms
    // -> max 3*120 = 360 ms deceleration + acceleration + deceleration (we take 400 to have a margin)
    // total time is thus bounded by 360 ms + 1000*abs(currentPosition-targetPosition) / (max_freq)
    uint16_t max_duration = 400 + (1000*ABS(targetPosition-motor_current_position()))/max_freq;
    set_motor_goto_position_accel_exec(targetPosition, max_freq, 2, 200*MOTOR_USTEPS);
    return boundedWaitNotification(admissible_notifications, pdMS_TO_TICKS(max_duration));

}

#if MOTOR_ACTIVE
void MotorControlTask(void *pvParameters)
{
    while (1)
    {
        uint32_t notif;

        switch (motorState) {
            case motorInit:
                // NON BOUNDED wait for calibrating notification
                xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif,portMAX_DELAY);

                if (notif& MOTOR_NOTIF_START_CALIBRATION) {
                    motorState = motorCalibrating;
                    calibState = calibStart;
                    MOTOR_DEBUG_PRINT("[MOTOR] Start calibration\r\n");
                }
                break;

            case motorCalibrating:
                switch (calibState) {
                    case calibStart:
                        compute_config();
                        calibState = calibDown;
                        set_motor_current_position_value(0);
                        targetPosition = MOTOR_USTEPS*steps_calib_down;
                        motor_enable();
                        MOTOR_DEBUG_PRINT("to calib down\r\n");
                        notif = move_and_wait(targetPosition, f_home,
                                MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE);

                    case calibDown:
                        MOTOR_DEBUG_PRINT("[MOTOR] calibDown\r\n");
                        if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO handle this case
                        } else if (notif & MOTOR_NOTIF_HALT) {
                            motorHaltCalib("calibDown");
                        } else if (notif & MOTOR_NOTIF_LIM_DOWN) {
                            // Finished going down, let's now stop the motor and go up.
                            // Stop motor
                            motor_anticipated_stop();
                            // Compute next position   
                            posOffset = MOTOR_USTEPS*steps_calib_up;
                            // Manually set the absolute position of the motor 
                            // Use this trick to avoid negative absolute positions
                            set_motor_current_position_value(posOffset);
                            targetPosition = 0;
                            calibState = calibUp;
                            notif = move_and_wait(
                                    targetPosition,
                                    f_home, 
                                    MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE);
                            MOTOR_DEBUG_PRINT("[MOTOR] finished calibDown\r\n");
                        } else {
                            // MOTOR_NOTIF_LIM_UP
                            // We were unrolling the rope, we will soon start
                            // enrolling it in the correct direction.
                            // Continue in this state.
                            // TODO fix the max duration
                            notif = boundedWaitNotification(
                                    MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE,
                                    pdMS_TO_TICKS(10000));
                        }
                        break;

                    case calibUp:
                        MOTOR_DEBUG_PRINT("[MOTOR] calibUp\r\n");
                        if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO handle this case
                        } else if (notif & MOTOR_NOTIF_HALT) {
                            motorHaltCalib("calibUp");
                        } else {
                            // case MOTOR_NOTIF_LIM_UP
                            // Stop motor
                            motor_anticipated_stop();
                            // Compute next position   
                            posOffset = MOTOR_USTEPS*steps_calib_end;
                            set_motor_current_position_value(0);

                            calibState = calibPosEnd;
                            MOTOR_DEBUG_PRINT("[MOTOR] finished calibUp\r\n");
                            notif = move_and_wait(
                                    posOffset,
                                    f_home, 
                                    MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE | MOTOR_NOTIF_MOVEMENT_FINISHED);
                        }
                        break;

                    case calibPosEnd:
                        MOTOR_DEBUG_PRINT("[MOTOR] calibPosEnd\r\n");
                        if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO handle this case
                        } else if (notif & MOTOR_NOTIF_HALT) {
                            motorHaltCalib("calibPosEnd");
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            homePosition = motor_current_position();
                            posOffset = MOTOR_USTEPS*steps_calib_vol;

                            targetPosition = homePosition + posOffset;
                            calibState = calibVol;
                            // Reset volume prior to flow check
                            reset_volume();
                            MOTOR_DEBUG_PRINT("[MOTOR] Start flow check.\r\n");
                            notif = move_and_wait(
                                    targetPosition,
                                    steps_calib_vol*MOTOR_USTEPS, 
                                    MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE | MOTOR_NOTIF_MOVEMENT_FINISHED);
                        }
                        break;

                    case calibVol:
                        MOTOR_DEBUG_PRINT("[MOTOR] calibVol\r\n");
                        if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO handle this case
                        } else if (notif & MOTOR_NOTIF_HALT) {
                            motorHaltCalib("calibVol");
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            if (volume > VOLUME_CHECK_THRESHOLD) {
                                // Sufficient volume insufflated.
                                calibState = calibVolEnd;
                                MOTOR_DEBUG_PRINT("[MOTOR] Flow check: OK\r\n");
                            notif = move_and_wait(
                                    homePosition,
                                    steps_calib_vol*MOTOR_USTEPS,
                                    MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE | MOTOR_NOTIF_MOVEMENT_FINISHED);
                            } else {
                                MOTOR_DEBUG_PRINT("[MOTOR] Flow check: FAIL\r\n");
                                xTaskNotify(mainTaskHandle, NOTIF_INCORRECT_FLOW, eSetBits);
                                // TODO: correct ?
                                motorState = motorError;
                            }
                        }
                        break;

                    case calibVolEnd:
                        MOTOR_DEBUG_PRINT("[MOTOR] calibVolEnd\r\n");
                        if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO handle this case
                        } else if (notif & MOTOR_NOTIF_HALT) {
                            motorHaltCalib("calibVolEnd");
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            motor_disable();
                            motorState = motorStopped;
                            MOTOR_DEBUG_PRINT("[MOTOR] END CALIB\r\n");
                        }
                        break;

                }
                break;

            case motorStopped:
                // Non bounded wait for notif 
                xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif,portMAX_DELAY);

                // if notif == START: update motor state
                if (notif& MOTOR_NOTIF_START) {
                    motorState = motorRunning;
                    breathState = startNewCycle;
                    previousWakeTime = xTaskGetTickCount();
                    MOTOR_DEBUG_PRINT("TCT %u \r\n",TCT);
                    MOTOR_DEBUG_PRINT("Ti %u \r\n",Ti);
                    MOTOR_DEBUG_PRINT("tot_pulses %u \r\n",tot_pulses);
                    MOTOR_DEBUG_PRINT("plat pulses %u \r\n",plateau_pulses);
                    MOTOR_DEBUG_PRINT("Ti pulses %u \r\n",insp_pulses);
                    MOTOR_DEBUG_PRINT("TtotTi %u \r\n",T_tot_Ti);
                    MOTOR_DEBUG_PRINT("TtotTe %u \r\n",T_tot_Te);
                    MOTOR_DEBUG_PRINT("f_insp %u \r\n",f_insp);
                    MOTOR_DEBUG_PRINT("f_exp %u \r\n",f_exp);
                }
                break;

            case motorRunning:
                switch (breathState) {
                    case startNewCycle:
                        compute_config();
                        motor_enable();
                        breathState = insp;
                        targetPosition = homePosition + insp_pulses;
                        MOTOR_DEBUG_PRINT("Ti pulses used %u \r\n",targetPosition);
                        MOTOR_DEBUG_PRINT("=> target %u \r\n",targetPosition);
                        MOTOR_DEBUG_PRINT("cur pos %u \r\n",motor_current_position());
                        MOTOR_DEBUG_PRINT("home %u \r\n",homePosition);
                        MOTOR_DEBUG_PRINT("to insp \r\n");
                            notif = move_and_wait(targetPosition, f_insp, MOTOR_NOTIF_CYCLE);
                        break;

                    case insp:
                        if (notif & MOTOR_NOTIF_HALT) {
                            motorHalt("INSP");
                        } else if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO
                        } else if (notif & MOTOR_NOTIF_LIM_UP) {
                            motor_anticipated_stop();
                            recalibrateFlag = 1;
                            breathState = reCalibHome;
                            set_motor_current_position_value(0);
                            targetPosition = MOTOR_USTEPS * steps_calib_end;
                            notif = move_and_wait(targetPosition, f_exp, MOTOR_NOTIF_CYCLE);
                        } else if (notif & MOTOR_NOTIF_LIM_DOWN) {
                            motor_anticipated_stop();
                            recalibrateFlag = 1;
                            breathState = exp;
                            targetPosition = homePosition;
                            notif = move_and_wait(targetPosition, f_exp, MOTOR_NOTIF_CYCLE);
                            MOTOR_DEBUG_PRINT("recalibrate INSP asked \r\n");
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            breathState = plateau;
                            targetPosition = homePosition + insp_pulses + plateau_pulses; 
                            notif = move_and_wait(targetPosition, f_plateau, MOTOR_NOTIF_CYCLE);
                            MOTOR_DEBUG_PRINT("[MOTOR] to plateau\r\n");
                        }
                        break;

                    case plateau:
                        if (notif & MOTOR_NOTIF_HALT) {
                            motorHalt("PLATEAU");
                        } else if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO
                        } else if (notif & MOTOR_NOTIF_LIM_UP) {
                            motor_anticipated_stop();
                            recalibrateFlag = 1;
                            breathState = reCalibHome;
                            set_motor_current_position_value(0);
                            targetPosition = MOTOR_USTEPS * steps_calib_end;
                            notif = move_and_wait(targetPosition, f_exp, MOTOR_NOTIF_CYCLE);
                        } else if (notif & MOTOR_NOTIF_LIM_DOWN) {
                            motor_anticipated_stop();
                            recalibrateFlag = 1;
                            breathState = exp;
                            targetPosition = homePosition;
                            notif = move_and_wait(targetPosition, f_exp, MOTOR_NOTIF_CYCLE);
                            MOTOR_DEBUG_PRINT("recalibrate plateau asked \r\n");
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            // go back to HOME
                            breathState = exp;
                            MOTOR_DEBUG_PRINT("[MOTOR] to exp\r\n");
                            notif = move_and_wait(homePosition, f_exp, MOTOR_NOTIF_CYCLE);
                        }
                        break;

                    case exp:
                        MOTOR_DEBUG_PRINT("in EXP phase \r\n");
                        if (notif & MOTOR_NOTIF_HALT) {
                            motorHalt("EXP");
                        } else if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO
                        } else if (notif & MOTOR_NOTIF_LIM_UP) {
                            motor_anticipated_stop();
                            recalibrateFlag = 1;
                            breathState = reCalibHome;
                            set_motor_current_position_value(0);
                            targetPosition = MOTOR_USTEPS * steps_calib_end;
                            notif = move_and_wait(targetPosition, f_exp, MOTOR_NOTIF_CYCLE);
                        } else if (notif & MOTOR_NOTIF_LIM_DOWN) {
                            // TODO
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            if (recalibrateFlag) {
                                breathState = reCalibUp;
                                currentPosition = motor_current_position();
                                posOffset = MOTOR_USTEPS * steps_recalib; 
                                if (currentPosition < posOffset){
                                    set_motor_current_position_value(posOffset);
                                    targetPosition = 0;
                                } else {
                                    targetPosition = currentPosition - posOffset; 
                                }
                                notif = move_and_wait(targetPosition, f_exp, MOTOR_NOTIF_CYCLE);

                            } else {
                                breathState = cycleEnd;
                                motor_disable();
                                MOTOR_DEBUG_PRINT("[MOTOR] to wait cycle end\r\n");
                            }
                        }
			break;

                    case reCalibUp:
                        MOTOR_DEBUG_PRINT("in reCalibUp phase \r\n");
                        if (notif & MOTOR_NOTIF_HALT) {
                            motorHalt("reCalibUp");
                        } else if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO
                        } else if (notif & MOTOR_NOTIF_LIM_UP) {
                            // Stop motor
                            motor_anticipated_stop();
                            // Compute actual position when notified
                            set_motor_current_position_value(0);
                            // Compute next position
                            targetPosition = MOTOR_USTEPS*steps_calib_end;
                            breathState = reCalibHome;
                            notif = move_and_wait(targetPosition, f_exp, MOTOR_NOTIF_CYCLE);
                        } else if (notif & MOTOR_NOTIF_LIM_DOWN) {
                            // TODO
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            genMotorError();
                        }
                        break;

                    case reCalibHome:
                        MOTOR_DEBUG_PRINT("in reCalibHome phase \r\n");
                        if (notif & MOTOR_NOTIF_HALT) {
                            motorHalt("reCalibUp");
                        } else if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO
                        } else if (notif & MOTOR_NOTIF_LIM_UP) {
                            // TODO
                        } else if (notif & MOTOR_NOTIF_LIM_DOWN) {
                            // TODO
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            recalibrateFlag = 0;
                            breathState = cycleEnd;
                            motor_disable();
                        }
                        break;

                    case cycleEnd:
                        vTaskDelayUntil(&previousWakeTime,ticksTctTime);
                        breathState = startNewCycle;
                        MOTOR_DEBUG_PRINT("to Start new cycle \r\n");
                        break;

                    case stopping:
                        // BOUNDED wait for end of move
                        // TODO ensure each low-level motor move generates an
                        // end of movement notif and that we always do a move
                        // before coming into this state.
                        if(motor_moving()){
                            notif = boundedWaitNotification(
                                    MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_MOVEMENT_FINISHED | MOTOR_NOTIF_OVER_PRESSURE,
                                    pdMS_TO_TICKS(3000)
                                    );
                            if (notif & MOTOR_NOTIF_HALT) {
                                // already stopping...
                                MOTOR_DEBUG_PRINT("[MOTOR] double Halt\r\n");
                            } else if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                                // TODO
                            } else if (notif & MOTOR_NOTIF_LIM_UP) {
                                // TODO
                            } else if (notif & MOTOR_NOTIF_LIM_DOWN) {
                                // TODO
                            } else {
                                // case MOTOR_NOTIF_MOVEMENT_FINISHED
                                motor_disable();
                                motorState = motorStopped;
                                MOTOR_DEBUG_PRINT("[MOTOR] to motor stopped\r\n");
                            }
                        } else {
                            motor_disable();
                            motorState = motorStopped;
                            MOTOR_DEBUG_PRINT("[MOTOR] to motor stopped\r\n");
                        }
                        break;
                }
                break;

            case motorError:
                // TODO
                vTaskDelayUntil(&previousWakeTime,pdMS_TO_TICKS(1000));
                break;
        }
    }
}

#else // MOTOR_ACTIVE

void MotorControlTask(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif // MOTOR_ACTIVE

