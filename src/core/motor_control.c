#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "core/main_task.h"
#include "core/system.h"
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

#define MOTOR_NOTIF_LIM (MOTOR_NOTIF_LIM_UP | MOTOR_NOTIF_LIM_DOWN)

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

#if DEBUG_MOTOR     
    debug_print("ie %u \r\n",ie);
    debug_print("bpm %u \r\n",bpm);
    debug_print("TCT %u \r\n",TCT);
    debug_print("Ti %u \r\n",Ti);
    debug_print("nsteps %u \r\n",n_steps);
    debug_print("pulse_lateau %u \r\n",plateau_pulses);
    debug_print("Ttot plate %u \r\n",T_tot_plateau);
#endif // DEBUG_MOTOR
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
#if DEBUG_MOTOR
    debug_print("[MOTOR] halting in %s\r\n", state);
#endif // DEBUG_MOTOR
}

static void motorHalt(char *state) {
    motor_anticipated_stop();
    breathState = stopping;
    set_motor_goto_position_accel_exec(homePosition, f_exp, 2, 200);
#if DEBUG_MOTOR
    debug_print("[MOTOR] halting in %s\r\n", state);
#endif // DEBUG_MOTOR
}

static uint32_t boundedWaitNotification(uint32_t admissible_notifications, TickType_t ticksToWait) {
    uint32_t notif_recv;
    if (xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,ticksToWait) == pdTRUE) {
        // received notification
        if (notif_recv & ~admissible_notifications) {
            // invalid notification
            genMotorError();
#if DEBUG_MOTOR
            debug_print("[MOTOR] Unexp notif %x\r\n", notif_recv); 
#endif // DEBUG_MOTOR
        } else {
            return notif_recv;
        }
    } else {
        // no notification received, timeout
        genMotorError();
#if DEBUG_MOTOR
        debug_print("[MOTOR] TIMEOUT\r\n"); 
#endif // DEBUG_MOTOR
    }
    return notif_recv;
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
                    compute_config();
                    motorState = motorCalibrating;
                    calibState = calibDown;
                    currentPosition = 0;
                    targetPosition = MOTOR_USTEPS*steps_calib_down;
                    motor_enable();
                    set_motor_goto_position_accel_exec(targetPosition, f_home, 2, 200);
#if DEBUG_MOTOR
                    debug_print("to calib down\r\n");
#endif // DEBUG_MOTOR
                }
                break;

            case motorCalibrating:
                switch (calibState) {
                    case calibDown:
#if DEBUG_MOTOR
                        debug_print("[MOTOR] calibDown\r\n");
#endif // DEBUG_MOTOR
                        // BOUNDED wait for limit switch down 
                        notif = boundedWaitNotification(
                                MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE,
                                pdMS_TO_TICKS(10000)
                                );
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
                            set_motor_goto_position_accel_exec(targetPosition, MOTOR_USTEPS*f_home, 2, 200);
#if DEBUG_MOTOR
                            debug_print("[MOTOR] finished calibDown\r\n");
#endif // DEBUG_MOTOR
                        } else {
                            // MOTOR_NOTIF_LIM_UP
                            // We were unrolling the rope, we will soon start
                            // enrolling it in the correct direction.
                            // Continue in this state.
                        }
                        break;

                    case calibUp:
#if DEBUG_MOTOR
                        debug_print("[MOTOR] calibUp\r\n");
#endif // DEBUG_MOTOR
                        // BOUNDED wait for limit switch up 
                        notif = boundedWaitNotification(
                                MOTOR_NOTIF_LIM_UP | MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE,
                                pdMS_TO_TICKS(10000)
                                );
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
                            set_motor_goto_position_accel_exec(posOffset, MOTOR_USTEPS*f_home, 2, 200);
#if DEBUG_MOTOR
                            debug_print("[MOTOR] finished calibUp\r\n");
#endif // DEBUG_MOTOR
                        }
                        break;

                    case calibPosEnd:
#if DEBUG_MOTOR
                        debug_print("[MOTOR] calibPosEnd\r\n");
#endif // DEBUG_MOTOR
                        // BOUNDED wait for limit switch up 
                        notif = boundedWaitNotification(
                                MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE | MOTOR_NOTIF_MOVEMENT_FINISHED,
                                pdMS_TO_TICKS(10000)
                                );
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
                            set_motor_goto_position_accel_exec(targetPosition, MOTOR_USTEPS*steps_calib_vol, 2, 200);
#if DEBUG_MOTOR
                            debug_print("[MOTOR] Start flow check.\r\n");
#endif // DEBUG_MOTOR
                        }
                        break;

                    case calibVol:
#if DEBUG_MOTOR
                        debug_print("[MOTOR] calibVol\r\n");
#endif // DEBUG_MOTOR
                        // BOUNDED wait for limit switch up 
                        notif = boundedWaitNotification(
                                MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE | MOTOR_NOTIF_MOVEMENT_FINISHED,
                                pdMS_TO_TICKS(10000)
                                );
                        if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO handle this case
                        } else if (notif & MOTOR_NOTIF_HALT) {
                            motorHaltCalib("calibVol");
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            if (volume > VOLUME_CHECK_THRESHOLD) {
                                // Sufficient volume insufflated.
                                calibState = calibVolEnd;
                                set_motor_goto_position_accel_exec(homePosition, MOTOR_USTEPS*steps_calib_vol, 2, 200);
#if DEBUG_MOTOR
                                debug_print("[MOTOR] Flow check: OK\r\n");
#endif // DEBUG_MOTOR
                            } else {
#if DEBUG_MOTOR
                                debug_print("[MOTOR] Flow check: FAIL\r\n");
#endif // DEBUG_MOTOR
                                xTaskNotify(mainTaskHandle, NOTIF_INCORRECT_FLOW, eSetBits);
                                // TODO: correct ?
                                motorState = motorError;
                            }
                        }
                        break;

                    case calibVolEnd:
#if DEBUG_MOTOR
                        debug_print("[MOTOR] calibVolEnd\r\n");
#endif // DEBUG_MOTOR
                        // BOUNDED wait for limit switch up 
                        notif = boundedWaitNotification(
                                MOTOR_NOTIF_HALT | MOTOR_NOTIF_OVER_PRESSURE | MOTOR_NOTIF_MOVEMENT_FINISHED,
                                pdMS_TO_TICKS(10000)
                                );
                        if (notif & MOTOR_NOTIF_OVER_PRESSURE) {
                            // TODO handle this case
                        } else if (notif & MOTOR_NOTIF_HALT) {
                            motorHaltCalib("calibVolEnd");
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            motor_disable();
                            motorState = motorStopped;
#if DEBUG_MOTOR
                            debug_print("[MOTOR] END CALIB\r\n");
#endif // DEBUG_MOTOR
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
#if DEBUG_MOTOR
                    debug_print("TCT %u \r\n",TCT);
                    debug_print("Ti %u \r\n",Ti);
                    debug_print("tot_pulses %u \r\n",tot_pulses);
                    debug_print("plat pulses %u \r\n",plateau_pulses);
                    debug_print("Ti pulses %u \r\n",insp_pulses);
                    debug_print("TtotTi %u \r\n",T_tot_Ti);
                    debug_print("TtotTe %u \r\n",T_tot_Te);
                    debug_print("f_insp %u \r\n",f_insp);
                    debug_print("f_exp %u \r\n",f_exp);
#endif // DEBUG_MOTOR
                }
                break;

            case motorRunning:
                switch (breathState) {
                    case startNewCycle:
                        compute_config();
                        motor_enable();
                        breathState = insp;
                        targetPosition = homePosition + insp_pulses;
                        set_motor_goto_position_accel_exec(targetPosition, f_insp, 2, 200); 
#if DEBUG_MOTOR
                        debug_print("Ti pulses used %u \r\n",targetPosition);
                        debug_print("=> target %u \r\n",targetPosition);
                        debug_print("cur pos %u \r\n",motor_current_position());
                        debug_print("home %u \r\n",homePosition);
                        debug_print("to insp \r\n");
#endif // DEBUG_MOTOR
                        break;

                    case insp:
                        // BOUNDED Wait notification
                        notif = boundedWaitNotification(
                                MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_MOVEMENT_FINISHED | MOTOR_NOTIF_OVER_PRESSURE,
                                pdMS_TO_TICKS(3000)
                                );
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
                            set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);
                        } else if (notif & MOTOR_NOTIF_LIM_DOWN) {
                            motor_anticipated_stop();
                            recalibrateFlag = 1;
                            breathState = exp;
                            targetPosition = homePosition;
                            set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);
#if DEBUG_MOTOR
                            debug_print("recalibrate INSP asked \r\n");
#endif
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            breathState = plateau;
                            targetPosition = homePosition + insp_pulses + plateau_pulses; 
                            set_motor_goto_position(targetPosition, f_plateau);
#if DEBUG_MOTOR
                            debug_print("[MOTOR] to plateau\r\n");
#endif // DEBUG_MOTOR
                        }
                        break;

                    case plateau:
                        notif = boundedWaitNotification(
                                MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_MOVEMENT_FINISHED | MOTOR_NOTIF_OVER_PRESSURE,
                                pdMS_TO_TICKS(2000)
                                );
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
                            set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);
                        } else if (notif & MOTOR_NOTIF_LIM_DOWN) {
                            motor_anticipated_stop();
                            recalibrateFlag = 1;
                            breathState = exp;
                            targetPosition = homePosition;
                            set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);
#if DEBUG_MOTOR
                            debug_print("recalibrate plateau asked \r\n");
#endif
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            // go back to HOME
                            breathState = exp;
                            set_motor_goto_position_accel_exec(homePosition, f_exp, 2, 200);
#if DEBUG_MOTOR
                            debug_print("[MOTOR] to exp\r\n");
#endif // DEBUG_MOTOR
                        }
                        break;

                    case exp:
#if DEBUG_MOTOR
                        debug_print("in EXP phase \r\n");
#endif
                        notif = boundedWaitNotification(
                                MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_MOVEMENT_FINISHED | MOTOR_NOTIF_OVER_PRESSURE,
                                pdMS_TO_TICKS(3000)
                                );
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
                            set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);
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
                                set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);

                            } else {
                                breathState = cycleEnd;
                                motor_disable();
#if DEBUG_MOTOR
                                debug_print("[MOTOR] to wait cycle end\r\n");
#endif
                            }
                        }
			break;

                    case reCalibUp:
#if DEBUG_MOTOR
                        debug_print("in reCalibUp phase \r\n");
#endif
                        notif = boundedWaitNotification(
                                MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_MOVEMENT_FINISHED | MOTOR_NOTIF_OVER_PRESSURE,
                                pdMS_TO_TICKS(3000)
                                );
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
                            set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);
                        } else if (notif & MOTOR_NOTIF_LIM_DOWN) {
                            // TODO
                        } else {
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            genMotorError();
                        }
                        break;

                    case reCalibHome:
#if DEBUG_MOTOR
                        debug_print("in reCalibHome phase \r\n");
#endif
                        notif = boundedWaitNotification(
                                MOTOR_NOTIF_LIM | MOTOR_NOTIF_HALT | MOTOR_NOTIF_MOVEMENT_FINISHED | MOTOR_NOTIF_OVER_PRESSURE,
                                pdMS_TO_TICKS(3000)
                                );
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
#if DEBUG_MOTOR
                        debug_print("to Start new cycle \r\n");
#endif // DEBUG_MOTOR
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
#if DEBUG_MOTOR
                                debug_print("[MOTOR] double Halt\r\n");
#endif // DEBUG_MOTOR
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
#if DEBUG_MOTOR
                                debug_print("[MOTOR] to motor stopped\r\n");
#endif // DEBUG_MOTOR
                            }
                        } else {
                            motor_disable();
                            motorState = motorStopped;
#if DEBUG_MOTOR
                            debug_print("[MOTOR] to motor stopped\r\n");
#endif // DEBUG_MOTOR
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

