#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <util/delay.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "core/main_task.h"
#include "core/system.h"
#include "core/analog_read.h"
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
// check that notif is always properly disabled when needed
// robustify calibration & etc.

#define MOTOR_VOL_CTRL 0

#define DEBUG_MOTOR 1
#if DEBUG_MOTOR
#define MOTOR_DEBUG_PRINT debug_print
#else
#define MOTOR_DEBUG_PRINT fake_debug_print
#endif // DEBUG_MOTOR
#define MOTOR_ERROR_PRINT debug_print

#define MOTOR_NOTIF_LIM (MOTOR_NOTIF_LIM_UP | MOTOR_NOTIF_LIM_DOWN)
#define MOTOR_NOTIF_CYCLE (MOTOR_NOTIF_LIM | MOTOR_NOTIF_GLOBAL_STATE | MOTOR_NOTIF_OVER_PRESSURE | MOTOR_NOTIF_MOVEMENT_FINISHED)

#define MOTOR_UP_POSITION (2000*MOTOR_USTEPS)

#define MAX_POS_MARGIN (10*MOTOR_USTEPS)

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

const uint32_t f_fast = 500*MOTOR_USTEPS;

uint32_t max_position;

const uint32_t vol_ml = 700;

///////////////////////////////////

volatile MotorState_t motorState;
volatile BreathState_t breathState;
volatile CalibState_t calibState;
volatile MotorErrorState_t motorErrorState;

uint32_t currentPosition;
uint32_t targetPosition;
uint32_t homePosition;
uint32_t posOffset;

TickType_t cycleStartTime;
uint32_t ticksTctTime;

// TODO reset proper value!
const uint32_t f_home = 100*MOTOR_USTEPS; // steps/s (not µsteps/s)

const int32_t steps_calib_down = 2000;
const int32_t steps_calib_up = 1000;
const int32_t steps_calib_end = 50;
const int32_t steps_calib_vol = 500;
const int32_t steps_recalib = 1000;

const uint32_t thresh_calib_vol_mil = 600;

static bool recalibrateFlag;
static uint32_t notif = 0;

static TimeOut_t timeOutBoundedWait;
static TickType_t boundedWaitTime;
static bool waitTimeoutAllowed;
static int32_t cycle_volume;

static uint32_t cycleCount;


static const char *state_names[] = {
    "motorInit",
    "motorCalibrating",
    "motorStopped",
    "motorRunning",
    "motorError"
};

static const char *bstate_names[] = {
    "insp",
    "plateau",
    "expiration",
    "cycleEnd",
    "startNewCycle",
    "stopping",
    "reCalibUp",
    "reCalibUpWaitStop",
    "reCalibHome",
    "expStopping",
    "reCalibDown",
    "preStopping",
};

static const char *cstate_names[] = {
    "calibStart",
    "calibDown",
    "calibDownWaitStop",
    "calibUp",
    "calibUpWaitStop",
    "calibPosEnd",
    "calibVol",
    "calibVolEnd"
};

static const char *notif_names[] = {
    "MOTOR_NOTIF_LIM_DOWN",
    "MOTOR_NOTIF_LIM_UP",
    "",
    "MOTOR_NOTIF_MOVEMENT_FINISHED",
    "MOTOR_NOTIF_GLOBAL_STATE",
    "",
    "",
    "MOTOR_NOTIF_OVER_PRESSURE",
};

// Threshold for volume calibraiton

// 0 replaces the whole task by an empty loop for testing purposes
#define MOTOR_ACTIVE 1

static void genMotorError(char *msg);
static void motorStChCalib(char *state);
static void motorUnimplementedCase(char *state);
static bool bwaitTimeoutExpired();
static bool resumeBoundedWaitNotification();
static bool boundedWaitNotification(TickType_t ticksToWait, bool allow_timeout);
static void startCycleEnd();
static void startRecalib();
static void unboundedWaitNotification(uint32_t admissible_notifications);
static void move_and_wait(uint32_t targetPosition, uint32_t max_speed);
static void stop_and_wait();
static void doExpiration();
static uint8_t test_notif(uint32_t tested_notif);
static uint8_t need_recalibration();
static void abort_insp();
static void overPressureRunning();

// TODO something real
uint32_t vol2steps(uint8_t tidal_vol) {
    MOTOR_DEBUG_PRINT("[MTR] v2st cycle ml: %li       ", cycle_volume/1000);
    MOTOR_DEBUG_PRINT("tidal_vol: %u ", (int16_t) tidal_vol);
    MOTOR_DEBUG_PRINT("n_steps: %lu\r\n", n_steps);
    if (cycle_volume == 0 || !MOTOR_VOL_CTRL) {
        return 10*tidal_vol;
    } else {
        int32_t error = (((int32_t) tidal_vol)*10*1000L) - cycle_volume;
        // gain of 0.5 step per ml
        return n_steps + error/2000;
        return 10*tidal_vol;
    }
}

void compute_config() {
    // Breathing cycles parameter
    TCT = (60000L / bpm); // in ms
    // TODO check for each possibilities
    Ti = (TCT/(1+ie)); // in ms
    Te = TCT - Ti;

    ticksTctTime = pdMS_TO_TICKS(TCT);

    n_steps = vol2steps(tidal_vol);
    uint32_t max_steps = max_position - homePosition;
    if (n_steps > max_steps) {
        MOTOR_ERROR_PRINT("To lrg nstep: %lu, max: %lu\r\n", n_steps, max_steps);
        n_steps = max_steps;
    }
    tot_pulses = n_steps * MOTOR_USTEPS;

    plateau_pulses = MOTOR_USTEPS*5; //tot_pulses/20;
    insp_pulses = tot_pulses - plateau_pulses;
    exp_pulses = tot_pulses;

    T_tot_plateau = Ti / 4;
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
    MOTOR_DEBUG_PRINT("pulse_plateau %u \r\n",plateau_pulses);
    MOTOR_DEBUG_PRINT("Ttot plateau %u \r\n",T_tot_plateau);
    MOTOR_DEBUG_PRINT("tmp_f_plateau %u \r\n",tmp_f_plateau);
    MOTOR_DEBUG_PRINT("f_plateau %u \r\n",f_plateau);
    MOTOR_DEBUG_PRINT("cycleCount %u \r\n",cycleCount);
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
    recalibrateFlag = false;
    boundedWaitTime = 0;
    vTaskSetTimeOutState(&timeOutBoundedWait);
    cycleCount = 0;
}

static void genMotorError(char *msg) {
    MOTOR_DEBUG_PRINT("[MOTOR] gErr %s %i/%i (%i)\r\n", msg, motorState, breathState, calibState);
    motorErrorState = errorStopping;
    //motor_anticipated_stop();
    //motor_disable();
    motorState = motorError;
    stop_and_wait();
}

static void motorStChCalib(char *state) {
    motor_anticipated_stop();
    motor_disable();
    motorState = motorInit;
    MOTOR_DEBUG_PRINT("[MOTOR] halting in %s\r\n", state);
}

static void motorUnimplementedCase(char *state) {
    MOTOR_ERROR_PRINT("[MOTOR] Unimplemnted %s\r\n", state);
    genMotorError("unimplemented");
}

// Return true if the bounded wait timeout has expired, false otherwise.
// This function never blocks.
static bool bwaitTimeoutExpired() {
    return xTaskCheckForTimeOut(&timeOutBoundedWait, &boundedWaitTime) == pdTRUE;
}

// return true if notification received
static bool resumeBoundedWaitNotification() {
    uint32_t notif_recv;
    if (!bwaitTimeoutExpired() &&
            (xTaskNotifyWait(0x0,ALL_NOTIF_BITS,&notif_recv,boundedWaitTime) == pdTRUE)) {
        notif |= notif_recv;
        return true;
    } else if (waitTimeoutAllowed) {
        return false;
    } else {
        // no notification received, timeout
        genMotorError("TIMEOUT");
        return false;
    }
}

// return true if notification received
static bool boundedWaitNotification(TickType_t ticksToWait, bool allow_timeout) {
    boundedWaitTime = ticksToWait;
    waitTimeoutAllowed = allow_timeout;
    vTaskSetTimeOutState(&timeOutBoundedWait);
    return resumeBoundedWaitNotification();
}

// return 1 and clear the notification flag if the notification is present
static uint8_t test_notif(uint32_t tested_notif) {
    if (notif & tested_notif) {
        notif &= ~tested_notif;
        int8_t i;
        for (i=0; i < 8; i++) {
            if ((tested_notif >>i) & 0x1) {
                MOTOR_DEBUG_PRINT("NOTIF %s\r\n", notif_names[i]);
            }
        }
        return 1;
    } else {
        return 0;
    }
}

static void startCycleEnd() {
    MOTOR_DEBUG_PRINT("[MOTOR] startCycleEnd\r\n");
    cycleCount += 1;
    breathState = cycleEnd;
    TickType_t curr_time = xTaskGetTickCount();
    TickType_t cycle_elapsed_time = curr_time - cycleStartTime;
    TickType_t wait_time;
    if (cycle_elapsed_time < ticksTctTime) {
        motor_disable();
        cycleStartTime += ticksTctTime;
        wait_time = ticksTctTime-cycle_elapsed_time;
    } else {
        // cycle was too long (for whatever reason, such as re-calibration
        // do not compensate on next cycle !
        cycleStartTime = curr_time;
        wait_time = 0;
    }
    // wait for the end of the cycle
    boundedWaitNotification(wait_time, true);
}

static void startRecalib() {
    breathState = reCalibUp;
    MOTOR_DEBUG_PRINT("[MOTOR] reCalibUp pos %u\r\n", motor_current_position());
    move_and_wait(0, f_home);
}

static void unboundedWaitNotification(uint32_t admissible_notifications) {
    uint32_t notif_recv;
    xTaskNotifyWait(0x0,ALL_NOTIF_BITS,&notif_recv,portMAX_DELAY);
    // received notification
    if (notif_recv & ~admissible_notifications) {
        // invalid notification
        genMotorError("invalid notif");
        MOTOR_DEBUG_PRINT("[MOTOR] Unexp notif %x\r\n", notif_recv);
    }
    notif |= notif_recv;
}

static void move_and_wait(uint32_t targetPosition, uint32_t max_freq) {
    // max speed: 1200 steps/s. Acceleration: 200 steps/s/10ms
    // -> max 3*120 = 360 ms deceleration + acceleration + deceleration (we take 400 to have a margin)
    // total time is thus bounded by 360 ms + 1000*abs(currentPosition-targetPosition) / (max_freq)
    if (motor_moving()) {
        genMotorError("INMOTION");
    } else {
        uint32_t max_duration = 400 + (1000*ABS(((int32_t) targetPosition) - ((int32_t) motor_current_position())))/max_freq;
        set_motor_goto_position_accel_exec(targetPosition, max_freq, 2, 200*MOTOR_USTEPS);
        MOTOR_DEBUG_PRINT("[MOTOR] move_and_wait curr:");
        MOTOR_DEBUG_PRINT("%lu target: %lu max_freq:", motor_current_position(), targetPosition);
        MOTOR_DEBUG_PRINT("%lu max_dur: %lu\r\n", max_freq, max_duration);
        boundedWaitNotification(pdMS_TO_TICKS(max_duration), false);
    }
}

static void stop_and_wait() {
    motor_anticipated_stop();
    MOTOR_DEBUG_PRINT("[MOTOR] anticipated stop\r\n");
    boundedWaitNotification(pdMS_TO_TICKS(2*1000), false);
}

static void doExpiration() {
    MOTOR_DEBUG_PRINT("[MOTOR] doExpiration\r\n");
    // TODO do not base PID on volume when there is an
    // error... (aka recalibreFlag is set).
    if (get_volume(&cycle_volume) != 0) {
        MOTOR_DEBUG_PRINT("[MOTOR] No valid volume\r\n");
        cycle_volume = 0;
    } else {
        MOTOR_DEBUG_PRINT("[MOTOR] Valid volume\r\n");
    }
    breathState = expiration;
    targetPosition = homePosition;
    return move_and_wait(targetPosition, f_exp);
}

static uint8_t need_recalibration() {
    return recalibrateFlag || ((cycleCount & 0x3) == 0);
}

static void abort_insp() {
    breathState = inspStopping;
    recalibrateFlag = true;
    MOTOR_DEBUG_PRINT("recalibrate INSP asked \r\n");
    // TODO wait a bit... to respect Ti ?
    stop_and_wait();
}

static void overPressureRunning() {
    switch (breathState) {
        case insp:
        case plateau:
            abort_insp();
            break;
        default:
            resumeBoundedWaitNotification();
    }
}

#if MOTOR_ACTIVE
void MotorControlTask(void *pvParameters)
{
    while (1)
    {
        MOTOR_DEBUG_PRINT("[MOTOR] In ");
        MOTOR_DEBUG_PRINT("%s / %s", state_names[motorState], bstate_names[breathState]);
        MOTOR_DEBUG_PRINT(" (%s) %i %i n 0x%lx\r\n", cstate_names[calibState], breathState, calibState, notif);
        switch (motorState) {
            case motorInit:
                if (globalState == calibration) {
                    motorState = motorCalibrating;
                    calibState = calibStart;
                    MOTOR_DEBUG_PRINT("[MOTOR] Start calibration\r\n");
                    notif = 0;
                } else {
                    // NON BOUNDED wait for calibrating notification
                    unboundedWaitNotification(MOTOR_NOTIF_GLOBAL_STATE);
                }
                break;

            case motorCalibrating:
                if (test_notif(MOTOR_NOTIF_OVER_PRESSURE)) {
                    motorUnimplementedCase("calib & OP");
                } else if (test_notif(MOTOR_NOTIF_GLOBAL_STATE)) {
                    motorStChCalib("");
                } else if (test_notif(MOTOR_NOTIF_MOVEMENT_FINISHED)) {
                    switch (calibState) {
                        case calibStart:
                            genMotorError("URCH calibStart");
                            break;

                        case calibDown:
                            genMotorError("Not touched calibDown");
                            break;

                        case calibDownWaitStop:
                            // Compute next position
                            posOffset = MOTOR_USTEPS*steps_calib_up;
                            // Manually set the absolute position of the motor
                            // Use this trick to avoid negative absolute positions
                            set_motor_current_position_value(posOffset);
                            targetPosition = 0;
                            calibState = calibUp;
                            MOTOR_DEBUG_PRINT("[MOTOR] finished calibDownWaitStop\r\n");
                            move_and_wait(targetPosition, f_home);
                            break;

                        case calibUp:
                            genMotorError("Not touched calibUp");
                            break;

                        case calibUpWaitStop:
                            // travelled distance = posOffset - motor_current_position();
                            // max position = MOTOR_UP_POSITION + travelled distance - MAX_POS_MARGIN
                            max_position = posOffset + MOTOR_UP_POSITION - motor_current_position() - MAX_POS_MARGIN;
                            MOTOR_DEBUG_PRINT("[MOTOR] setting max_pos: %lu ,", max_position);
                            MOTOR_DEBUG_PRINT("posOffset: %lu, current: %lu\r\n", posOffset, motor_current_position());
                            // Compute next position
                            posOffset = MOTOR_UP_POSITION + MOTOR_USTEPS*steps_calib_end;
                            set_motor_current_position_value(MOTOR_UP_POSITION);
                            calibState = calibPosEnd;
                            MOTOR_DEBUG_PRINT("[MOTOR] finished calibUpWaitStop\r\n");
                            move_and_wait(posOffset, f_home);
                            break;

                        case calibPosEnd:
                            homePosition = motor_current_position();
                            // Start volume calibration
                            posOffset = MOTOR_USTEPS*steps_calib_vol;
                            targetPosition = homePosition + posOffset;
                            if (targetPosition > max_position) {
                                MOTOR_ERROR_PRINT("[MOTOR] max position too small for volume check.\r\n");
                                targetPosition = max_position;
                            }
                            calibState = calibVol;
                            // Reset volume prior to flow check
                            reset_volume();
                            MOTOR_DEBUG_PRINT("[MOTOR] Start flow check.\r\n");
                            move_and_wait(targetPosition, f_fast);
                            break;

                        case calibVol:
                            if (MOCK_VOLUME_SENSOR ||
                                    (get_volume(&cycle_volume) == 0 &&
                                     cycle_volume > VOLUME_CHECK_THRESHOLD)) {
                                cycle_volume = 0;
                                // Sufficient volume insufflated.
                                calibState = calibVolEnd;
                                MOTOR_DEBUG_PRINT("[MOTOR] Flow check: OK\r\n");
                                move_and_wait(homePosition, f_fast);
                            } else {
                                MOTOR_DEBUG_PRINT("[MOTOR] Flow check: FAIL\r\n");
                                xTaskNotify(mainTaskHandle, NOTIF_INCORRECT_FLOW, eSetBits);
                                motor_disable();
                                motorState = motorError;
                            }
                            break;

                        case calibVolEnd:
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            motor_disable();
                            motorState = motorStopped;
                            MOTOR_DEBUG_PRINT("[MOTOR] END CALIB\r\n");
                            break;
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_UP)) {
                    switch (calibState) {
                        case calibDown:
                            // We were unrolling the rope, we will soon start
                            // enrolling it in the correct direction.
                            // Continue in this state.
                            resumeBoundedWaitNotification();
                            break;

                        case calibUp:
                            // Stop motor
                            calibState = calibUpWaitStop;
                            MOTOR_DEBUG_PRINT("[MOTOR] finished calibUp\r\n");
                            stop_and_wait();
                            break;

                        default:
                            genMotorError("unexpected notif");
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_DOWN)) {
                    switch (calibState) {
                        case calibDown:
                            // Finished going down, let's now stop the motor and go up.
                            // Stop motor
                            MOTOR_DEBUG_PRINT("[MOTOR] finished calibDown\r\n");
                            calibState = calibDownWaitStop;
                            stop_and_wait();
                            break;

                        default:
                            genMotorError("unexpected notif");
                    }
                } else {
                    switch (calibState) {
                        case calibStart:
                            calibState = calibDown;
                            set_motor_current_position_value(0);
                            targetPosition = MOTOR_USTEPS*steps_calib_down;
                            motor_enable();
                            MOTOR_DEBUG_PRINT("to calib down\r\n");
                            move_and_wait(targetPosition, f_home);
                            break;

                        default:
                            genMotorError("URCH Cno notif");
                    }
                }
                break;

            case motorStopped:
                // if notif == START: update motor state
                if (globalState == run) {
                    motorState = motorRunning;
                    cycleStartTime = xTaskGetTickCount();
                    MOTOR_DEBUG_PRINT("TCT %u \r\n",TCT);
                    MOTOR_DEBUG_PRINT("Ti %u \r\n",Ti);
                    MOTOR_DEBUG_PRINT("tot_pulses %u \r\n",tot_pulses);
                    MOTOR_DEBUG_PRINT("plat pulses %u \r\n",plateau_pulses);
                    MOTOR_DEBUG_PRINT("Ti pulses %u \r\n",insp_pulses);
                    MOTOR_DEBUG_PRINT("TtotTi %u \r\n",T_tot_Ti);
                    MOTOR_DEBUG_PRINT("TtotTe %u \r\n",T_tot_Te);
                    MOTOR_DEBUG_PRINT("f_insp %u \r\n",f_insp);
                    MOTOR_DEBUG_PRINT("f_exp %u \r\n",f_exp);
                    notif = 0;
                    motor_enable();
                    startRecalib();
                } else {
                    // Non bounded wait for notif
                    unboundedWaitNotification(MOTOR_NOTIF_GLOBAL_STATE);
                }
                break;

            case motorRunning:
                if (test_notif(MOTOR_NOTIF_OVER_PRESSURE)) {
                    overPressureRunning();
                } else if (test_notif(MOTOR_NOTIF_GLOBAL_STATE)) {
                    switch (globalState) {
                        case run:
                            // continue running
                            // in case we are stopping, we will re-start as
                            // soon as we reach the stopped state
                            MOTOR_DEBUG_PRINT("[MOTOR] continue running\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case stop:
                        case critical_failure:
                            switch (breathState) {
                                case preStopping:
                                case stopping:
                                    MOTOR_DEBUG_PRINT("[MOTOR] stop again\r\n");
                                    resumeBoundedWaitNotification();
                                    break;
                                default:
                                    // stop motor
                                    breathState = preStopping;
                                    MOTOR_DEBUG_PRINT("[MOTOR] halting\r\n");
                                    stop_and_wait();
                                    break;
                            }
                            break;
                        case welcome:
                        case welcome_wait_cal:
                        case calibration:
                            genMotorError("URCH motorGlStCh");
                            break;
                    }
                } else if (test_notif(MOTOR_NOTIF_MOVEMENT_FINISHED)) {
                    switch (breathState) {
                        case insp:
                            breathState = plateau;
                            targetPosition = homePosition + insp_pulses + plateau_pulses;
                            move_and_wait(targetPosition, f_plateau);
                            MOTOR_DEBUG_PRINT("[MOTOR] to plateau %lu %lu\r\n", plateau_pulses, f_plateau);
                            break;
                        case plateau:
                            measure_p_plateau();
                        case inspStopping:
                            doExpiration();
                            break;
                        case expiration:
                            MOTOR_DEBUG_PRINT("[MOTOR] finished EXP\r\n");
                            if (need_recalibration()) {
                                startRecalib();
                            } else {
                                MOTOR_DEBUG_PRINT("[MOTOR] to wait cycle end\r\n");
                                startCycleEnd();
                            }
                            break;
                        case expStopping:
                            // Move a bit down to escape from limit switch
                            breathState = reCalibDown;
                            move_and_wait(motor_current_position()+steps_calib_end*MOTOR_USTEPS, f_home);
                            break;
                        case cycleEnd:
                            MOTOR_ERROR_PRINT("[MOTOR] MF while cycleEnd\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case stopping:
                            motor_disable();
                            motorState = motorStopped;
                            MOTOR_DEBUG_PRINT("[MOTOR] to motor stopped\r\n");
                            break;
                        case reCalibUpWaitStop:
                            // Compute actual position when notified
                            set_motor_current_position_value(MOTOR_UP_POSITION);
                            // Compute next position
                            targetPosition = MOTOR_UP_POSITION + MOTOR_USTEPS*steps_calib_end;
                            breathState = reCalibHome;
                            MOTOR_DEBUG_PRINT("[MOTOR] to reCalibHome\r\n");
                            move_and_wait(targetPosition, f_fast);
                            break;
                        case reCalibHome:
                            recalibrateFlag = false;
                            startCycleEnd();
                            break;
                        case reCalibDown:
                            startRecalib();
                            break;
                        case startNewCycle:
                        case reCalibUp:
                            genMotorError("URCH mfinished");
                            break;
                        case preStopping:
                            breathState = stopping;
                            move_and_wait(homePosition, f_exp);
                            break;
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_UP)) {
                    switch (breathState) {
                        case insp:
                        case plateau:
                        case inspStopping:
                            MOTOR_ERROR_PRINT("[MOTOR] UP while going down\r\n");
                            recalibrateFlag = true;
                            resumeBoundedWaitNotification();
                            break;
                        case expiration:
                            breathState = expStopping;
                            stop_and_wait();
                            break;
                        case preStopping:
                        case expStopping:
                            MOTOR_ERROR_PRINT("[MOTOR] UP while anticipated stopping\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case cycleEnd:
                            MOTOR_ERROR_PRINT("[MOTOR] LIM while cycleEnd\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case stopping:
                            MOTOR_ERROR_PRINT("[MOTOR] UP while stopping\r\n");
                            recalibrateFlag = true;
                            stop_and_wait();
                            // remain in the stopping state, we just shortened the course
                            break;
                        case reCalibUp:
                            // Stop motor
                            breathState = reCalibUpWaitStop;
                            MOTOR_DEBUG_PRINT("[MOTOR] to reCalibUpWaitStop\r\n");
                            stop_and_wait();
                            break;
                        case reCalibHome:
                            MOTOR_ERROR_PRINT("[MOTOR] UP while reCalibHome\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case startNewCycle:
                        case reCalibUpWaitStop:
                        case reCalibDown:
                            genMotorError("URCH LIM_UP");
                            break;
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_DOWN)) {
                    switch (breathState) {
                        case insp:
                        case plateau:
                            abort_insp();
                            break;
                        case expiration:
                        case inspStopping:
                            MOTOR_ERROR_PRINT("[MOTOR] LIM_DOWN while going up\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case expStopping:
                        case preStopping:
                            MOTOR_ERROR_PRINT("[MOTOR] DOWN while anticipated stopping\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case cycleEnd:
                            MOTOR_ERROR_PRINT("[MOTOR] LIM while cycleEnd\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case stopping:
                            MOTOR_ERROR_PRINT("[MOTOR] UP while stopping\r\n");
                            recalibrateFlag = true;
                            stop_and_wait();
                            // remain in the stopping state, we just shortened the course
                            break;
                        case reCalibUp:
                            MOTOR_DEBUG_PRINT("[MOTOR] reCalibUp wait\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case reCalibHome:
                            MOTOR_ERROR_PRINT("[MOTOR] DOWN while reCalibHome\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case startNewCycle:
                        case reCalibUpWaitStop:
                        case reCalibDown:
                            genMotorError("URCH LIM_DOWN");
                            break;
                    }
                } else {
                    switch (breathState) {
                        case cycleEnd:
                            if (bwaitTimeoutExpired()) {
                                measure_peep();
                                breathState = startNewCycle;
                                MOTOR_DEBUG_PRINT("to Start new cycle \r\n");
                            } else {
                                // Unreachable
                                genMotorError("cycleEnd URCH");
                            }
                            break;
                        case startNewCycle:
                            compute_config();
                            motor_enable();
                            reset_volume();
                            breathState = insp;
                            targetPosition = homePosition + insp_pulses;
                            MOTOR_DEBUG_PRINT("Ti pulses used %u \r\n",targetPosition);
                            MOTOR_DEBUG_PRINT("=> target %u \r\n",targetPosition);
                            MOTOR_DEBUG_PRINT("cur pos %u \r\n",motor_current_position());
                            MOTOR_DEBUG_PRINT("home %u \r\n",homePosition);
                            MOTOR_DEBUG_PRINT("to insp \r\n");
                            move_and_wait(targetPosition, f_insp);
                            break;
                        default:
                            genMotorError("URCH Bno notif");
                    }
                }
                break;

            case motorError:
                //vTaskDelay(pdMS_TO_TICKS(10000));
                switch (motorErrorState) {
                    case errorStopping:
                        if (test_notif(MOTOR_NOTIF_MOVEMENT_FINISHED)) {
                            MOTOR_DEBUG_PRINT("[MOTOR] Finished stopping\r\n");
                            motorErrorState = errorStopped;
                            motor_disable();
                            xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
                        } else {
                            resumeBoundedWaitNotification();
                        }
                        break;
                    case errorStopped:
                        unboundedWaitNotification(MOTOR_NOTIF_CYCLE);
                        break;
                }
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

