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
#include "core/motor_control.h"
#include "core/volume.h"
#include "core/alarm.h"
#include "core/parameters.h"
#include "hal/limit_switch.h"

#define CURR_DEBUG_PREFIX motor
#include "core/debug.h"


// TODO
// * delay motor disable to prevent overrunning
// * fix timeout values

#define MOTOR_VOL_CTRL 1

#if DEBUG_MOTOR
#define DEBUG_PRINT debug_print_prefix
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_MOTOR
#define MOTOR_ERROR_PRINT debug_print

#define MOTOR_NOTIF_LIM (MOTOR_NOTIF_LIM_UP | MOTOR_NOTIF_LIM_DOWN)
#define MOTOR_NOTIF_CYCLE (MOTOR_NOTIF_LIM | MOTOR_NOTIF_GLOBAL_STATE | MOTOR_NOTIF_OVER_PRESSURE | MOTOR_NOTIF_MOVEMENT_FINISHED)

#define MOTOR_UP_POSITION (2000*MOTOR_USTEPS)

#define MAX_POS_MARGIN (10*MOTOR_USTEPS)

#define LIM_UNPRESSED 0
#define LIM_PRESSED 1
#define LIM_DONTCARE 2
#define LIM_UNREACHABLE 255

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

volatile uint32_t cycleCount;


#if DEBUG_MOTOR
static const char *state_names[] = {
    "Init",
    "Calib",
    "Stopped",
    "Running",
    "Error"
};

static const char *bstate_names[] = {
    "insp",
    "plateau",
    "expiration",
    "cycleEnd",
    "stNewCycle",
    "stopping",
    "reCbUp",
    "reCbUpWaitStop",
    "reCbHome",
    "expStopping",
    "reCbDown",
    "preStopping",
};

static const char *cstate_names[] = {
    "cStart",
    "cDown",
    "cDownWaitStop",
    "cUp",
    "cUpWaitStop",
    "cPosEnd",
    "cVol",
    "cVolEnd",
    "cWaitStopAbort"
};

static const char *notif_names[] = {
    "NF_LIM_DOWN",
    "NF_LIM_UP",
    "",
    "NF_MVT_FINISHED",
    "NF_GLOBAL_STATE",
    "",
    "",
    "NF_OVER_PRESSURE",
};
#endif // DEBUG_MOTOR

// Threshold for volume calibraiton

// 0 replaces the whole task by an empty loop for testing purposes
#define MOTOR_ACTIVE 1

static void genMotorError(char *msg);
static bool bwaitTimeoutExpired();
static bool resumeBoundedWaitNotification();
static bool boundedWaitNotification(TickType_t ticksToWait, bool allow_timeout);
static void startCycleEnd();
static void startRecalib();
static void unboundedWaitNotification();
static void move_and_wait(uint32_t targetPosition, uint32_t max_speed);
static void calib_move_and_wait(uint32_t targetPosition, uint32_t max_speed);
static void run_move_and_wait(uint32_t targetPosition, uint32_t max_speed);
static void stop_and_wait();
static void startExpiration();
static uint8_t test_notif(uint32_t tested_notif);
static uint8_t need_recalibration();
static void startInsp();
static void finishInsp(bool need_stop);
static void startPlateau();
static void finishPlateau(bool need_stop);
static void overPressureRunning();

// TODO something real
uint32_t vol2steps(uint8_t tidal_vol) {
    DEBUG_PRINT("cycle ml %li", cycle_volume/1000);
    DEBUG_PRINT("tidal_vol %u", (int16_t) tidal_vol);
    DEBUG_PRINT("n_steps %lu", n_steps);
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
        MOTOR_ERROR_PRINT("To lrg nstep: %lu, max: %lu", n_steps, max_steps);
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

    DEBUG_PRINT("ie %u",ie);
    DEBUG_PRINT("bpm %u",bpm);
    DEBUG_PRINT("TCT %u",TCT);
    DEBUG_PRINT("Ti %u",Ti);
    DEBUG_PRINT("nsteps %u",n_steps);
    DEBUG_PRINT("pulse_plateau %u",plateau_pulses);
    DEBUG_PRINT("Ttot plateau %u",T_tot_plateau);
    DEBUG_PRINT("tmp_f_plateau %u",tmp_f_plateau);
    DEBUG_PRINT("f_plateau %u",f_plateau);
    DEBUG_PRINT("cycleCount %u",cycleCount);
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
    DEBUG_PRINT("gErr %s %i/%i (%i)", msg, motorState, breathState, calibState);
    motorErrorState = errorStopping;
    motorState = motorError;
    stop_and_wait();
}

// Return true if the bounded wait timeout has expired, false otherwise.
// This function never blocks.
static bool bwaitTimeoutExpired() {
    return xTaskCheckForTimeOut(&timeOutBoundedWait, &boundedWaitTime) == pdTRUE;
}

// return true if notification received
static bool resumeBoundedWaitNotification() {
    uint32_t notif_recv;
    if (notif) {
        DEBUG_PRINT("rBWN notif %x", notif);
        return true;
    } else {
        if (bwaitTimeoutExpired()) {
            boundedWaitTime = 0;
        }
        if (xTaskNotifyWait(0x0,ALL_NOTIF_BITS,&notif_recv,boundedWaitTime)) {
            notif |= notif_recv;
            return true;
        } else {
            if (!waitTimeoutAllowed) {
                genMotorError("TIMEOUT");
            }
            return false;
        }
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
                DEBUG_PRINT("TST %s", notif_names[i]);
            }
        }
        return 1;
    } else {
        return 0;
    }
}

static void startCycleEnd() {
    DEBUG_PRINT("startCycleEnd");
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
    DEBUG_PRINT("reCalibUp pos %u", motor_current_position());
    run_move_and_wait(0, f_home);
}

static void unboundedWaitNotification() {
    while (!boundedWaitNotification(portMAX_DELAY, true));
}

static void move_and_wait(uint32_t targetPosition, uint32_t max_freq) {
    // max speed: 1200 steps/s. Acceleration: 200 steps/s/10ms
    // -> max 3*120 = 360 ms deceleration + acceleration + deceleration (we take 400 to have a margin)
    // total time is thus bounded by 360 ms + 1000*abs(currentPosition-targetPosition) / (max_freq)
    if (motor_moving()) {
        genMotorError("INMOTION");
    } else {
        uint32_t max_duration = 400 + (1000*ABS(((int32_t) targetPosition) - ((int32_t) motor_current_position())))/max_freq;
        DEBUG_PRINT("m&w curr pre %lu", motor_current_position());
        //DEBUG_PRINT("target %lu max_freq %lu", targetPosition, max_freq);
        //DEBUG_PRINT("max_dur %lu", max_duration);
        set_motor_goto_position_accel_exec(targetPosition, max_freq, 2, 200*MOTOR_USTEPS);
        DEBUG_PRINT("m&w curr post %lu", motor_current_position());
        DEBUG_PRINT("target %lu max_freq %lu", targetPosition, max_freq);
        DEBUG_PRINT("max_dur %lu", max_duration);
        boundedWaitNotification(pdMS_TO_TICKS(max_duration), false);
    }
}

static void stop_and_wait() {
    motor_anticipated_stop();
    DEBUG_PRINT("anticipated stop");
    boundedWaitNotification(pdMS_TO_TICKS(2*1000), false);
}

static void startExpiration() {
    DEBUG_PRINT("startExpiration");
    HOOK_START_EXP;
    // TODO do not base PID on volume when there is a motor
    // issue ... (aka recalibrateFlag is set).
    if (get_volume(&cycle_volume) != 0) {
        DEBUG_PRINT("No valid volume");
        cycle_volume = 0;
    } else {
        DEBUG_PRINT("Valid volume");
        check_volume(cycle_volume);
    }
    breathState = expiration;
    targetPosition = homePosition;
    return run_move_and_wait(targetPosition, f_exp);
}

// TODO change this
static uint8_t need_recalibration() {
    bool ll_error = motor_error();
    if (ll_error) {
        DEBUG_PRINT("RCB LL ERR");
    }
    bool mod_recalib = ((cycleCount & 0x3) == 0);
    if (mod_recalib) {
        DEBUG_PRINT("RCB MOD");
    }
    if (recalibrateFlag) {
        DEBUG_PRINT("RCB FLAG");
    }
    return recalibrateFlag || mod_recalib || ll_error;
}

static void startInsp() {
    compute_config();
    motor_enable();
    HOOK_START_INSP;
    breathState = insp;
    targetPosition = homePosition + insp_pulses;
    DEBUG_PRINT("Ti pulses used %u",targetPosition);
    DEBUG_PRINT("=> target %u",targetPosition);
    DEBUG_PRINT("cur pos %u",motor_current_position());
    DEBUG_PRINT("home %u",homePosition);
    DEBUG_PRINT("to insp");
    run_move_and_wait(targetPosition, f_insp);
}

static void finishInsp(bool need_stop) {
    HOOK_END_INSP;
    if (need_stop) {
        // abort
        breathState = inspStopping;
        recalibrateFlag = true;
        DEBUG_PRINT("recalibrate INSP asked");
        // TODO wait a bit... to respect Ti ?
        stop_and_wait();
    } else {
        startPlateau();
        // Note: added for debug
        recalibrateFlag = true;
    }
}

static void startPlateau() {
    breathState = plateau;
    targetPosition = homePosition + insp_pulses + plateau_pulses;
    run_move_and_wait(targetPosition, f_plateau);
    DEBUG_PRINT("to plateau %lu %lu", plateau_pulses, f_plateau);
}

static void finishPlateau(bool need_stop) {
    HOOK_END_PLATEAU;
    if (need_stop) {
        // abort
        breathState = inspStopping;
        recalibrateFlag = true;
        DEBUG_PRINT("recalibrate PLATEAU asked");
        // TODO wait a bit... to respect Ti ?
        stop_and_wait();
    } else {
        startExpiration();
    }
}

static void overPressureRunning() {
    switch (breathState) {
        case insp:
            finishInsp(true);
            break;
        case plateau:
            finishPlateau(true);
            break;
        default:
            resumeBoundedWaitNotification();
    }
}

static void resetCalib(){
    motorState = motorInit;
    motor_disable();
}

static uint8_t flagNotifEndCalib;
static AlarmCause_t notifToSendEndCalib;

static void resetEndCalib() {
    flagNotifEndCalib = 0;
    notifToSendEndCalib = noError;
}

static void endAbortCalib(){
    DEBUG_PRINT("END ABORT");
    // Reset calibration
    resetCalib();
    // Notify main task if needed
    if (flagNotifEndCalib) {
        sendNewAlarm(notifToSendEndCalib);
    }
    // Wait for notification
    unboundedWaitNotification();
}

static void abortCalib(uint8_t flagEnd, AlarmCause_t error){
    DEBUG_PRINT("abort calib %s",cstate_names[calibState]);
    flagNotifEndCalib = flagEnd;
    notifToSendEndCalib = error;
    switch (calibState) {
        case calibStart:
            resetCalib();
            break;
        case calibDown:
        case calibUp:
        case calibPosEnd:
        case calibVol:
        case calibVolEnd:
            calibState = calibWaitStopAbort;
            stop_and_wait();
            break;
        case calibDownWaitStop:
        case calibUpWaitStop:
            calibState = calibWaitStopAbort;
            resumeBoundedWaitNotification();
        case calibWaitStopAbort:
            resumeBoundedWaitNotification();
            break;
    }
}

// Test if the value 'lim' (expected to be the result of a get_lim_X_v() call)  is 'value'
// return 0 if succesful test, 1 otherwise
static uint8_t testLimValue(uint8_t lim, uint8_t value) {
    if (value == LIM_DONTCARE) {
        return 0;
    } else {
        return !(lim == value);
    }
}

// Test the value of the both limit switch
// return 0 if succesful tests, 1 otherwise
static uint8_t testLims(uint8_t up, uint8_t down) {
    return testLimValue(get_lim_v(LIM_UP),up) ||
        testLimValue(get_lim_v(LIM_DOWN), down);
}

// Test the values of the switch depending on the
// current calibState value
// Return 0 if succesful test, 1 otherwise
// TODO: extend to return error code (using shift in each case for example)
static uint32_t checkLimsCalib() {
    if (!testLims(LIM_PRESSED, LIM_PRESSED)) {
        DEBUG_PRINT("both pressed");
        return LIM_UNREACHABLE;
    }
    switch (calibState) {
        case calibDown:
        case calibPosEnd:
            return testLims(LIM_DONTCARE, LIM_UNPRESSED);
        case calibUp:
            return testLims(LIM_UNPRESSED, LIM_DONTCARE);
        case calibVol:
        case calibVolEnd:
            return testLims(LIM_UNPRESSED, LIM_UNPRESSED);
        default:
            DEBUG_PRINT("URCH clc");
            return LIM_UNREACHABLE;
    }
}

// As checkLimsCalib, but when running cycles
static uint32_t checkLimsRun() {
    if (!testLims(LIM_PRESSED, LIM_PRESSED)) {
        return LIM_UNREACHABLE;
    }
    switch (breathState) {
        case insp:
        case plateau:
        case expiration:
            return testLims(LIM_UNPRESSED, LIM_UNPRESSED);
        case reCalibUp:
        case inspStopping:
            return testLims(LIM_UNPRESSED, LIM_DONTCARE);
        case reCalibDown:
        case reCalibHome:
        case expStopping:
        case stopping:
            return testLims(LIM_DONTCARE, LIM_UNPRESSED);

        default:
            return LIM_UNREACHABLE;
    }
}

// Move and wait with an addition limitswitch test
// TODO: handle different error code and return appropriate error code
static void calib_move_and_wait(uint32_t targetPosition, uint32_t max_freq) {
    uint32_t switches_test_value = checkLimsCalib();
    if (switches_test_value == LIM_UNREACHABLE) {
        genMotorError("calib LIM test unreachable");
        move_and_wait(targetPosition, max_freq);
    }
    if (switches_test_value == 0) {
        move_and_wait(targetPosition, max_freq);
    } else {
        abortCalib(1,calibIncorrectFlow);
    }
}

static void run_move_and_wait(uint32_t targetPosition, uint32_t max_freq) {
    DEBUG_PRINT("MOVE AND WAIT");
    uint32_t switches_test_value = checkLimsRun();
    if (switches_test_value == LIM_UNREACHABLE) {
        genMotorError("URCH LIM test");
    }
    if (switches_test_value == 0) {
        move_and_wait(targetPosition, max_freq);
    } else {
        switch (breathState) {
            case insp:
            case plateau:
            case expiration:
            case inspStopping:
            case expStopping:
            case stopping:
                // TODO send notification
                recalibrateFlag = true;
                move_and_wait(targetPosition, max_freq);
                break;
            case reCalibUp:
            case reCalibDown:
            case reCalibHome:
                genMotorError("CF recalib");
                break;

            default:
                genMotorError("URCH switch error");
                break;
        }
    }
    //move_and_wait(targetPosition,max_freq);
}

#if MOTOR_ACTIVE
void MotorControlTask(void *pvParameters)
{
    while (1)
    {
        DEBUG_PRINT("In %s/%s/%s", state_names[motorState], bstate_names[breathState], cstate_names[calibState]);
        DEBUG_PRINT("%i %i n 0x%lx", breathState, calibState, notif);
        switch (motorState) {
            case motorInit:
                DEBUG_PRINT("IN INIT %i", globalState);
                if (globalState == calibration) {
                    motorState = motorCalibrating;
                    calibState = calibStart;
                    resetEndCalib();
                    DEBUG_PRINT("Start calib");
                    notif = 0;
                } else {
                    // NON BOUNDED wait for calibrating notification
                    unboundedWaitNotification();
                    notif = 0;
                }
                break;

            case motorCalibrating:
                if (test_notif(MOTOR_NOTIF_OVER_PRESSURE)) {
                    abortCalib(1,calibPatientConnected);
                } else if (test_notif(MOTOR_NOTIF_GLOBAL_STATE)) {
                    abortCalib(0,noError);
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
                            DEBUG_PRINT("finished calibDownWaitStop");
                            calib_move_and_wait(targetPosition, f_home);
                            break;

                        case calibUp:
                            genMotorError("Not touched calibUp");
                            break;

                        case calibUpWaitStop:
                            // travelled distance = posOffset - motor_current_position();
                            // max position = MOTOR_UP_POSITION + travelled distance - MAX_POS_MARGIN
                            max_position = posOffset + MOTOR_UP_POSITION - motor_current_position() - MAX_POS_MARGIN;
                            DEBUG_PRINT("setting max_pos %lu", max_position);
                            DEBUG_PRINT("posOffset: %lu, current: %lu", posOffset, motor_current_position());
                            // Compute next position
                            posOffset = MOTOR_UP_POSITION + MOTOR_USTEPS*steps_calib_end;
                            set_motor_current_position_value(MOTOR_UP_POSITION);
                            calibState = calibPosEnd;
                            DEBUG_PRINT("finished calibUpWaitStop");
                            calib_move_and_wait(posOffset, f_home);
                            break;

                        case calibPosEnd:
                            homePosition = motor_current_position();
                            // Start volume calibration
                            posOffset = MOTOR_USTEPS*steps_calib_vol;
                            targetPosition = homePosition + posOffset;
                            if (targetPosition > max_position) {
                                MOTOR_ERROR_PRINT("MXPOS small Vcheck");
                                targetPosition = max_position;
                            }
                            calibState = calibVol;
                            // Reset volume prior to flow check
                            reset_volume();
                            DEBUG_PRINT("Start Vcheck");
                            calib_move_and_wait(targetPosition, f_fast);
                            break;

                        case calibVol:
                            if (MOCK_VOLUME_SENSOR ||
                                    (get_volume(&cycle_volume) == 0 &&
                                     cycle_volume > VOLUME_CHECK_THRESHOLD)) {
                                cycle_volume = 0;
                                // Sufficient volume insufflated.
                                calibState = calibVolEnd;
                                DEBUG_PRINT("Vcheck OK");
                                calib_move_and_wait(homePosition, f_fast);
                            } else {
                                DEBUG_PRINT("Vcheck FAIL");
                                abortCalib(1,calibIncorrectFlow);
                            }
                            break;

                        case calibVolEnd:
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            motor_disable();
                            motorState = motorStopped;
                            DEBUG_PRINT("END CALIB");
                            break;

                        case calibWaitStopAbort:
                            endAbortCalib();
                            break;
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_UP)) {
                    switch (calibState) {
                        case calibDownWaitStop:
                        case calibDown:
                            // We were unrolling the rope, we will soon start
                            // enrolling it in the correct direction.
                            // Continue in this state.
                            resumeBoundedWaitNotification();
                            break;

                        case calibUp:
                            // Stop motor
                            calibState = calibUpWaitStop;
                            DEBUG_PRINT("finished calibUp");
                            stop_and_wait();
                            break;

                        case calibPosEnd:
                        case calibVol:
                        case calibVolEnd:
                            DEBUG_PRINT("LIM UP in %s",cstate_names[calibState]);
                            abortCalib(1,calibIncorrectFlow);
                            break;

                        default:
                            genMotorError("unexpected notif");
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_DOWN)) {
                    switch (calibState) {
                        case calibDown:
                            // Finished going down, let's now stop the motor and go up.
                            // Stop motor
                            DEBUG_PRINT("finished calibDown");
                            calibState = calibDownWaitStop;
                            stop_and_wait();
                            break;

                        case calibUp:
                        case calibPosEnd:
                        case calibVol:
                        case calibVolEnd:
                            DEBUG_PRINT("LIM DOWN in %s",cstate_names[calibState]);
                            abortCalib(1,calibIncorrectFlow);
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
                            DEBUG_PRINT("to calib down/r\n");
                            calib_move_and_wait(targetPosition, f_home);
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
                    //DEBUG_PRINT("TCT %u",TCT);
                    //DEBUG_PRINT("Ti %u",Ti);
                    //DEBUG_PRINT("tot_pulses %u",tot_pulses);
                    //DEBUG_PRINT("plat pulses %u",plateau_pulses);
                    //DEBUG_PRINT("Ti pulses %u",insp_pulses);
                    //DEBUG_PRINT("TtotTi %u",T_tot_Ti);
                    //DEBUG_PRINT("TtotTe %u",T_tot_Te);
                    //DEBUG_PRINT("f_insp %u",f_insp);
                    //DEBUG_PRINT("f_exp %u",f_exp);
                    notif = 0;
                    motor_enable();
                    startRecalib();
                } else {
                    // Non bounded wait for notif
                    unboundedWaitNotification();
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
                            DEBUG_PRINT("continue running");
                            resumeBoundedWaitNotification();
                            break;
                        case stop:
                        case critical_failure:
                            switch (breathState) {
                                case preStopping:
                                case stopping:
                                    DEBUG_PRINT("stop again");
                                    resumeBoundedWaitNotification();
                                    break;
                                default:
                                    // stop motor
                                    breathState = preStopping;
                                    DEBUG_PRINT("halting");
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
                            finishInsp(false);
                            break;
                        case plateau:
                            finishPlateau(false);
                            break;
                        case inspStopping:
                            startExpiration();
                            break;
                        case expiration:
                            DEBUG_PRINT("finished EXP");
                            if (need_recalibration()) {
                                startRecalib();
                            } else {
                                DEBUG_PRINT("to WaitCycleEnd");
                                startCycleEnd();
                            }
                            break;
                        case expStopping:
                            // Move a bit down to escape from limit switch
                            breathState = reCalibDown;
                            run_move_and_wait(motor_current_position()+steps_calib_end*MOTOR_USTEPS, f_home);
                            break;
                        case cycleEnd:
                            MOTOR_ERROR_PRINT("MF while cycleEnd");
                            resumeBoundedWaitNotification();
                            break;
                        case stopping:
                            motor_disable();
                            motorState = motorStopped;
                            DEBUG_PRINT("to motor stopped");
                            break;
                        case reCalibUpWaitStop:
                            // Compute actual position when notified
                            set_motor_current_position_value(MOTOR_UP_POSITION);
                            // Compute next position
                            targetPosition = MOTOR_UP_POSITION + MOTOR_USTEPS*steps_calib_end;
                            breathState = reCalibHome;
                            DEBUG_PRINT("to reCalibHome");
                            run_move_and_wait(targetPosition, f_fast);
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
                            run_move_and_wait(homePosition, f_exp);
                            break;
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_UP)) {
                    switch (breathState) {
                        case insp:
                        case plateau:
                        case inspStopping:
                            MOTOR_ERROR_PRINT("UP while going down");
                            recalibrateFlag = true;
                            resumeBoundedWaitNotification();
                            break;
                        case expiration:
                            breathState = expStopping;
                            stop_and_wait();
                            break;
                        case preStopping:
                        case expStopping:
                            MOTOR_ERROR_PRINT("UP while AntStop");
                            resumeBoundedWaitNotification();
                            break;
                        case cycleEnd:
                            MOTOR_ERROR_PRINT("LIM while cycleEnd");
                            resumeBoundedWaitNotification();
                            break;
                        case stopping:
                            MOTOR_ERROR_PRINT("UP while stopping");
                            recalibrateFlag = true;
                            stop_and_wait();
                            // remain in the stopping state, we just shortened the course
                            break;
                        case reCalibUp:
                            // Stop motor
                            breathState = reCalibUpWaitStop;
                            DEBUG_PRINT("to reCalibUpWaitStop");
                            stop_and_wait();
                            break;
                        case reCalibHome:
                            MOTOR_ERROR_PRINT("UP while reCalibHome");
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
                            finishInsp(true);
                            break;
                        case plateau:
                            finishPlateau(true);
                            break;
                        case expiration:
                        case inspStopping:
                            MOTOR_ERROR_PRINT("LIM_DOWN while going up");
                            resumeBoundedWaitNotification();
                            break;
                        case expStopping:
                        case preStopping:
                            MOTOR_ERROR_PRINT("DOWN while AntStop");
                            resumeBoundedWaitNotification();
                            break;
                        case cycleEnd:
                            MOTOR_ERROR_PRINT("LIM while cycleEnd");
                            resumeBoundedWaitNotification();
                            break;
                        case stopping:
                            MOTOR_ERROR_PRINT("UP while stopping");
                            recalibrateFlag = true;
                            stop_and_wait();
                            // remain in the stopping state, we just shortened the course
                            break;
                        case reCalibUp:
                            DEBUG_PRINT("reCalibUp wait");
                            resumeBoundedWaitNotification();
                            break;
                        case reCalibHome:
                            MOTOR_ERROR_PRINT("DOWN while reCalibHome");
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
                                DEBUG_PRINT("to Start new cycle");
                            } else {
                                // Unreachable
                                genMotorError("cycleEnd URCH");
                            }
                            break;
                        case startNewCycle:
                            startInsp();
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
                            DEBUG_PRINT("Finished stopping");
                            motorErrorState = errorStopped;
                            motor_disable();
                            sendNewAlarm(cfMotorError);
                        } else {
                            resumeBoundedWaitNotification();
                        }
                        break;
                    case errorStopped:
                        unboundedWaitNotification();
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

