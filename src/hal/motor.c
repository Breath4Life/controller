////////////////////////////////////////////////////////////////////////////////
// Name: motorControl_pwmHard.ino
//
// Descr: First sketch about motor control based on hard PWM and hard Counter
//        Targe is Arduino Mega
//        Acceleration not yet implemented (but the way is paved)
//        Many improvements are possible
//        Not yet fully tested
//
//        Add a 100 ohms damping resistor between PWM output and Step input of Motor drive
//
//
// Author:  Guerric Meurice de Dormale <gm@bitandbyte.io>
//          Gaetan Cassiers <gaetan.cassiers@uclouvain.be>
//
//
////////////////////////////////////////////////////////////////////////////////

#define MOTOR_DBG 0
#define DEBUG_MOTOR 0

#include <stdint.h>
#include <limits.h>

#include <avr/interrupt.h>

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

#define MOTORCTRL_DIR_FORWARD 0
#define MOTORCTRL_DIR_BACKWARD 1
#define MOTORCTRL_DRIVE_ENBL_TRUE 0
#define MOTORCTRL_DRIVE_ENBL_FALSE 1

#define MOTORCTRL_PWM_RUN_VALUE _BV(CS32)
#define MOTORCTRL_PWM_OVF_IRQ_CFG _BV(TOIE3)
#define MOTORCTRL_PWM_OVF_IRQ TIMER3_OVF_vect

#define MOTORCTRL_STEP_CNT_IRQ TIMER5_COMPA_vect

#define COUNTER_STEP_MAX UINT_MAX

#define MOTORCTRL_MAX_SPEED_SMALL_MVMT 500

#ifndef RT_B4L
#define ARDUINO_RT
#endif // RT_B4L

#ifndef ARDUINO_RT
#include "hal/pins.h"
#include "hal/io.h"
#include "core/system.h"
#include "core/debug.h"
#include "core/motor_control.h"
#include "task.h"
#include "FreeRTOS.h"
#endif // ARDUINO_RT

#ifdef ARDUINO_RT
const uint8_t MOTORCTRL_DRIVE_DIR_PIN = 22;
const uint8_t MOTORCTRL_DRIVE_STEP_PIN = 2; // pwm_step (OC3B)
const uint8_t MOTORCTRL_DRIVE_STEP_PIN_BIS = 3; // pwm_step (OC3C)
const uint8_t MOTORCTRL_DRIVE_ENBL_PIN = 24;

const uint8_t MOTORCTRL_STEP_CNT_PIN = 47; // CNT5 input
const uint8_t MOTORCTRL_STEP_CNT_OUT_DBG_PIN = 25;
const uint8_t MOTORCTRL_STEP_CNT_OUT_DBG_PIN2 = 27;
#endif // ARDUINO_RT

const unsigned int MOTORCTRL_PWM_FREQ_DIV_FACTOR = 31250;

// We use signed number for position to account for potential extra steps while being at homing position
static volatile long motor_position_abs = 0;
static volatile long motor_position_rel = 0;
static volatile long motor_target_position_abs = 0;
static volatile long motor_target_position_rel = 0;
static volatile uint16_t motor_target_pwm_freq = 0;
static volatile uint16_t motor_increment_pwm_freq = 0;
static volatile uint16_t motor_current_pwm_freq = 0;
static volatile uint8_t motor_direction = 0;
volatile uint8_t motor_inmotion = 0;
static volatile uint8_t motor_accel_enbl = 0;
static volatile uint16_t motor_step_cnt_incr_curr = 0;
static volatile uint16_t motor_step_cnt_accel_incr_base = 0;
static volatile uint16_t motor_step_cnt_accel_incr_sum = 0;
static volatile uint16_t motor_step_cnt_accel_incr_last = 0;
static volatile uint16_t motor_pwm_freq_accel_last = 0;

static void setup_pwm_step();
static void disable_cnt5_irq();
static void enable_cnt5_irq();
static void setup_ext_cnt5();
static void irq_step_count_clbk();
static void set_period_pwm_step(const unsigned int period);
static void set_freq_pwm_step(const unsigned int freq);
static void stop_pwm_step();
static void set_threshold_cnt5(const unsigned int thresh);
static void reset_cnt5();
static unsigned int get_cnt5();

static void setup_pwm_step()
{
  cli();//stop interrupts
  TCCR3A = 0;
  TCCR3B = 0;

  // WGM mode 9 (PWM, Phase and Frequency Correct - OCRn)
  // pre-scaler: div-256
  // OCR3A fixes the period
  // OCR3B fixes duty cycle
  // 3 is the minimum period for the timer
  // setting OCR3B = OCR3A guarantees 0 output
  // setting a low period minimizes turn-on latency.
  // always keep OCR3C and OCR3B in sync
  OCR3A = 3;
  OCR3B = COUNTER_STEP_MAX;
  OCR3C = COUNTER_STEP_MAX;
  // WGM mode 9 (PWM, Phase and Frequency Correct - OCRn)
  // pre-scaler: div-256
  // OCR3A fixes the period
  // OCR3B fixes duty cycle
  TCCR3A = _BV(COM3A0) | _BV(COM3B1) | _BV(COM3B0) | _BV(COM3C1) | _BV(COM3C0) | _BV(WGM30);
  TCCR3B |= _BV(WGM33);

  // Start
  TCCR3B |= MOTORCTRL_PWM_RUN_VALUE;

  // Disable interrupts
  TIMSK3 = 0;

#ifdef ARDUINO_RT
  pinMode(MOTORCTRL_DRIVE_STEP_PIN, OUTPUT);
  pinMode(MOTORCTRL_DRIVE_STEP_PIN_BIS, OUTPUT);
  //pinMode(5, OUTPUT); // to debug PWM, always duty cycle 1/2
#else
  dio_init(DIO_PIN_MOTOR_STEP, DIO_OUTPUT);
  dio_init(DIO_PIN_MOTOR_STEP_BIS, DIO_OUTPUT);
#endif // ARDUINO_RT

  sei();//allow interrupts
}

static void disable_cnt5_irq()
{
  TIMSK5 &= ~ _BV(OCIE5A);
}

static void enable_cnt5_irq()
{
  TIMSK5 |= _BV(OCIE5A);
}

static void setup_ext_cnt5()
{
  cli();//stop interrupts

  // Counter for PWM output
#ifdef ARDUINO_RT
  pinMode(MOTORCTRL_STEP_CNT_PIN, INPUT);
#else
  dio_init(DIO_PIN_STEP_COUNTER_TN, DIO_INPUT);
#endif // ARDUINO_RT
  // WGM mode 4 (CTC - OCRn)
  // External rising-edge clock
  // ? add ICNC5 ?
  TCCR5A = 0;
  TCCR5B = _BV(CS52) | _BV(CS51) | _BV(CS50) | _BV(WGM52);
  OCR5A = MOTORCTRL_PWM_FREQ_DIV_FACTOR; // By default
  TIMSK5 = _BV(OCIE5A);

  sei();//allow interrupts
}

#ifdef ARDUINO_RT
void setup()
#else
void setup_motor()
#endif // ARDUINO_RT
{

  setup_pwm_step();
  setup_ext_cnt5();

#ifdef ARDUINO_RT
  pinMode(MOTORCTRL_STEP_CNT_OUT_DBG_PIN, OUTPUT);
  digitalWrite(MOTORCTRL_STEP_CNT_OUT_DBG_PIN, 0);
  pinMode(MOTORCTRL_STEP_CNT_OUT_DBG_PIN2, OUTPUT);
  digitalWrite(MOTORCTRL_STEP_CNT_OUT_DBG_PIN2, 0);
#endif // ARDUINO_RT

#ifdef ARDUINO_RT
  // TMC Dir pin
  pinMode(MOTORCTRL_DRIVE_DIR_PIN, OUTPUT);
  // TMC enable pin
  pinMode(MOTORCTRL_DRIVE_ENBL_PIN, OUTPUT);
  digitalWrite(MOTORCTRL_DRIVE_ENBL_PIN, MOTORCTRL_DRIVE_ENBL_FALSE);
#else
  // TMC Dir pin
  dio_init(DIO_PIN_MOTOR_DIRECTION, DIO_OUTPUT);
  // TMC enable pin
  dio_init(DIO_PIN_MOTOR_ENABLE, DIO_OUTPUT);
  dio_write(DIO_PIN_MOTOR_ENABLE, MOTORCTRL_DRIVE_ENBL_FALSE);
#endif // ARDUINO_RT

#ifdef ARDUINO_RT
  Serial.begin(9600);
  while (! Serial); // Wait untilSerial is ready
  Serial.println("Test Monitor startup...");
#endif // ARDUINO_RT
}

// Interrupt callback for Timer5 (step counter)
ISR(MOTORCTRL_STEP_CNT_IRQ)
{
  irq_step_count_clbk();
}

#define LOOP_DUM_LEN 1L
static void irq_step_count_clbk()
{
#if MOTOR_DBG
    uint32_t i;
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("In IRQ callback.\r\n");
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("============\r\n");
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("motor_position_rel:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", motor_position_rel);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("motor_target_position_rel:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", motor_target_position_rel);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("motor_target_pwm_freq:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", motor_target_pwm_freq);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("motor_increment_pwm_freq:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", motor_increment_pwm_freq);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("motor_current_pwm_freq:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", motor_current_pwm_freq);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("motor_step_cnt_incr_curr:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", motor_step_cnt_incr_curr);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("motor_step_cnt_accel_incr_base:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", motor_step_cnt_accel_incr_base);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("motor_step_cnt_accel_incr_sum:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", motor_step_cnt_accel_incr_sum);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("motor_step_cnt_accel_incr_last:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", motor_step_cnt_accel_incr_last);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("============\r\n");
#endif // MOTOR_DBG
  int32_t remaining_distance;
  int32_t target_position_rel = motor_target_position_rel;
  int32_t position_rel = motor_position_rel;
  uint16_t new_pwm_freq;
  uint32_t new_step_cnt_value;

  position_rel = position_rel + get_cnt5() + motor_step_cnt_incr_curr;
  remaining_distance = target_position_rel - position_rel;
  motor_position_rel = position_rel;

#if MOTOR_DBG
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("remaining_distance:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", remaining_distance);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("target_position_rel:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", target_position_rel);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("position_rel:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", position_rel);
    for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print("motor_position_rel:"); for (i=0; i<LOOP_DUM_LEN; i++) ; debug_print(" %i\r\n", motor_position_rel);
  debug_print("============\r\n");
#endif // MOTOR_DBG

  // CASE: movement finished
  if (position_rel >= target_position_rel)
  {
#if MOTOR_DBG
    debug_print("Movement finished.\r\n");
#endif // MOTOR_DBG
    stop_pwm_step(1);
#if MOTOR_DBG
    debug_print("Motor stopped.\r\n");
#endif // MOTOR_DBG
    // Update absolute position with final relative one
    if (motor_direction == MOTORCTRL_DIR_FORWARD)
    {
      motor_position_abs += position_rel;
    }
    else
    {
      motor_position_abs -= position_rel;
    }
  }
  // CASE: no acceleraion / deceleration required
  else if(motor_accel_enbl == 0)
  {
    new_step_cnt_value = MIN(remaining_distance, COUNTER_STEP_MAX);
    set_threshold_cnt5((uint16_t) new_step_cnt_value);
  }
  // CASE: check if need to accelerate / deceleration
  else
  {
    // Acceleration
    if ((position_rel < target_position_rel/2) && (motor_current_pwm_freq < motor_target_pwm_freq))
    {
#if MOTOR_DBG
      debug_print("ACCELERATING.\r\n");
      debug_print("position_rel: %u \t", position_rel);
      debug_print("remaining_distance: %u \t", remaining_distance);
      debug_print("motor_current_pwm_freq: %u \t", motor_current_pwm_freq);
#endif // MOTOR_DBG
      // PWM
      new_pwm_freq = motor_current_pwm_freq + motor_increment_pwm_freq;
      // Step counter
      new_step_cnt_value = motor_step_cnt_accel_incr_last + motor_step_cnt_accel_incr_base;
      if (new_pwm_freq >= motor_target_pwm_freq) {
#if MOTOR_DBG
          debug_print("\nAccelerating too fast.\n");
#endif // MOTOR_DBG
          // Reaching max speed
          motor_pwm_freq_accel_last = motor_current_pwm_freq;
          //motor_step_cnt_accel_incr_last  // already done at previous call
          // Move at cruising speed
          new_step_cnt_value = remaining_distance - position_rel;
          new_step_cnt_value = MIN(new_step_cnt_value, COUNTER_STEP_MAX);
          new_pwm_freq = motor_target_pwm_freq;
      } else if (position_rel + new_step_cnt_value >= target_position_rel/2) {
#if MOTOR_DBG
          debug_print("\nAccelerating too far.\n");
#endif // MOTOR_DBG
          // Reaching middle of trip
          motor_pwm_freq_accel_last = motor_current_pwm_freq;
          // Move at new_pwm_freq to target_position_rel-position_rel
          // i.e., move by (target_position_rel-2*position_rel)
          // = remaining_distance - position_rel
          new_step_cnt_value = remaining_distance - position_rel;
      } else {
          // Still accelerating
          motor_step_cnt_accel_incr_last = new_step_cnt_value;
      }

#if MOTOR_DBG
      debug_print("new_pwm_freq: %u \r\n", new_pwm_freq);
      debug_print("new_pwm_freq2: %u \r\n", new_pwm_freq);
#endif // MOTOR_DBG
      set_freq_pwm_step(new_pwm_freq);
#if MOTOR_DBG
      debug_print("new_pwm_freq3: %u \r\n", new_pwm_freq);
#endif // MOTOR_DBG
      motor_current_pwm_freq = new_pwm_freq;

      motor_step_cnt_accel_incr_sum = position_rel;
#if MOTOR_DBG
      debug_print("mt_stp_cnt_ac_inc_last: %u\r\n", motor_step_cnt_accel_incr_last);
#endif // MOTOR_DBG
    }
    // Deceleration
    else if ((position_rel >= target_position_rel/2) && (remaining_distance <= motor_step_cnt_accel_incr_sum))
    {
#if MOTOR_DBG
      debug_print("DECELERATING.\r\n");
      debug_print("remaining_distance: %u \t", remaining_distance);
      debug_print("motor_current_pwm_freq: %u \t", motor_current_pwm_freq);
#endif // MOTOR_DBG
      // PWM
      if (motor_current_pwm_freq > motor_pwm_freq_accel_last) {
          new_pwm_freq = motor_pwm_freq_accel_last;
          // Step counter
          new_step_cnt_value = motor_step_cnt_accel_incr_last;
      } else {
          new_pwm_freq = motor_current_pwm_freq - motor_increment_pwm_freq;
          // Step counter
          new_step_cnt_value = motor_step_cnt_accel_incr_last - motor_step_cnt_accel_incr_base;
      }
      // Avoid potential rounding issues. Never reaches zero!
      new_pwm_freq = MAX(new_pwm_freq, motor_increment_pwm_freq);
#if MOTOR_DBG
      debug_print("new_pwm_freq: %u\r\n", new_pwm_freq);
#endif // MOTOR_DBG
      set_freq_pwm_step(new_pwm_freq);
      motor_current_pwm_freq = new_pwm_freq;

      //motor_step_cnt_accel_incr_sum = new_step_cnt_value;
      // Do not go too far
      new_step_cnt_value = MIN(new_step_cnt_value, remaining_distance);
      motor_step_cnt_accel_incr_last = new_step_cnt_value;
    }
    // Cruise speed
    else
    {
#if MOTOR_DBG
      debug_print("CRUISE SPEED.\r\n");
      //debug_print("motor_step_cnt_accel_incr_sum: %u \t", motor_step_cnt_accel_incr_sum);
      debug_print("remaining_distance: %u \t", remaining_distance);
#endif // MOTOR_DBG
      // Distance to go to decelerating point
      // motor_step_cnt_accel_incr_sum represents number of steps for accelerating ramp
      //set_freq_pwm_step(motor_current_pwm_freq);
      new_step_cnt_value = remaining_distance - motor_step_cnt_accel_incr_sum;
      new_step_cnt_value = MIN(new_step_cnt_value, COUNTER_STEP_MAX);
#if MOTOR_DBG
      debug_print("CRUISE SPEED.\r\n");
      debug_print("new_step_cnt_value: %u\r\n", new_step_cnt_value);
#endif // MOTOR_DBG
    }

    // Go to next position
#if MOTOR_DBG
    debug_print("new_step_cnt_value: %u\r\n", new_step_cnt_value);
#endif // MOTOR_DBG
    set_threshold_cnt5((uint16_t) new_step_cnt_value);
  }
}

// Minimum input period is 2
static void set_period_pwm_step(const unsigned int period)
{
  // The updates to these values are sampled when the counter reaches zero,
  // hence the following instructions should happen atomically most of the
  // time.

  //cli();//stop interrupts
  // Stop counter
  TCCR3B &= ~MOTORCTRL_PWM_RUN_VALUE;
  // Update TOP values
  OCR3A = period;
  OCR3B = period/2;
  OCR3C = period/2;
  // Start counter
  TCCR3B |= MOTORCTRL_PWM_RUN_VALUE;
  //sei();//allow interrupts
}

// Return minimum 2 (PWM requirement)
static unsigned int convert_freq2period_pwm_step(const unsigned int freq)
{
  unsigned int timer_period = MOTORCTRL_PWM_FREQ_DIV_FACTOR / freq;
  return MAX(timer_period,3);
}

// Maximum input freq is MOTORCTRL_PWM_FREQ_DIV_FACTOR/2
static void set_freq_pwm_step(const unsigned int freq)
{
#if MOTOR_DBG
      debug_print("set_freq_pwm_step: %u \r\n", freq);
#endif // MOTOR_DBG
  unsigned int timer_period = convert_freq2period_pwm_step(freq);
#if MOTOR_DBG
      debug_print("timer period: %u \r\n", timer_period);
#endif // MOTOR_DBG
  set_period_pwm_step(timer_period);
#if MOTOR_DBG
      debug_print("set_pwm_freq final\r\n");
#endif // MOTOR_DBG
}

// Maximum input freq is MOTORCTRL_PWM_FREQ_DIV_FACTOR/2
static void stop_pwm_step(uint32_t enable_interrupt)
{
  // 3 is the minimum period for the timer
  // setting OCR3B = OCR3A guarantees 0 output
  // setting a low period minimizes turn-on latency.
  // We first set OCR3B to COUNTER_STEP_MAX to guarantee absence of glitches.

  // Stop counter
  TCCR3B &= ~MOTORCTRL_PWM_RUN_VALUE;
  // Update TOP values to minimum
  OCR3B = 3;
  OCR3C = 3;
  OCR3A = 3;
  // Enable overflow interrupt (end of operations)
  if (enable_interrupt != 0) {
    TIFR3 |= _BV(TOV3); // Clear pending interrupt flag (if any)
    TIMSK3 |= MOTORCTRL_PWM_OVF_IRQ_CFG;
  }
  // Start counter
  TCCR3B |= MOTORCTRL_PWM_RUN_VALUE;
#if MOTOR_DBG
    debug_print("Stop pwm final.\r\n");
#endif // MOTOR_DBG
}

// Interrupt callback for Timer1 (PWM overflow)
ISR(MOTORCTRL_PWM_OVF_IRQ)
{
#if MOTOR_DBG
    debug_print("PWM ISR.\r\n");
#endif // MOTOR_DBG
  // Disable overflow interrupt
  TIMSK3 &= ~ MOTORCTRL_PWM_OVF_IRQ_CFG;
  // Report that motor is not in motion anymore
  motor_inmotion = 0;
#ifndef ARDUINO_RT
  BaseType_t higherPriorityTaskWoken;
  xTaskNotifyFromISR(motorControlTaskHandle,
          MOTOR_NOTIF_MOVEMENT_FINISHED,
          eSetBits,
          &higherPriorityTaskWoken);
  if (higherPriorityTaskWoken) {
      taskYIELD();
  }
#if MOTOR_DBG
    debug_print("notified control.\r\n");
#endif // MOTOR_DBG
  // should do a portYIELD_FROM_ISR(higherPriorityTaskWoken), but is not available in port
#endif // ARDUINO_RT
}

// Maximum input threshold is COUNTER_STEP_MAX
static void set_threshold_cnt5(const unsigned int thresh)
{
  motor_step_cnt_incr_curr = thresh;
  // Counter interrupt is off by one, so need to adapt threshold value
  OCR5A = thresh-1;
}

static void reset_cnt5()
{
  TCNT5 = 0;
}

static unsigned int get_cnt5()
{
  return TCNT5;
}

uint8_t motor_moving() {
    return motor_inmotion;
}

uint32_t motor_current_position() {
  cli();//stop interrupts
  uint32_t curr_pos = motor_position_abs;
  uint32_t curr_pos_offset = get_cnt5();
  uint8_t cur_direction = motor_direction;
  sei();//allow interrupts
  if(motor_inmotion) {
      if (cur_direction == MOTORCTRL_DIR_FORWARD) {
          curr_pos += curr_pos_offset;
      } else {
          curr_pos -= curr_pos_offset;
      }
  }
  return curr_pos;
}

uint32_t motor_remaining_distance(){
  uint32_t m_curr_pos_abs = motor_current_position();
  if (m_curr_pos_abs > motor_target_position_abs) { 
      return m_curr_pos_abs - motor_target_position_abs;  
  } else {
      return motor_target_position_abs - m_curr_pos_abs; 
  }
}


void motor_enable()
{
#ifdef ARDUINO_RT
      digitalWrite(MOTORCTRL_DRIVE_ENBL_PIN, MOTORCTRL_DRIVE_ENBL_TRUE);
#else
      dio_write(DIO_PIN_MOTOR_ENABLE, MOTORCTRL_DRIVE_ENBL_TRUE);
#endif // ARDUINO_RT
}

void motor_disable()
{
#ifdef ARDUINO_RT
      digitalWrite(MOTORCTRL_DRIVE_ENBL_PIN, MOTORCTRL_DRIVE_ENBL_FALSE);
#else
      dio_write(DIO_PIN_MOTOR_ENABLE, MOTORCTRL_DRIVE_ENBL_FALSE);
#endif // ARDUINO_RT
}

void set_motor_goto_position_accel_exec(
        uint32_t target_position_abs,
        const uint16_t target_speed,
        const uint16_t step_num_base,
        const uint16_t step_freq_base)
{
  uint8_t direction;
  long target_position_rel;
  uint16_t selected_speed;
  uint8_t accel_enbl;

  if (motor_inmotion == 0)
  {
    reset_cnt5();

    // First compute the requested absolute and relative position and update motor direction
    target_position_rel = target_position_abs - motor_position_abs;
    if (target_position_rel >= 0)
    {
#if DEBUG_MOTOR
      debug_print("Forward %lu \r\n",motor_position_abs);
#endif
      direction = MOTORCTRL_DIR_FORWARD;
      if (target_position_rel == 1) // Special case: counter config is problematic in that case
      {
        target_position_abs += 1;
        target_position_rel = 2; // Will go a little bit too far
      }
    }
    else
    {
#if DEBUG_MOTOR
      debug_print("Backward %lu \r\n",motor_position_abs);
#endif
      direction = MOTORCTRL_DIR_BACKWARD;
      target_position_rel = -target_position_rel;
      if (target_position_rel == 1) // Special case: counter config is problematic in that case
      {
        target_position_abs -= 1;
        target_position_rel = 2; // Will go a little bit too far
      }
    }
    motor_target_position_abs = target_position_abs;
    motor_target_position_rel = target_position_rel;
    motor_position_rel = 0;

    //debug_print("To direction %u\r\n", direction);
#ifdef ARDUINO_RT
    digitalWrite(MOTORCTRL_DRIVE_DIR_PIN, direction);
#else
    dio_write(DIO_PIN_MOTOR_DIRECTION, direction);
#endif // ARDUINO_RT
    motor_direction = direction;

    // Then start PWM and set the step counter to necessary threshold
    if (target_position_rel > 0)
    {
      // CASE: small movement. No time to accelerate / decelerate
      //
      // Assume 2*step_num_base is always < COUNTER_STEP_MAX
      // 2* since there is acceleration and deceleration
      if (target_position_rel <= 2*step_num_base)
      {
        debug_print("No time to accelerate/decelerate.\r\n");
        accel_enbl = 0;
        set_threshold_cnt5(target_position_rel);
        selected_speed = MIN(MOTORCTRL_MAX_SPEED_SMALL_MVMT,target_speed);
      }
      // CASE: slow movement
      else if (target_speed == step_freq_base)
      {
        debug_print("Slow movement.\r\n");
        accel_enbl = 0;
        if (target_position_rel <= COUNTER_STEP_MAX)
        {
          set_threshold_cnt5(target_position_rel);
        }
        else
        {
          set_threshold_cnt5(COUNTER_STEP_MAX);
        }
        selected_speed = step_freq_base;
      }
      // CASE: acceleration needed
      else
      {
        debug_print("Acceleration needed.\r\n");
        accel_enbl = 1;
        set_threshold_cnt5(step_num_base);
        selected_speed = step_freq_base;
        motor_step_cnt_accel_incr_base = step_num_base;
        motor_step_cnt_accel_incr_last = step_num_base;
        motor_step_cnt_accel_incr_sum = 0;
      }

      //motor_enable();
      motor_inmotion = 1;

      // Take PWM freq
      motor_current_pwm_freq = selected_speed;
      if (accel_enbl == 0)
      {
        motor_increment_pwm_freq = 0;
        motor_target_pwm_freq = selected_speed;
      }
      else
      {
        motor_increment_pwm_freq = selected_speed;
        motor_target_pwm_freq = target_speed;
      }
      cli();
      set_freq_pwm_step(selected_speed);
      sei();
      motor_accel_enbl = accel_enbl;
    }
    else
    {
        //motor_disable();
    }
  }
}

// Used to manually set the absolute position of the motor
// Dirty Hack to handle negative target position
// TODO: clean
void set_motor_current_position_value(long new_abs_position){
    motor_position_abs = new_abs_position;
}

// Used when the motor need to be stopped before it ends 
// its movement
// TODO: dirty, clean
void motor_anticipated_stop(){
#if DEBUG_MOTOR
    debug_print("ANT STATES: %lu %lu \r\n",motor_position_abs,motor_position_abs + get_cnt5());
#endif

    // stop PWM stepper
    stop_pwm_step(0);
    
    // set stop state
    motor_inmotion = 0;

    // Update states
    if (motor_direction == MOTORCTRL_DIR_FORWARD)
    {
      motor_position_abs += get_cnt5();
    }
    else
    {
      motor_position_abs -= get_cnt5();
    }

#if DEBUG_MOTOR
    debug_print("motor ANTICIPATED STOP \r\n");
#endif
}

void set_motor_goto_position(uint32_t target_position_abs, const uint16_t target_speed)
{
  // Special case: behavior like slow movement
  // No acceleration / deceleration
  set_motor_goto_position_accel_exec(target_position_abs, target_speed, 0, target_speed);
}

#ifdef ARDUINO_RT
void loop()
{
  uint16_t test_motor_speed_step;
  uint16_t test_step_num_base;
  uint16_t test_motor_speed; // Max MOTORCTRL_PWM_FREQ_DIV_FACTOR/2
  uint32_t test_motor_target_pos;
  uint32_t test_motor_target_pos_limit;

/*
  // Simple test case for constant speed movement
  test_motor_speed = 5000; // Max MOTORCTRL_PWM_FREQ_DIV_FACTOR/2
  test_motor_target_pos = 100000;
  test_motor_target_pos_limit = 2000;

  //while(1) {

    set_motor_goto_position(test_motor_target_pos, test_motor_speed);
    test_motor_target_pos +=40;
    if (test_motor_target_pos > test_motor_target_pos_limit)
    {
      test_motor_target_pos = 0;
    }

    while (motor_inmotion)
    {
        delay(1000);
    }
  //}
*/


  // Simple test case for accel / decel movement
  test_motor_speed_step = 200;
  test_step_num_base = 2;
  test_motor_speed = 10000; // Max MOTORCTRL_PWM_FREQ_DIV_FACTOR/2
  test_motor_target_pos = 40000;
  test_motor_target_pos_limit = 20;

  set_motor_goto_position_accel_exec(test_motor_target_pos, test_motor_speed, test_step_num_base, test_motor_speed_step);

  while (motor_inmotion)
  {
      delay(100);
  }

  test_motor_target_pos *= 2;

  //set_motor_goto_position_accel_exec(test_motor_target_pos, test_motor_speed, test_step_num_base, test_motor_speed_step);

  while(1) {}

}
#endif // ARDUINO_RT
