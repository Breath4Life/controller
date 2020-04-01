////////////////////////////////////////////////////////////////////////////////
// Name: motorControl_pwmHard.ino
//
// Descr: First sketch about motor control based on hard PWM and hard Counter
//        Targe is Arduino Mega
//        Not yet working with BoL motor
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


#include <stdint.h>
#include <limits.h>

#include <avr/interrupt.h>

#define MOTORCTRL_DIR_FORWARD 0
#define MOTORCTRL_DIR_BACKWARD 1
#define MOTORCTRL_DRIVE_ENBL_TRUE 0
#define MOTORCTRL_DRIVE_ENBL_FALSE 1

#define MOTORCTRL_PWM_RUN_VALUE _BV(CS32)
#define MOTORCTRL_PWM_OVF_IRQ_CFG _BV(TOIE3)
#define MOTORCTRL_PWM_OVF_IRQ TIMER3_OVF_vect

#define MOTORCTRL_STEP_CNT_IRQ TIMER5_COMPA_vect

#define COUNTER_STEP_MAX 0xFFFF

#ifndef RT_B4L
#define ARDUINO_RT
#endif // RT_B4L

#ifndef ARDUINO_RT
#include "pins.h"
#include "io.h"
#endif // ARDUINO_RT

#ifdef ARDUINO_RT
const uint8_t MOTORCTRL_DRIVE_DIR_PIN = 22;
const uint8_t MOTORCTRL_DRIVE_STEP_PIN = 2; // pwm_step (OC3B)
const uint8_t MOTORCTRL_DRIVE_ENBL_PIN = 24;

const uint8_t MOTORCTRL_STEP_CNT_PIN = 47; // CNT5 input
const uint8_t MOTORCTRL_STEP_CNT_OUT_DBG_PIN = 25;
const uint8_t MOTORCTRL_STEP_CNT_OUT_DBG_PIN2 = 27;
#endif // ARDUINO_RT

const unsigned int TIM1_FREQ_DIV_FACTOR = 31250;

// We use signed number for position to account for potential extra steps while being at homing position
static volatile long motor_position = 0;
static volatile long motor_target_position = 0;
static volatile uint8_t motor_direction = 0;
static volatile uint8_t motor_inmotion = 0;
static volatile int  motor_step_cnt_incr = 0; 

static void setup_pwm_step();
static void disable_cnt5_irq();
static void enable_cnt5_irq();
static void setup_ext_cnt5();
static void irq_step_count_clbk();
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
  OCR3A = 3;
  OCR3B = COUNTER_STEP_MAX;
  // WGM mode 9 (PWM, Phase and Frequency Correct - OCRn)
  // pre-scaler: div-256
  // OCR3A fixes the period
  // OCR3B fixes duty cycle
  TCCR3A = _BV(COM3A0) | _BV(COM3B1) | _BV(COM3B0) | _BV(WGM30);
  TCCR3B |= _BV(WGM33);

  // Start
  TCCR3B |= MOTORCTRL_PWM_RUN_VALUE;

  // Disable interrupts
  TIMSK3 = 0;

#ifdef ARDUINO_RT
  pinMode(MOTORCTRL_DRIVE_STEP_PIN, OUTPUT);
  //pinMode(5, OUTPUT); // to debug PWM, always duty cycle 1/2
#else
  dio_init(DIO_PIN_MOTOR_STEP, DIO_OUTPUT);
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
  OCR5A = TIM1_FREQ_DIV_FACTOR; // By default
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
#else
  // TMC Dir pin
  dio_init(DIO_PIN_MOTOR_DIRECTION, DIO_OUTPUT);
  // TMC enable pin
  dio_init(DIO_PIN_MOTOR_ENABLE, DIO_OUTPUT);
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

static void irq_step_count_clbk()
{
  long remaining_distance;

  if (motor_direction == MOTORCTRL_DIR_FORWARD)
  {
    // Before interrupt, counter is reset
    // if execution is not fast enough, take remaining steps (if any)
    motor_position += get_cnt5();
    motor_position += motor_step_cnt_incr;

    if (motor_position >= motor_target_position)
    {
      stop_pwm_step();
    }
    else
    {
      remaining_distance = motor_target_position - motor_position;
      if (remaining_distance <= COUNTER_STEP_MAX)
      {
        set_threshold_cnt5(remaining_distance);
      }
      else
      {
        set_threshold_cnt5(COUNTER_STEP_MAX);
      }
    }
  }
  else
  {
    motor_position -= get_cnt5();
    motor_position -= motor_step_cnt_incr;
    if (motor_position <= motor_target_position)
    {
      stop_pwm_step();
    }
    else
    {
      remaining_distance = motor_position - motor_target_position;
      if (remaining_distance <= COUNTER_STEP_MAX)
      {
        set_threshold_cnt5(remaining_distance);
      }
      else
      {
        set_threshold_cnt5(COUNTER_STEP_MAX);
      }
    }
  }
}

// Maximum input freq is TIM1_FREQ_DIV_FACTOR/2
static void set_freq_pwm_step(const unsigned int freq)
{
  unsigned int timer1_period;
  timer1_period = TIM1_FREQ_DIV_FACTOR / freq;
  // The updates to these values are sampled when the counter reaches zero,
  // hence the following instructions should happen atomically most of the
  // time.

  cli();//stop interrupts
  // Stop counter
  TCCR3B &= ~MOTORCTRL_PWM_RUN_VALUE;
  // Update TOP values
  OCR3A = timer1_period;
  OCR3B = timer1_period/2;
  // Start counter
  TCCR3B |= MOTORCTRL_PWM_RUN_VALUE;
  sei();//allow interrupts
}

// Maximum input freq is TIM1_FREQ_DIV_FACTOR/2
static void stop_pwm_step()
{
  cli();//stop interrupts
  // 3 is the minimum period for the timer
  // setting OCR3B = OCR3A guarantees 0 output
  // setting a low period minimizes turn-on latency.
  // We first set OCR3B to COUNTER_STEP_MAX to guarantee absence of glitches.

  // Stop counter
  TCCR3B &= ~MOTORCTRL_PWM_RUN_VALUE;
  // Update TOP values to minimum
  OCR3B = 3;
  OCR3A = 3;  
  // Enable overflow interrupt (end of operations)
  TIFR3 |= _BV(TOV3); // Clear pending interrupt flag (if any)
  TIMSK3 |= MOTORCTRL_PWM_OVF_IRQ_CFG;
  // Start counter
  TCCR3B |= MOTORCTRL_PWM_RUN_VALUE;
  sei();//allow interrupts
}

// Interrupt callback for Timer1 (PWM overflow)
ISR(MOTORCTRL_PWM_OVF_IRQ)
{
  //digitalWrite(MOTORCTRL_STEP_CNT_OUT_DBG_PIN, 1);
  // Disable overflow interrupt
  TIMSK3 &= ~ MOTORCTRL_PWM_OVF_IRQ_CFG;
  // Report that motor is not in motion anymore
  motor_inmotion = 0;
}

// Maximum input threshold is COUNTER_STEP_MAX
static void set_threshold_cnt5(const unsigned int thresh)
{
  motor_step_cnt_incr = thresh;
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

int32_t motor_remaining_distance() {
  cli();//stop interrupts
  int32_t curr_pos = motor_position;
  int32_t curr_pos_offset = get_cnt5();
  uint8_t cur_direction = motor_direction;
  sei();//allow interrupts
  if (cur_direction == MOTORCTRL_DIR_FORWARD) {
      curr_pos += curr_pos_offset;
  } else {
      curr_pos -= curr_pos_offset;
  }
  return curr_pos;
}

void set_motor_goto_position(const unsigned long position, const unsigned int speed)
{
  long remaining_distance;

  if (motor_inmotion == 0)
  {
    reset_cnt5();

    motor_target_position = position;
    remaining_distance = motor_target_position - motor_position;
    if (remaining_distance >= 0)
    {
      motor_direction = MOTORCTRL_DIR_FORWARD;
      if (remaining_distance == 1) // Special case: counter config is problematic in that case
      {
        motor_target_position += 1;
        remaining_distance = 2; // Will go a little bit too far
      }
    }
    else
    {
      motor_direction = MOTORCTRL_DIR_BACKWARD;
      remaining_distance = -remaining_distance;
      if (remaining_distance == 1) // Special case: counter config is problematic in that case
      {
        motor_target_position -= 1;
        remaining_distance = 2; // Will go a little bit too far
      }
    }

#ifdef ARDUINO_RT
    digitalWrite(MOTORCTRL_DRIVE_DIR_PIN, motor_direction);
#else
    dio_write(DIO_PIN_MOTOR_DIRECTION, motor_direction);
#endif // ARDUINO_RT


    if (remaining_distance > 0)
    {
      if (remaining_distance <= COUNTER_STEP_MAX)
      {
        set_threshold_cnt5(remaining_distance);
      }
      else
      {
        set_threshold_cnt5(COUNTER_STEP_MAX);
      }

#ifdef ARDUINO_RT
      digitalWrite(MOTORCTRL_DRIVE_ENBL_PIN, MOTORCTRL_DRIVE_ENBL_TRUE);
#else
    dio_write(DIO_PIN_MOTOR_ENABLE, MOTORCTRL_DRIVE_ENBL_TRUE);
#endif // ARDUINO_RT
      motor_inmotion = 1;
      set_freq_pwm_step(speed);
      // Verify delay between set_freq_pwm_step() and real start of PWM.
      //digitalWrite(MOTORCTRL_STEP_CNT_OUT_DBG_PIN, 1);
    }
    else
    {
#ifdef ARDUINO_RT
      digitalWrite(MOTORCTRL_DRIVE_ENBL_PIN, MOTORCTRL_DRIVE_ENBL_FALSE);
#else
    dio_write(DIO_PIN_MOTOR_ENABLE, MOTORCTRL_DRIVE_ENBL_FALSE);
#endif // ARDUINO_RT
    }
  }
}

#ifdef ARDUINO_RT
void loop()
{

  unsigned int motor_speed = 1000; // Max TIM1_FREQ_DIV_FACTOR/2
  unsigned long motor_target_pos = 10;
  unsigned long motor_target_pos_limit = 20;

  while(1) {

    set_motor_goto_position(motor_target_pos, motor_speed);
    motor_target_pos +=4;
    if (motor_target_pos > motor_target_pos_limit)
    {
      motor_target_pos = 0;
    }

    while (motor_inmotion)
    {
        delay(100);
    }
  }
}
#endif // ARDUINO_RT
