/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * stepper.cpp - A singleton object to execute motion plans using stepper motors
 * Marlin Firmware
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "endstops.h"
#include "planner.h"
#include "ultralcd.h"
#include "language.h"

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

Stepper stepper; // Singleton

// public:

block_t* Stepper::current_block = NULL;  // A pointer to the block currently being traced

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  bool Stepper::abort_on_endstop_hit = false;
#endif

#if ENABLED(Z_DUAL_ENDSTOPS)
  bool Stepper::performing_homing = false;
#endif

// private:

uint8_t Stepper::last_direction_bits = 0;        // The next stepping-bits to be output
uint16_t Stepper::cleaning_buffer_counter = 0;

#if ENABLED(Z_DUAL_ENDSTOPS)
  bool Stepper::locked_z_motor = false;
  bool Stepper::locked_z2_motor = false;
#endif

int32_t  Stepper::counter_X = 0,
      Stepper::counter_Y = 0,
      Stepper::counter_Z = 0;

volatile uint32_t Stepper::step_events_completed = 0; // The number of step events executed in the current block


int32_t Stepper::acceleration_time, Stepper::deceleration_time;

volatile int32_t Stepper::count_position[NUM_AXIS] = { 0 };
volatile int8_t Stepper::count_direction[NUM_AXIS] = { 1, 1, 1 };

bool Stepper::axis_locked[NUM_AXIS] = {false, false, false};


uint16_t Stepper::acc_step_rate; // needed for deceleration start point
uint8_t Stepper::step_loops, Stepper::step_loops_nominal;
uint16_t Stepper::timer_nominal;

volatile int32_t Stepper::endstops_trigsteps[3];

#if ENABLED(X_DUAL_STEPPER_DRIVERS)
  #define X_APPLY_DIR(v,Q) do{ X_DIR_WRITE(v); X2_DIR_WRITE((v) != INVERT_X2_VS_X_DIR); }while(0)
  #define X_APPLY_STEP(v,Q) do{ X_STEP_WRITE(v); X2_STEP_WRITE(v); }while(0)
#else
  #define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
  #define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)
#endif

#if ENABLED(Y_DUAL_STEPPER_DRIVERS)
  #define Y_APPLY_DIR(v,Q) do{ Y_DIR_WRITE(v); Y2_DIR_WRITE((v) != INVERT_Y2_VS_Y_DIR); }while(0)
  #define Y_APPLY_STEP(v,Q) do{ Y_STEP_WRITE(v); Y2_STEP_WRITE(v); }while(0)
#else
  #define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
  #define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)
#endif

#if ENABLED(Z_DUAL_STEPPER_DRIVERS)
  #define Z_APPLY_DIR(v,Q) do{ Z_DIR_WRITE(v); Z2_DIR_WRITE(v); }while(0)
  #if ENABLED(Z_DUAL_ENDSTOPS)
    #define Z_APPLY_STEP(v,Q) \
    if (performing_homing) { \
      if (Z_HOME_DIR > 0) {\
        if (!(TEST(endstops.old_endstop_bits, Z_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z2_MAX) && (count_direction[Z_AXIS] > 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
      } \
      else { \
        if (!(TEST(endstops.old_endstop_bits, Z_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z_motor) Z_STEP_WRITE(v); \
        if (!(TEST(endstops.old_endstop_bits, Z2_MIN) && (count_direction[Z_AXIS] < 0)) && !locked_z2_motor) Z2_STEP_WRITE(v); \
      } \
    } \
    else { \
      Z_STEP_WRITE(v); \
      Z2_STEP_WRITE(v); \
    }
  #else
    #define Z_APPLY_STEP(v,Q) do{ Z_STEP_WRITE(v); Z2_STEP_WRITE(v); }while(0)
  #endif
#else
  #define Z_APPLY_DIR(v,Q) Z_DIR_WRITE(v)
  #define Z_APPLY_STEP(v,Q) Z_STEP_WRITE(v)
#endif

#define MultiU24X32toH16(intRes, longIn1, longIn2) intRes = ((uint64_t)longIn1 * longIn2) >> 24

/**
 *         __________________________
 *        /|                        |\     _________________         ^
 *       / |                        | \   /|               |\        |
 *      /  |                        |  \ / |               | \       s
 *     /   |                        |   |  |               |  \      p
 *    /    |                        |   |  |               |   \     e
 *   +-----+------------------------+---+--+---------------+----+    e
 *   |               BLOCK 1            |      BLOCK 2          |    d
 *
 *                           time ----->
 *
 *  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
 *  first block->accelerate_until step_events_completed, then keeps going at constant speed until
 *  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
 *  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.
 */
void Stepper::wake_up() {
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

/**
 * Set the stepper direction of each axis
 *
 *   COREXY: X_AXIS=A_AXIS and Y_AXIS=B_AXIS
 *   COREXZ: X_AXIS=A_AXIS and Z_AXIS=C_AXIS
 *   COREYZ: Y_AXIS=B_AXIS and Z_AXIS=C_AXIS
 */
void Stepper::set_directions() {

  #define SET_STEP_DIR(AXIS) \
    if(!axis_locked[AXIS ##_AXIS]) { \
      if (motor_direction(AXIS ##_AXIS)) { \
        AXIS ##_APPLY_DIR(INVERT_## AXIS ##_DIR, false); \
        count_direction[AXIS ##_AXIS] = -1; \
      } \
      else { \
        AXIS ##_APPLY_DIR(!INVERT_## AXIS ##_DIR, false); \
        count_direction[AXIS ##_AXIS] = 1; \
      } \
    }

  SET_STEP_DIR(X); // A
  SET_STEP_DIR(Y); // B
  SET_STEP_DIR(Z); // C
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
HAL_STEP_TIMER_ISR {
  HAL_timer_isr_prologue(STEP_TIMER_NUM);
  Stepper::isr();
}
void Stepper::isr() {
  if (cleaning_buffer_counter) {
    current_block = NULL;
    planner.discard_current_block();
    #ifdef SD_FINISHED_RELEASECOMMAND
      if ((cleaning_buffer_counter == 1) && (SD_FINISHED_STEPPERRELEASE)) enqueue_and_echo_commands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    #endif
    cleaning_buffer_counter--;
    HAL_timer_set_compare(STEP_TIMER_NUM, 200);
    return;
  }

  // If there is no current block, attempt to pop one from the buffer
  if (!current_block) {
    // Anything in the buffer?

    current_block = planner.get_current_block();
    if (current_block) {
      current_block->busy = true;
      trapezoid_generator_reset();

      // Initialize Bresenham counters to 1/2 the ceiling
      counter_X = counter_Y = counter_Z = -(current_block->step_event_count >> 1);

      step_events_completed = 0;

      #if ENABLED(Z_LATE_ENABLE)
        if (current_block->steps[Z_AXIS] > 0) {
          enable_z();
          HAL_timer_set_compare(STEP_TIMER_NUM, STEPPER_TIMER_RATE / 1000); //1ms wait
          return;
        }
      #endif
    }
    else {
      HAL_timer_set_compare(STEP_TIMER_NUM, STEPPER_TIMER_RATE / 1000); // 1kHz.
    }
  }

  if (current_block) {

    // Update endstops state, if enabled
    if (endstops.enabled
    ) endstops.update();

    // Take multiple steps per interrupt (For high speed moves)
    for (int8_t i = 0; i < step_loops; i++) {
      #ifndef USBCON
        customizedSerial.checkRx(); // Check for serial chars.
      #endif

      #define _COUNTER(AXIS) counter_## AXIS
      #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
      #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

      #define STEP_ADD(AXIS) \
        _COUNTER(AXIS) += current_block->steps[_AXIS(AXIS)]; \
        if (_COUNTER(AXIS) > 0 && !axis_locked[_AXIS(AXIS)]) { \
          _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0); \
        }

      STEP_ADD(X);
      STEP_ADD(Y);
      STEP_ADD(Z);

      delayMicroseconds(3);

      #define STEP_IF_COUNTER(AXIS) \
        if (_COUNTER(AXIS) > 0) { \
          _COUNTER(AXIS) -= current_block->step_event_count; \
          if (!axis_locked[_AXIS(AXIS)]) { \
            count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
            _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
          } \
        }

      STEP_IF_COUNTER(X);
      STEP_IF_COUNTER(Y);
      STEP_IF_COUNTER(Z);

      step_events_completed++;
      if (step_events_completed >= current_block->step_event_count) break;
    }


    // Calculate new timer value
    uint16_t timer, step_rate;
    if (step_events_completed <= (uint32_t)current_block->accelerate_until) {

      MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;

      // upper limit
      NOMORE(acc_step_rate, current_block->nominal_rate);

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      HAL_timer_set_compare(STEP_TIMER_NUM, timer);
      acceleration_time += timer;


    }
    else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
      MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);

      if (step_rate <= acc_step_rate) { // Still decelerating?
        step_rate = acc_step_rate - step_rate;
        NOLESS(step_rate, current_block->final_rate);
      }
      else
        step_rate = current_block->final_rate;

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      HAL_timer_set_compare(STEP_TIMER_NUM, timer);
      deceleration_time += timer;
    }
    else {
      HAL_timer_set_compare(STEP_TIMER_NUM, timer_nominal);
      // ensure we're running at the correct step rate, even if we just came off an acceleration
      step_loops = step_loops_nominal;
    }

    // OCR1A = (OCR1A < (TCNT1 + 16)) ? (TCNT1 + 16) : OCR1A;

    // If current block is finished, reset pointer
    if (step_events_completed >= current_block->step_event_count) {
      refresh_cmd_timeout();
      current_block = NULL;
      planner.discard_current_block();
    }
  }
}

void Stepper::init() {
  digipot_init(); //Initialize Digipot Motor Current
  microstep_init(); //Initialize Microstepping Pins

  // initialise TMC Steppers
  #if ENABLED(HAVE_TMCDRIVER)
    tmc_init();
  #endif
    // initialise L6470 Steppers
  #if ENABLED(HAVE_L6470DRIVER)
    L6470_init();
  #endif

  // Initialize Dir Pins
  #if HAS_X_DIR
    X_DIR_INIT;
  #endif
  #if HAS_X2_DIR
    X2_DIR_INIT;
  #endif
  #if HAS_Y_DIR
    Y_DIR_INIT;
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS_Y2_DIR
      Y2_DIR_INIT;
    #endif
  #endif
  #if HAS_Z_DIR
    Z_DIR_INIT;
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS_Z2_DIR
      Z2_DIR_INIT;
    #endif
  #endif

  //Initialize Enable Pins - steppers default to disabled.
  #if HAS_X_ENABLE
    X_ENABLE_INIT;
    if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
  #endif

  #if HAS_Y_ENABLE
    Y_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS_Y2_ENABLE
      Y2_ENABLE_INIT;
      if (!Y_ENABLE_ON) Y2_ENABLE_WRITE(HIGH);
    #endif
  #endif

  #if HAS_Z_ENABLE
    Z_ENABLE_INIT;
    if (!Z_ENABLE_ON) Z_ENABLE_WRITE(HIGH);
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS_Z2_ENABLE
      Z2_ENABLE_INIT;
      if (!Z_ENABLE_ON) Z2_ENABLE_WRITE(HIGH);
    #endif
  #endif

  //
  // Init endstops and pullups here
  //
  endstops.init();

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(axis) disable_## axis()

  #define AXIS_INIT(axis, AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(axis)

  // Initialize Step Pins
  #if HAS_X_STEP
    #if ENABLED(X_DUAL_STEPPER_DRIVERS)
      X2_STEP_INIT;
      X2_STEP_WRITE(INVERT_X_STEP_PIN);
    #endif
    AXIS_INIT(x, X, X);
  #endif

  #if HAS_Y_STEP
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS)
      Y2_STEP_INIT;
      Y2_STEP_WRITE(INVERT_Y_STEP_PIN);
    #endif
    AXIS_INIT(y, Y, Y);
  #endif

  #if HAS_Z_STEP
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS)
      Z2_STEP_INIT;
      Z2_STEP_WRITE(INVERT_Z_STEP_PIN);
    #endif
    AXIS_INIT(z, Z, Z);
  #endif

  HAL_timer_start(STEP_TIMER_NUM, 100);
  ENABLE_STEPPER_DRIVER_INTERRUPT();

  endstops.enable(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_directions(); // Init directions to last_direction_bits = 0
}

/**
 * Block until all buffered steps are executed
 */
void Stepper::synchronize()
{
  while (planner.blocks_queued())
    idle();
}

/**
 * Leave axis pins untouched for external controller to use it
 */
void Stepper::leave_control_on(AxisEnum axis)
{
  axis_locked[axis] = true;
}

/**
 * Take control back on axis pins
 */
void Stepper::take_control_on(AxisEnum axis)
{
  axis_locked[axis] = false;
  set_directions();
}

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XYZ steps.
 * For CORE machines XYZ needs to be translated to ABC.
 *
 * This allows get_axis_position_mm to correctly
 * derive the current XYZ position later on.
 */
void Stepper::set_position(const int32_t& x, const int32_t& y, const int32_t& z) {
  CRITICAL_SECTION_START;

  #if ENABLED(COREXY)
    // corexy positioning
    // these equations follow the form of the dA and dB equations on http://www.corexy.com/theory.html
    count_position[A_AXIS] = x + y;
    count_position[B_AXIS] = x - y;
    count_position[Z_AXIS] = z;
  #elif ENABLED(COREXZ)
    // corexz planning
    count_position[A_AXIS] = x + z;
    count_position[Y_AXIS] = y;
    count_position[C_AXIS] = x - z;
  #elif ENABLED(COREYZ)
    // coreyz planning
    count_position[X_AXIS] = x;
    count_position[B_AXIS] = y + z;
    count_position[C_AXIS] = y - z;
  #else
    // default non-h-bot planning
    count_position[X_AXIS] = x;
    count_position[Y_AXIS] = y;
    count_position[Z_AXIS] = z;
  #endif

  CRITICAL_SECTION_END;
}

/**
 * Get a stepper's position in steps.
 */
int32_t Stepper::position(AxisEnum axis) {
  CRITICAL_SECTION_START;
  int32_t count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

/**
 * Get an axis position according to stepper position(s)
 * For CORE machines apply translation from ABC to XYZ.
 */
float Stepper::get_axis_position_mm(AxisEnum axis) {
  float axis_steps;
  #if ENABLED(COREXY) || ENABLED(COREXZ) || ENABLED(COREYZ)
    // Requesting one of the "core" axes?
    if (axis == CORE_AXIS_1 || axis == CORE_AXIS_2) {
      CRITICAL_SECTION_START;
      int32_t pos1 = count_position[CORE_AXIS_1],
           pos2 = count_position[CORE_AXIS_2];
      CRITICAL_SECTION_END;
      // ((a1+a2)+(a1-a2))/2 -> (a1+a2+a1-a2)/2 -> (a1+a1)/2 -> a1
      // ((a1+a2)-(a1-a2))/2 -> (a1+a2-a1+a2)/2 -> (a2+a2)/2 -> a2
      axis_steps = (pos1 + ((axis == CORE_AXIS_1) ? pos2 : -pos2)) * 0.5f;
    }
    else
      axis_steps = position(axis);
  #else
    axis_steps = position(axis);
  #endif
  return axis_steps * planner.steps_to_mm[axis];
}

void Stepper::finish_and_disable() {
  synchronize();
  disable_all_steppers();
}

void Stepper::quick_stop() {
  cleaning_buffer_counter = 5000;
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (planner.blocks_queued()) planner.discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

void Stepper::endstop_triggered(AxisEnum axis) {
  #if ENABLED(COREXY) || ENABLED(COREXZ) || ENABLED(COREYZ)
    float axis_pos = count_position[axis];
    if (axis == CORE_AXIS_1)
      axis_pos = (axis_pos + count_position[CORE_AXIS_2]) * 0.5;
    else if (axis == CORE_AXIS_2)
      axis_pos = (count_position[CORE_AXIS_1] - axis_pos) * 0.5;
    endstops_trigsteps[axis] = axis_pos;
  #else // !COREXY && !COREXZ && !COREYZ
    endstops_trigsteps[axis] = count_position[axis] + count_direction[axis];
  #endif // !COREXY && !COREXZ && !COREYZ

  kill_current_block();
}

void Stepper::report_positions() {
  CRITICAL_SECTION_START;
  int32_t xpos = count_position[X_AXIS],
       ypos = count_position[Y_AXIS],
       zpos = count_position[Z_AXIS];
  CRITICAL_SECTION_END;

  #if ENABLED(COREXY) || ENABLED(COREXZ)
    SERIAL_PROTOCOLPGM(MSG_COUNT_A);
  #else
    SERIAL_PROTOCOLPGM(MSG_COUNT_X);
  #endif
  SERIAL_PROTOCOL(xpos);

  #if ENABLED(COREXY) || ENABLED(COREYZ)
    SERIAL_PROTOCOLPGM(" B:");
  #else
    SERIAL_PROTOCOLPGM(" Y:");
  #endif
  SERIAL_PROTOCOL(ypos);

  #if ENABLED(COREXZ) || ENABLED(COREYZ)
    SERIAL_PROTOCOLPGM(" C:");
  #else
    SERIAL_PROTOCOLPGM(" Z:");
  #endif
  SERIAL_PROTOCOL(zpos);

  SERIAL_EOL;
}

void Stepper::shift_z_position(int8_t shift)
{
  count_position[Z_AXIS] += shift;
}

/**
 * Software-controlled Stepper Motor Current
 */

#if HAS_DIGIPOTSS
  // From Arduino DigitalPotControl example
  void Stepper::digitalPotWrite(int16_t address, int16_t value) {
    WRITE(DIGIPOTSS_PIN, LOW); // take the SS pin low to select the chip
    SPI.transfer(address); //  send in the address and value via SPI:
    SPI.transfer(value);
    WRITE(DIGIPOTSS_PIN, HIGH); // take the SS pin high to de-select the chip:
    //delay(10);
  }
#endif //HAS_DIGIPOTSS

void Stepper::digipot_init() {
  #if HAS_DIGIPOTSS
    const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;

    SPI.begin();
    SET_OUTPUT(DIGIPOTSS_PIN);
    for (uint8_t i = 0; i < COUNT(digipot_motor_current); i++) {
      //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
      digipot_current(i, digipot_motor_current[i]);
    }
  #endif
  #if HAS_MOTOR_CURRENT_PWM
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
      SET_OUTPUT(MOTOR_CURRENT_PWM_XY_PIN);
      digipot_current(0, motor_current_setting[0]);
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
      SET_OUTPUT(MOTOR_CURRENT_PWM_Z_PIN);
      digipot_current(1, motor_current_setting[1]);
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
      SET_OUTPUT(MOTOR_CURRENT_PWM_E_PIN);
      digipot_current(2, motor_current_setting[2]);
    #endif
    //Set timer5 to 31khz so the PWM of the motor power is as constant as possible. (removes a buzzing noise)
    TCCR5B = (TCCR5B & ~(_BV(CS50) | _BV(CS51) | _BV(CS52))) | _BV(CS50);
  #endif
}

void Stepper::digipot_current(uint8_t driver, int16_t current) {
  #if HAS_DIGIPOTSS
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
  #elif HAS_MOTOR_CURRENT_PWM
    #define _WRITE_CURRENT_PWM(P) analogWrite(P, 255L * current / (MOTOR_CURRENT_PWM_RANGE))
    switch (driver) {
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
        case 0: _WRITE_CURRENT_PWM(MOTOR_CURRENT_PWM_XY_PIN); break;
      #endif
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
        case 1: _WRITE_CURRENT_PWM(MOTOR_CURRENT_PWM_Z_PIN); break;
      #endif
      #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
        case 2: _WRITE_CURRENT_PWM(MOTOR_CURRENT_PWM_E_PIN); break;
      #endif
    }
  #else
    UNUSED(driver);
    UNUSED(current);
  #endif
}

void Stepper::microstep_init() {
  #if HAS_MICROSTEPS_E1
    SET_OUTPUT(E1_MS1_PIN);
    SET_OUTPUT(E1_MS2_PIN);
  #endif

  #if HAS_MICROSTEPS
    SET_OUTPUT(X_MS1_PIN);
    SET_OUTPUT(X_MS2_PIN);
    SET_OUTPUT(Y_MS1_PIN);
    SET_OUTPUT(Y_MS2_PIN);
    SET_OUTPUT(Z_MS1_PIN);
    SET_OUTPUT(Z_MS2_PIN);
    const uint8_t microstep_modes[] = MICROSTEP_MODES;
    for (uint16_t i = 0; i < COUNT(microstep_modes); i++)
      microstep_mode(i, microstep_modes[i]);
  #endif
}

/**
 * Software-controlled Microstepping
 */
void Stepper::microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2) {
  if (ms1 >= 0) switch (driver) {
    case 0: WRITE(X_MS1_PIN, ms1); break;
    case 1: WRITE(Y_MS1_PIN, ms1); break;
    case 2: WRITE(Z_MS1_PIN, ms1); break;
  }
  if (ms2 >= 0) switch (driver) {
    case 0: WRITE(X_MS2_PIN, ms2); break;
    case 1: WRITE(Y_MS2_PIN, ms2); break;
    case 2: WRITE(Z_MS2_PIN, ms2); break;
  }
}

void Stepper::microstep_mode(uint8_t driver, uint8_t stepping_mode) {
  switch (stepping_mode) {
    case 1: microstep_ms(driver, MICROSTEP1); break;
    case 2: microstep_ms(driver, MICROSTEP2); break;
    case 4: microstep_ms(driver, MICROSTEP4); break;
    case 8: microstep_ms(driver, MICROSTEP8); break;
    case 16: microstep_ms(driver, MICROSTEP16); break;
  }
}

void Stepper::microstep_readings() {
  SERIAL_PROTOCOLLNPGM("MS1,MS2 Pins");
  SERIAL_PROTOCOLPGM("X: ");
  SERIAL_PROTOCOL(digitalRead(X_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(X_MS2_PIN));
  SERIAL_PROTOCOLPGM("Y: ");
  SERIAL_PROTOCOL(digitalRead(Y_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(Y_MS2_PIN));
  SERIAL_PROTOCOLPGM("Z: ");
  SERIAL_PROTOCOL(digitalRead(Z_MS1_PIN));
  SERIAL_PROTOCOLLN(digitalRead(Z_MS2_PIN));
}
