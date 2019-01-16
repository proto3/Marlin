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
 * stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
 * Derived from Grbl
 *
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

#ifndef STEPPER_H
#define STEPPER_H

#include "Marlin.h"
#include "planner.h"
#include "stepper_indirection.h"
#include "language.h"
#include "types.h"
#include "HAL_timers_Teensy.h"

class Stepper;
extern Stepper stepper;

#define MultiU16X8toH16(intRes, charIn1, intIn2) intRes = ((uint32_t)charIn1 * intIn2) >> 16

class Stepper {

  public:

    static block_t* current_block;  // A pointer to the block currently being traced

    #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
      static bool abort_on_endstop_hit;
    #endif

    #if ENABLED(Z_DUAL_ENDSTOPS)
      static bool performing_homing;
    #endif

  private:

    static uint8_t last_direction_bits;        // The next stepping-bits to be output
    static uint16_t cleaning_buffer_counter;

    #if ENABLED(Z_DUAL_ENDSTOPS)
      static bool locked_z_motor, locked_z2_motor;
    #endif

    // Counter variables for the Bresenham line tracer
    static int32_t counter_X, counter_Y, counter_Z;
    static volatile uint32_t step_events_completed; // The number of step events executed in the current block

    static int32_t acceleration_time, deceleration_time;
    //uint32_t accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
    static uint16_t acc_step_rate; // needed for deceleration start point
    static uint8_t step_loops, step_loops_nominal;
    static uint16_t timer_nominal;

    static volatile int32_t endstops_trigsteps[3];
    static volatile int32_t endstops_stepsTotal, endstops_stepsDone;

    #if HAS_MOTOR_CURRENT_PWM
      #ifndef PWM_MOTOR_CURRENT
        #define PWM_MOTOR_CURRENT DEFAULT_PWM_MOTOR_CURRENT
      #endif
      static constexpr int16_t motor_current_setting[3] = PWM_MOTOR_CURRENT;
    #endif

    //
    // Positions of stepper motors, in step units
    //
    static volatile int32_t count_position[NUM_AXIS];

    //
    // Current direction of stepper motors (+1 or -1)
    //
    static volatile int8_t count_direction[NUM_AXIS];

    //
    // When an axis is locked, stepper won't touch it's pins
    //
    static bool axis_locked[NUM_AXIS];

  public:

    //
    // Constructor / initializer
    //
    Stepper() { };

    //
    // Initialize stepper hardware
    //
    static void init();

    //
    // Interrupt Service Routines
    //
    static void isr();

    //
    // Block until all buffered steps are executed
    //
    static void synchronize();

    //
    // Leave axis pins untouched for external controller to use it
    //
    static void leave_control_on(AxisEnum axis);

    //
    // Take control back on axis pins
    //
    static void take_control_on(AxisEnum axis);

    //
    // Set the current position in steps
    //
    static void set_position(const int32_t& x, const int32_t& y, const int32_t& z);

    //
    // Set direction bits for all steppers
    //
    static void set_directions();

    //
    // Get the position of a stepper, in steps
    //
    static int32_t position(AxisEnum axis);

    //
    // Report the positions of the steppers, in steps
    //
    static void report_positions();

    //
    // Shift known Z position without moving stepper
    //
    static void shift_z_position(int8_t shift);

    //
    // Get the position (mm) of an axis based on stepper position(s)
    //
    static float get_axis_position_mm(AxisEnum axis);

    //
    // The stepper subsystem goes to sleep when it runs out of things to execute. Call this
    // to notify the subsystem that it is time to go to work.
    //
    static void wake_up();

    //
    // Wait for moves to finish and disable all steppers
    //
    static void finish_and_disable();

    //
    // Quickly stop all steppers and clear the blocks queue
    //
    static void quick_stop();

    //
    // The direction of a single motor
    //
    static FORCE_INLINE bool motor_direction(AxisEnum axis) { return TEST(last_direction_bits, axis); }

    #if HAS_DIGIPOTSS
      static void digitalPotWrite(int16_t address, int16_t value);
    #endif
    static void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2);
    static void digipot_current(uint8_t driver, int16_t current);
    static void microstep_mode(uint8_t driver, uint8_t stepping);
    static void microstep_readings();

    #if ENABLED(Z_DUAL_ENDSTOPS)
      static FORCE_INLINE void set_homing_flag(bool state) { performing_homing = state; }
      static FORCE_INLINE void set_z_lock(bool state) { locked_z_motor = state; }
      static FORCE_INLINE void set_z2_lock(bool state) { locked_z2_motor = state; }
    #endif

    static inline void kill_current_block() {
      step_events_completed = current_block->step_event_count;
    }

    //
    // Handle a triggered endstop
    //
    static void endstop_triggered(AxisEnum axis);

    //
    // Triggered position of an axis in mm (not core-savvy)
    //
    static FORCE_INLINE float triggered_position_mm(AxisEnum axis) {
      return endstops_trigsteps[axis] * planner.steps_to_mm[axis];
    }


  private:

    static FORCE_INLINE uint16_t calc_timer(uint16_t step_rate) {
      uint16_t timer;

      NOMORE(step_rate, MAX_STEP_FREQUENCY);

      // if (step_rate > 20000) { // If steprate > 20kHz >> step 4 times
      //   step_rate >>= 2;
      //   step_loops = 4;
      // }
      // else if (step_rate > 10000) { // If steprate > 10kHz >> step 2 times
      //   step_rate >>= 1;
      //   step_loops = 2;
      // }
      // else {
        step_loops = 1;
      // }

      // if(step_rate == 10000)
      //   return 187;


      step_rate *= 10000.0/9855;

      NOLESS(step_rate, 60);
      step_rate -= 60; // Correct for minimal speed
      timer = round((float)STEPPER_TIMER_RATE / step_rate);
      // if (step_rate >= (8 * 256)) { // higher step rate
      //   uint32_t table_address = (uint32_t)&speed_lookuptable_fast[(uint8_t)(step_rate >> 8)][0];
      //   uint8_t tmp_step_rate = (step_rate & 0x00ff);
      //   uint16_t gain = (uint16_t)pgm_read_word_near(table_address + 2);
      //   MultiU16X8toH16(timer, tmp_step_rate, gain);
      //   timer = (uint16_t)pgm_read_word_near(table_address) - timer;
      // }
      // else { // lower step rates
      //   uint32_t table_address = (uint32_t)&speed_lookuptable_slow[0][0];
      //   table_address += ((step_rate) >> 1) & 0xfffc;
      //   timer = (uint16_t)pgm_read_word_near(table_address);
      //   timer -= (((uint16_t)pgm_read_word_near(table_address + 2) * (uint8_t)(step_rate & 0x0007)) >> 3);
      // }
      if (timer < 100) { // (20kHz - this should never happen)
        timer = 100;
        MYSERIAL.print(MSG_STEPPER_TOO_HIGH);
        MYSERIAL.println(step_rate);
      }

      return timer;
    }

    // Initializes the trapezoid generator from the current block. Called whenever a new
    // block begins.
    static FORCE_INLINE void trapezoid_generator_reset() {

      if (current_block->direction_bits != last_direction_bits) {
        last_direction_bits = current_block->direction_bits;
        set_directions();
      }

      deceleration_time = 0;
      // step_rate to timer interval
      timer_nominal = calc_timer(current_block->nominal_rate);
      // make a note of the number of step loops required at nominal speed
      step_loops_nominal = step_loops;
      acc_step_rate = current_block->initial_rate;
      acceleration_time = calc_timer(acc_step_rate);
      HAL_timer_set_compare(STEP_TIMER_NUM, acceleration_time);

      // SERIAL_ECHO_START;
      // SERIAL_ECHOPGM("advance :");
      // SERIAL_ECHO(current_block->advance/256.0);
      // SERIAL_ECHOPGM("advance rate :");
      // SERIAL_ECHO(current_block->advance_rate/256.0);
      // SERIAL_ECHOPGM("initial advance :");
      // SERIAL_ECHO(current_block->initial_advance/256.0);
      // SERIAL_ECHOPGM("final advance :");
      // SERIAL_ECHOLN(current_block->final_advance/256.0);
    }

    static void digipot_init();
    static void microstep_init();
};

#endif // STEPPER_H
