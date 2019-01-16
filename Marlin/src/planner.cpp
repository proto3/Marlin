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
 * planner.cpp
 *
 * Buffer movement commands and manage the acceleration profile plan
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis.
 *
 *
 * Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 *
 * s == speed, a == acceleration, t == time, d == distance
 *
 * Basic definitions:
 *   Speed[s_, a_, t_] := s + (a*t)
 *   Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 *
 * Distance to reach a specific speed with a constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 *   d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 *
 * Speed after a given distance of travel with constant acceleration:
 *   Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 *   m -> Sqrt[2 a d + s^2]
 *
 * DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 *
 * When to start braking (di) to reach a specified destination speed (s2) after accelerating
 * from initial speed s1 without ever stopping at a plateau:
 *   Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 *   di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 *
 * IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 *
 */

#include "planner.h"
#include "stepper.h"
#include "ultralcd.h"
#include "language.h"
#include "torch_height_control.h"

#include "Marlin.h"


Planner planner;

  // public:

/**
 * A ring buffer of moves described in steps
 */
block_t Planner::block_buffer[BLOCK_BUFFER_SIZE];
volatile uint8_t Planner::block_buffer_head = 0;           // Index of the next block to be pushed
volatile uint8_t Planner::block_buffer_tail = 0;

float Planner::max_feedrate_mm_s[NUM_AXIS], // Max speeds in mm per second
      Planner::axis_steps_per_mm[NUM_AXIS],
      Planner::steps_to_mm[NUM_AXIS];

unsigned long Planner::max_acceleration_steps_per_s2[NUM_AXIS],
              Planner::max_acceleration_mm_per_s2[NUM_AXIS]; // Use M201 to override by software

millis_t Planner::min_segment_time;
float Planner::min_feedrate_mm_s,
      Planner::acceleration,         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
      Planner::max_xy_jerk,          // The largest speed change requiring no acceleration
      Planner::max_z_jerk;

// private:

long Planner::position[NUM_AXIS] = { 0 };

float Planner::previous_speed[NUM_AXIS],
      Planner::previous_nominal_speed;

#ifdef XY_FREQUENCY_LIMIT
  // Old direction bits. Used for speed calculations
  unsigned char Planner::old_direction_bits = 0;
  // Segment times (in µs). Used for speed calculations
  long Planner::axis_segment_time[2][3] = { {MAX_FREQ_TIME + 1, 0, 0}, {MAX_FREQ_TIME + 1, 0, 0} };
#endif

/**
 * Class and Instance Methods
 */

Planner::Planner() { init(); }

void Planner::init() {
  block_buffer_head = block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position
  LOOP_XYZ(i) previous_speed[i] = 0.0;
  previous_nominal_speed = 0.0;
}

/**
 * Calculate trapezoid parameters, multiplying the entry- and exit-speeds
 * by the provided factors.
 */
void Planner::calculate_trapezoid_for_block(block_t* block, float entry_factor, float exit_factor) {
  unsigned long initial_rate = ceil(block->nominal_rate * entry_factor),
                final_rate = ceil(block->nominal_rate * exit_factor); // (steps per second)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  NOLESS(initial_rate, 120);
  NOLESS(final_rate, 120);

  long accel = block->acceleration_steps_per_s2;
  int32_t accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, accel));
  int32_t decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -accel));

  // Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort accel and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, accel, block->step_event_count));
    accelerate_steps = max(accelerate_steps, 0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }


  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;
  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if (!block->busy) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps + plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
  }
  CRITICAL_SECTION_END;
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal
// velocities of the respective blocks.
//inline float junction_jerk(block_t *before, block_t *after) {
//  return sqrt(
//    pow((before->speed_x-after->speed_x), 2)+pow((before->speed_y-after->speed_y), 2));
//}


// The kernel called by recalculate() when scanning the plan from last to first entry.
void Planner::reverse_pass_kernel(block_t* previous, block_t* current, block_t* next) {
  if (!current) return;
  UNUSED(previous);

  if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    float max_entry_speed = current->max_entry_speed;
    if (current->entry_speed != max_entry_speed) {

      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if (!current->nominal_length_flag && max_entry_speed > next->entry_speed) {
        current->entry_speed = min(max_entry_speed,
                                   max_allowable_speed(-current->acceleration, next->entry_speed, current->millimeters));
      }
      else {
        current->entry_speed = max_entry_speed;
      }
      current->recalculate_flag = true;

    }
  } // Skip last block. Already initialized and set for recalculation.
}

/**
 * recalculate() needs to go over the current plan twice.
 * Once in reverse and once forward. This implements the reverse pass.
 */
void Planner::reverse_pass() {

  if (movesplanned() > 0) {

    block_t* block[3] = { NULL, NULL, NULL };

    // Make a local copy of block_buffer_tail, because the interrupt can alter it
    CRITICAL_SECTION_START;
      uint8_t tail = block_buffer_tail;
    CRITICAL_SECTION_END

    uint8_t b = BLOCK_MOD(block_buffer_head);
    while (b != tail) {
      b = prev_block_index(b);
      block[2] = block[1];
      block[1] = block[0];
      block[0] = &block_buffer[b];
      reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}

// The kernel called by recalculate() when scanning the plan from first to last entry.
void Planner::forward_pass_kernel(block_t* previous, block_t* current, block_t* next) {
  if (!previous) return;
  UNUSED(next);

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = min(current->entry_speed,
                               max_allowable_speed(-previous->acceleration, previous->entry_speed, previous->millimeters));
      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

/**
 * recalculate() needs to go over the current plan twice.
 * Once in reverse and once forward. This implements the forward pass.
 */
void Planner::forward_pass() {
  block_t* block[3] = { NULL, NULL, NULL };

  for (uint8_t b = block_buffer_tail; b != block_buffer_head; b = next_block_index(b)) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[b];
    forward_pass_kernel(block[0], block[1], block[2]);
  }
  forward_pass_kernel(block[1], block[2], NULL);
}

/**
 * Recalculate the trapezoid speed profiles for all blocks in the plan
 * according to the entry_factor for each junction. Must be called by
 * recalculate() after updating the blocks.
 */
void Planner::recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t* current;
  block_t* next = NULL;

  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        float nom = current->nominal_speed;
        calculate_trapezoid_for_block(current, current->entry_speed / nom, next->entry_speed / nom);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index(block_index);
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if (next) {
    float nom = next->nominal_speed;
    calculate_trapezoid_for_block(next, next->entry_speed / nom, (MINIMUM_PLANNER_SPEED) / nom);
    next->recalculate_flag = false;
  }
}

/*
 * Recalculate the motion plan according to the following algorithm:
 *
 *   1. Go over every block in reverse order...
 *
 *      Calculate a junction speed reduction (block_t.entry_factor) so:
 *
 *      a. The junction jerk is within the set limit, and
 *
 *      b. No speed reduction within one block requires faster
 *         deceleration than the one, true constant acceleration.
 *
 *   2. Go over every block in chronological order...
 *
 *      Dial down junction speed reduction values if:
 *      a. The speed increase within one block would require faster
 *         acceleration than the one, true constant acceleration.
 *
 * After that, all blocks will have an entry_factor allowing all speed changes to
 * be performed using only the one, true constant acceleration, and where no junction
 * jerk is jerkier than the set limit, Jerky. Finally it will:
 *
 *   3. Recalculate "trapezoids" for all blocks.
 */
void Planner::recalculate() {
  reverse_pass();
  forward_pass();
  recalculate_trapezoids();
}



/**
 * Paste extruder pressure,
 */
void Planner::check_axes_activity() {
  unsigned char axis_active[NUM_AXIS] = { 0 };

  if (blocks_queued()) {
    block_t* block;
    for (uint8_t b = block_buffer_tail; b != block_buffer_head; b = next_block_index(b)) {
      block = &block_buffer[b];
      LOOP_XYZ(i) if (block->steps[i]) axis_active[i]++;
    }
  }
  #if ENABLED(DISABLE_X)
    if (!axis_active[X_AXIS]) disable_x();
  #endif
  #if ENABLED(DISABLE_Y)
    if (!axis_active[Y_AXIS]) disable_y();
  #endif
  #if ENABLED(DISABLE_Z)
    if (!axis_active[Z_AXIS]) disable_z();
  #endif
}

/**
 * Planner::buffer_line
 *
 * Add a new linear movement to the buffer.
 *
 *  x,y,z     - target position in mm
 *  fr_mm_s   - (target) speed of the move
 */

  void Planner::buffer_line(const float& x, const float& y, const float& z, float fr_mm_s)
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while (block_buffer_tail == next_buffer_head) idle();

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  long target[NUM_AXIS] = {
    lround(x * axis_steps_per_mm[X_AXIS]),
    lround(y * axis_steps_per_mm[Y_AXIS]),
    lround(z * axis_steps_per_mm[Z_AXIS])
  };

  long dx = target[X_AXIS] - position[X_AXIS],
       dy = target[Y_AXIS] - position[Y_AXIS],
       dz = target[Z_AXIS] - position[Z_AXIS];

  // Prepare to set up new block
  block_t* block = &block_buffer[block_buffer_head];

  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;

  // Number of steps for each axis
  #if ENABLED(COREXY)
    // corexy planning
    // these equations follow the form of the dA and dB equations on http://www.corexy.com/theory.html
    block->steps[A_AXIS] = labs(dx + dy);
    block->steps[B_AXIS] = labs(dx - dy);
    block->steps[Z_AXIS] = labs(dz);
  #elif ENABLED(COREXZ)
    // corexz planning
    block->steps[A_AXIS] = labs(dx + dz);
    block->steps[Y_AXIS] = labs(dy);
    block->steps[C_AXIS] = labs(dx - dz);
  #elif ENABLED(COREYZ)
    // coreyz planning
    block->steps[X_AXIS] = labs(dx);
    block->steps[B_AXIS] = labs(dy + dz);
    block->steps[C_AXIS] = labs(dy - dz);
  #else
    // default non-h-bot planning
    block->steps[X_AXIS] = labs(dx);
    block->steps[Y_AXIS] = labs(dy);
    block->steps[Z_AXIS] = labs(dz);
  #endif

  block->step_event_count = max(block->steps[X_AXIS], max(block->steps[Y_AXIS], block->steps[Z_AXIS]));

  // Bail if this is a zero-length block
  if (block->step_event_count <= dropsegments) return;

  // Compute direction bits for this block
  uint8_t db = 0;
  #if ENABLED(COREXY)
    if (dx < 0) SBI(db, X_HEAD); // Save the real Extruder (head) direction in X Axis
    if (dy < 0) SBI(db, Y_HEAD); // ...and Y
    if (dz < 0) SBI(db, Z_AXIS);
    if (dx + dy < 0) SBI(db, A_AXIS); // Motor A direction
    if (dx - dy < 0) SBI(db, B_AXIS); // Motor B direction
  #elif ENABLED(COREXZ)
    if (dx < 0) SBI(db, X_HEAD); // Save the real Extruder (head) direction in X Axis
    if (dy < 0) SBI(db, Y_AXIS);
    if (dz < 0) SBI(db, Z_HEAD); // ...and Z
    if (dx + dz < 0) SBI(db, A_AXIS); // Motor A direction
    if (dx - dz < 0) SBI(db, C_AXIS); // Motor C direction
  #elif ENABLED(COREYZ)
    if (dx < 0) SBI(db, X_AXIS);
    if (dy < 0) SBI(db, Y_HEAD); // Save the real Extruder (head) direction in Y Axis
    if (dz < 0) SBI(db, Z_HEAD); // ...and Z
    if (dy + dz < 0) SBI(db, B_AXIS); // Motor B direction
    if (dy - dz < 0) SBI(db, C_AXIS); // Motor C direction
  #else
    if (dx < 0) SBI(db, X_AXIS);
    if (dy < 0) SBI(db, Y_AXIS);
    if (dz < 0) SBI(db, Z_AXIS);
  #endif
  block->direction_bits = db;

  //enable active axes
  #if ENABLED(COREXY)
    if (block->steps[A_AXIS] || block->steps[B_AXIS]) {
      enable_x();
      enable_y();
    }
    #if DISABLED(Z_LATE_ENABLE)
      if (block->steps[Z_AXIS]) enable_z();
    #endif
  #elif ENABLED(COREXZ)
    if (block->steps[A_AXIS] || block->steps[C_AXIS]) {
      enable_x();
      enable_z();
    }
    if (block->steps[Y_AXIS]) enable_y();
  #else
    if (block->steps[X_AXIS]) enable_x();
    if (block->steps[Y_AXIS]) enable_y();
    #if DISABLED(Z_LATE_ENABLE)
      if (block->steps[Z_AXIS]) enable_z();
    #endif
  #endif

  NOLESS(fr_mm_s, min_feedrate_mm_s);

  /**
   * This part of the code calculates the total length of the movement.
   * For cartesian bots, the X_AXIS is the real X movement and same for Y_AXIS.
   * But for corexy bots, that is not true. The "X_AXIS" and "Y_AXIS" motors (that should be named to A_AXIS
   * and B_AXIS) cannot be used for X and Y length, because A=X+Y and B=X-Y.
   * So we need to create other 2 "AXIS", named X_HEAD and Y_HEAD, meaning the real displacement of the Head.
   * Having the real displacement of the head, we can calculate the total movement length and apply the desired speed.
   */
  #if ENABLED(COREXY) || ENABLED(COREXZ) || ENABLED(COREYZ)
    float delta_mm[7];
    #if ENABLED(COREXY)
      delta_mm[X_HEAD] = dx * steps_to_mm[A_AXIS];
      delta_mm[Y_HEAD] = dy * steps_to_mm[B_AXIS];
      delta_mm[Z_AXIS] = dz * steps_to_mm[Z_AXIS];
      delta_mm[A_AXIS] = (dx + dy) * steps_to_mm[A_AXIS];
      delta_mm[B_AXIS] = (dx - dy) * steps_to_mm[B_AXIS];
    #elif ENABLED(COREXZ)
      delta_mm[X_HEAD] = dx * steps_to_mm[A_AXIS];
      delta_mm[Y_AXIS] = dy * steps_to_mm[Y_AXIS];
      delta_mm[Z_HEAD] = dz * steps_to_mm[C_AXIS];
      delta_mm[A_AXIS] = (dx + dz) * steps_to_mm[A_AXIS];
      delta_mm[C_AXIS] = (dx - dz) * steps_to_mm[C_AXIS];
    #elif ENABLED(COREYZ)
      delta_mm[X_AXIS] = dx * steps_to_mm[X_AXIS];
      delta_mm[Y_HEAD] = dy * steps_to_mm[B_AXIS];
      delta_mm[Z_HEAD] = dz * steps_to_mm[C_AXIS];
      delta_mm[B_AXIS] = (dy + dz) * steps_to_mm[B_AXIS];
      delta_mm[C_AXIS] = (dy - dz) * steps_to_mm[C_AXIS];
    #endif
  #else
    float delta_mm[4];
    delta_mm[X_AXIS] = dx * steps_to_mm[X_AXIS];
    delta_mm[Y_AXIS] = dy * steps_to_mm[Y_AXIS];
    delta_mm[Z_AXIS] = dz * steps_to_mm[Z_AXIS];
  #endif

  block->millimeters = sqrt(
    #if ENABLED(COREXY)
      sq(delta_mm[X_HEAD]) + sq(delta_mm[Y_HEAD]) + sq(delta_mm[Z_AXIS])
    #elif ENABLED(COREXZ)
      sq(delta_mm[X_HEAD]) + sq(delta_mm[Y_AXIS]) + sq(delta_mm[Z_HEAD])
    #elif ENABLED(COREYZ)
      sq(delta_mm[X_AXIS]) + sq(delta_mm[Y_HEAD]) + sq(delta_mm[Z_HEAD])
    #else
      sq(delta_mm[X_AXIS]) + sq(delta_mm[Y_AXIS]) + sq(delta_mm[Z_AXIS])
    #endif
  );

  float inverse_millimeters = 1.0 / block->millimeters;  // Inverse millimeters to remove multiple divides

  // Calculate moves/second for this move. No divide by zero due to previous checks.
  float inverse_mm_s = fr_mm_s * inverse_millimeters;

  int moves_queued = movesplanned();

  // Slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
  #if ENABLED(OLD_SLOWDOWN) || ENABLED(SLOWDOWN)
    bool mq = moves_queued > 1 && moves_queued < (BLOCK_BUFFER_SIZE) / 2;
    #if ENABLED(OLD_SLOWDOWN)
      if (mq) fr_mm_s *= 2.0 * moves_queued / (BLOCK_BUFFER_SIZE);
    #endif
    #if ENABLED(SLOWDOWN)
      //  segment time im micro seconds
      unsigned long segment_time = lround(1000000.0/inverse_mm_s);
      if (mq) {
        if (segment_time < min_segment_time) {
          // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
          inverse_mm_s = 1000000.0 / (segment_time + lround(2 * (min_segment_time - segment_time) / moves_queued));
          #ifdef XY_FREQUENCY_LIMIT
            segment_time = lround(1000000.0 / inverse_mm_s);
          #endif
        }
      }
    #endif
  #endif

  block->nominal_speed = block->millimeters * inverse_mm_s; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_mm_s); // (step/sec) Always > 0

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[NUM_AXIS];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  LOOP_XYZ(i) {
    current_speed[i] = delta_mm[i] * inverse_mm_s;
    float cs = fabs(current_speed[i]), mf = max_feedrate_mm_s[i];
    if (cs > mf) speed_factor = min(speed_factor, mf / cs);
  }

  // Max segement time in us.
  #ifdef XY_FREQUENCY_LIMIT

    // Check and limit the xy direction change frequency
    unsigned char direction_change = block->direction_bits ^ old_direction_bits;
    old_direction_bits = block->direction_bits;
    segment_time = lround((float)segment_time / speed_factor);

    long xs0 = axis_segment_time[X_AXIS][0],
         xs1 = axis_segment_time[X_AXIS][1],
         xs2 = axis_segment_time[X_AXIS][2],
         ys0 = axis_segment_time[Y_AXIS][0],
         ys1 = axis_segment_time[Y_AXIS][1],
         ys2 = axis_segment_time[Y_AXIS][2];

    if (TEST(direction_change, X_AXIS)) {
      xs2 = axis_segment_time[X_AXIS][2] = xs1;
      xs1 = axis_segment_time[X_AXIS][1] = xs0;
      xs0 = 0;
    }
    xs0 = axis_segment_time[X_AXIS][0] = xs0 + segment_time;

    if (TEST(direction_change, Y_AXIS)) {
      ys2 = axis_segment_time[Y_AXIS][2] = axis_segment_time[Y_AXIS][1];
      ys1 = axis_segment_time[Y_AXIS][1] = axis_segment_time[Y_AXIS][0];
      ys0 = 0;
    }
    ys0 = axis_segment_time[Y_AXIS][0] = ys0 + segment_time;

    long max_x_segment_time = max(xs0, max(xs1, xs2)),
         max_y_segment_time = max(ys0, max(ys1, ys2)),
         min_xy_segment_time = min(max_x_segment_time, max_y_segment_time);
    if (min_xy_segment_time < MAX_FREQ_TIME) {
      float low_sf = speed_factor * min_xy_segment_time / (MAX_FREQ_TIME);
      speed_factor = min(speed_factor, low_sf);
    }
  #endif // XY_FREQUENCY_LIMIT

  // Correct the speed
  if (speed_factor < 1.0) {
    LOOP_XYZ(i) current_speed[i] *= speed_factor;
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  float steps_per_mm = block->step_event_count / block->millimeters;

  // Limit acceleration per axis
  block->acceleration_steps_per_s2 = ceil(acceleration * steps_per_mm);
  if (max_acceleration_steps_per_s2[X_AXIS] < (block->acceleration_steps_per_s2 * block->steps[X_AXIS]) / block->step_event_count)
    block->acceleration_steps_per_s2 = (max_acceleration_steps_per_s2[X_AXIS] * block->step_event_count) / block->steps[X_AXIS];
  if (max_acceleration_steps_per_s2[Y_AXIS] < (block->acceleration_steps_per_s2 * block->steps[Y_AXIS]) / block->step_event_count)
    block->acceleration_steps_per_s2 = (max_acceleration_steps_per_s2[Y_AXIS] * block->step_event_count) / block->steps[Y_AXIS];
  if (max_acceleration_steps_per_s2[Z_AXIS] < (block->acceleration_steps_per_s2 * block->steps[Z_AXIS]) / block->step_event_count)
    block->acceleration_steps_per_s2 = (max_acceleration_steps_per_s2[Z_AXIS] * block->step_event_count) / block->steps[Z_AXIS];

  block->acceleration = block->acceleration_steps_per_s2 / steps_per_mm;
  block->acceleration_rate = (long)(block->acceleration_steps_per_s2 * 16777216.0 / ((F_CPU) * 0.125));

  // Start with a safe speed
  float vmax_junction = max_xy_jerk * 0.5,
        vmax_junction_factor = 1.0,
        mz2 = max_z_jerk * 0.5,
        csz = current_speed[Z_AXIS];
  if (fabs(csz) > mz2) vmax_junction = min(vmax_junction, mz2);
  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float dsx = current_speed[X_AXIS] - previous_speed[X_AXIS],
          dsy = current_speed[Y_AXIS] - previous_speed[Y_AXIS],
          dsz = fabs(csz - previous_speed[Z_AXIS]),
          jerk = HYPOT(dsx, dsy);

    //    if ((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) vmax_junction_factor = max_xy_jerk / jerk;
    if (dsz > max_z_jerk) vmax_junction_factor = min(vmax_junction_factor, max_z_jerk / dsz);

    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
  }
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  block->nominal_length_flag = (block->nominal_speed <= v_allowable);
  block->recalculate_flag = true; // Always calculate trapezoid for new block

  // Update previous path unit_vector and nominal speed
  LOOP_XYZ(i) previous_speed[i] = current_speed[i];
  previous_nominal_speed = block->nominal_speed;

  calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed, safe_speed / block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update position
  LOOP_XYZ(i) position[i] = target[i];

  recalculate();

  stepper.wake_up();

} // buffer_line()


/**
 * Directly set the planner XYZ position (hence the stepper positions).
 *
 * On CORE machines stepper ABC will be translated from the given XYZ.
 */
  void Planner::set_position_mm(const float& x, const float& y, const float& z)
  {

    long nx = position[X_AXIS] = lround(x * axis_steps_per_mm[X_AXIS]),
         ny = position[Y_AXIS] = lround(y * axis_steps_per_mm[Y_AXIS]),
         nz = position[Z_AXIS] = lround(z * axis_steps_per_mm[Z_AXIS]);
    stepper.set_position(nx, ny, nz);
    previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.

    LOOP_XYZ(i) previous_speed[i] = 0.0;
  }

/**
 * Directly set the planner Z position.
 */
void Planner::set_z_position_step(const long& z) {
  position[Z_AXIS] = z;
  previous_speed[Z_AXIS] = 0.0;
}

// Recalculate the steps/s^2 acceleration rates, based on the mm/s^2
void Planner::reset_acceleration_rates() {
  LOOP_XYZ(i)
    max_acceleration_steps_per_s2[i] = max_acceleration_mm_per_s2[i] * axis_steps_per_mm[i];

  torchHeightController.set_max_acc_step_s2(max_acceleration_steps_per_s2[Z_AXIS]);
}

// Recalculate position, steps_to_mm if axis_steps_per_mm changes!
void Planner::refresh_positioning() {
  LOOP_XYZ(i) steps_to_mm[i] = 1.0 / axis_steps_per_mm[i];
    set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
  reset_acceleration_rates();
}
