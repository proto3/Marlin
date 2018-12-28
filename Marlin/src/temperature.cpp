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
 * temperature.cpp - temperature control
 */

#include "ultralcd.h"
#include "temperature.h"
#include "plasma.h"
#include "torch_height_control.h"

#if ENABLED(BABYSTEPPING)
  #include "stepper.h"
#endif

#if ENABLED(USE_WATCHDOG)
  #include "watchdog.h"
#endif

Temperature thermalManager;

// public:

#if ENABLED(BABYSTEPPING)
  volatile int Temperature::babystepsTodo[3] = { 0 };
#endif

// private:

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  int Temperature::meas_shift_index;  // Index of a delayed sample in buffer
  int Temperature::current_raw_filwidth = 0;  //Holds measured filament diameter - one extruder only
#endif

/**
 * Manage heating activities for extruder hot-ends and a heated bed
 *  - Acquire updated temperature readings
 *    - Also resets the watchdog timer
 *  - Invoke thermal runaway protection
 *  - Apply filament width to the extrusion rate (may move)
 *  - Update the heated bed PID output value
 */
void Temperature::manage_heater() {
  updateTemperaturesFromRawValues(); // also resets the watchdog

  // Control the extruder rate based on the width sensor
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    if (filament_sensor) {
      meas_shift_index = filwidth_delay_index1 - meas_delay_cm;
      if (meas_shift_index < 0) meas_shift_index += MAX_MEASUREMENT_DELAY + 1;  //loop around buffer if needed

      // Get the delayed info and add 100 to reconstitute to a percent of
      // the nominal filament diameter then square it to get an area
      meas_shift_index = constrain(meas_shift_index, 0, MAX_MEASUREMENT_DELAY);
      float vm = pow((measurement_delay[meas_shift_index] + 100.0) * 0.01, 2);
      NOLESS(vm, 0.01);
      volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM] = vm;
    }
  #endif //FILAMENT_WIDTH_SENSOR
}

/**
 * Get the raw values into the actual temperatures.
 * The raw values are created in interrupt context,
 * and this function is called from normal context
 * as it would block the stepper routine.
 */
void Temperature::updateTemperaturesFromRawValues() {
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    filament_width_meas = analog2widthFil();
  #endif

  #if ENABLED(USE_WATCHDOG)
    // Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();
  #endif
}


#if ENABLED(FILAMENT_WIDTH_SENSOR)

  // Convert raw Filament Width to millimeters
  float Temperature::analog2widthFil() {
    return current_raw_filwidth / 16383.0 * 5.0;
    //return current_raw_filwidth;
  }

  // Convert raw Filament Width to a ratio
  int Temperature::widthFil_to_size_ratio() {
    float temp = filament_width_meas;
    if (temp < MEASURED_LOWER_LIMIT) temp = filament_width_nominal;  //assume sensor cut out
    else NOMORE(temp, MEASURED_UPPER_LIMIT);
    return filament_width_nominal / temp * 100;
  }
#endif

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_heater()
 */
void Temperature::init() {

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    ANALOG_SELECT(FILWIDTH_PIN);
  #endif

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  SBI(TIMSK0, OCIE0B);

  // Wait for temperature measurement to settle
  delay(250);
}

/**
 * Timer 0 is shared with millies
 *  - Update the raw temperature values
 *  - Check new temperature values for MIN/MAX errors
 *  - Step the babysteps value for each axis towards 0
 */
ISR(TIMER0_COMPB_vect) { Temperature::isr(); }

void Temperature::isr() {
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    static unsigned long raw_filwidth_value = 0;
  #endif

  PlasmaState plasma_state = plasmaManager.update();
  torchHeightController.update(plasma_state);

  lcd_buttons_update();

  #if ENABLED(BABYSTEPPING)
    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; axis++) {
      int curTodo = babystepsTodo[axis]; //get rid of volatile for performance

      if (curTodo > 0) {
        stepper.babystep(axis,/*fwd*/true);
        babystepsTodo[axis]--; //fewer to do next time
      }
      else if (curTodo < 0) {
        stepper.babystep(axis,/*fwd*/false);
        babystepsTodo[axis]++; //fewer to do next time
      }
    }
  #endif //BABYSTEPPING
}
