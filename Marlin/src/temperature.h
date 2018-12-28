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
 * temperature.h - temperature controller
 */

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include "planner.h"

class Temperature {

  #if ENABLED(BABYSTEPPING)
  public:
    static volatile int babystepsTodo[3];
  #endif


  #if ENABLED(FILAMENT_WIDTH_SENSOR)
  private:
    static int meas_shift_index;  // Index of a delayed sample in buffer
    static int current_raw_filwidth;  //Holds measured filament diameter - one extruder only
  #endif

  public:

    /**
     * Instance Methods
     */
    void init();

    /**
     * Called from the Temperature ISR
     */
    static void isr();

    /**
     * Call periodically to manage heaters
     */
    static void manage_heater();

    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      static float analog2widthFil(); // Convert raw Filament Width to millimeters
      static int widthFil_to_size_ratio(); // Convert raw Filament Width to an extrusion ratio
    #endif

    #if ENABLED(BABYSTEPPING)

      static void babystep_axis(AxisEnum axis, int distance) {
        #if ENABLED(COREXY) || ENABLED(COREXZ) || ENABLED(COREYZ)
          #if ENABLED(BABYSTEP_XY)
            switch (axis) {
              case CORE_AXIS_1: // X on CoreXY and CoreXZ, Y on CoreYZ
                babystepsTodo[CORE_AXIS_1] += distance * 2;
                babystepsTodo[CORE_AXIS_2] += distance * 2;
                break;
              case CORE_AXIS_2: // Y on CoreXY, Z on CoreXZ and CoreYZ
                babystepsTodo[CORE_AXIS_1] += distance * 2;
                babystepsTodo[CORE_AXIS_2] -= distance * 2;
                break;
              case NORMAL_AXIS: // Z on CoreXY, Y on CoreXZ, X on CoreYZ
                babystepsTodo[NORMAL_AXIS] += distance;
                break;
            }
          #elif ENABLED(COREXZ) || ENABLED(COREYZ)
            // Only Z stepping needs to be handled here
            babystepsTodo[CORE_AXIS_1] += distance * 2;
            babystepsTodo[CORE_AXIS_2] -= distance * 2;
          #else
            babystepsTodo[Z_AXIS] += distance;
          #endif
        #else
          babystepsTodo[axis] += distance;
        #endif
      }

    #endif // BABYSTEPPING

  private:
    static void updateTemperaturesFromRawValues();
};

extern Temperature thermalManager;

#endif // TEMPERATURE_H
