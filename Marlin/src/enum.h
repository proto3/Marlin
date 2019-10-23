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

#ifndef __ENUM_H__
#define __ENUM_H__

/**
 * Axis indices as enumerated constants
 *
 * Special axis:
 *  - A_AXIS and B_AXIS are used by COREXY printers
 *  - X_HEAD and Y_HEAD is used for systems that don't have a 1:1 relationship
 *    between X_AXIS and X Head movement, like CoreXY bots
 */
enum AxisEnum {
  NO_AXIS = -1,
  X_AXIS  = 0,
  A_AXIS  = 0,
  Y_AXIS  = 1,
  B_AXIS  = 1,
  Z_AXIS  = 2,
  C_AXIS  = 2,
  X_HEAD  = 3,
  Y_HEAD  = 4,
  Z_HEAD  = 5
};

#define LOOP_XYZ(VAR)  for (uint8_t VAR=X_AXIS; VAR<=Z_AXIS; VAR++)

typedef enum {
  LINEARUNIT_MM,
  LINEARUNIT_INCH
} LinearUnit;

/**
 * Debug flags
 * Not yet widely applied
 */
enum DebugFlags {
  DEBUG_NONE          = 0,
  DEBUG_ECHO          = _BV(0), ///< Echo commands in order as they are processed
  DEBUG_INFO          = _BV(1), ///< Print messages for code that has debug output
  DEBUG_ERRORS        = _BV(2), ///< Not implemented
  DEBUG_DRYRUN        = _BV(3), ///< Ignore temperature setting and E movement commands
  DEBUG_COMMUNICATION = _BV(4), ///< Not implemented
  DEBUG_LEVELING      = _BV(5)  ///< Print detailed output for homing and leveling
};

enum EndstopEnum {
  X_MIN,
  Y_MIN,
  Z_MIN,
  Z_MIN_PROBE,
  X_MAX,
  Y_MAX,
  Z_MAX,
  Z2_MIN,
  Z2_MAX
};

/**
 * Plasma states
 */
 enum PlasmaState {
   Off,
   Ignition,
   Established,
   Established_THC,
   Slowdown_THC
 };

#if ENABLED(EMERGENCY_PARSER)
  enum e_parser_state {
    state_RESET,
    state_N,
    state_M,
    state_M1,
    state_M10,
    state_M11,
    state_M112,
    state_M4,
    state_M41,
    state_M410,
    state_IGNORE // to '\n'
  };
#endif


/**
 * States for managing Marlin and host communication
 * Marlin sends messages if blocked or busy
 */
#if ENABLED(HOST_KEEPALIVE_FEATURE)
  enum MarlinBusyState {
    NOT_BUSY,           // Not in a handler
    IN_HANDLER,         // Processing a GCode
    IN_PROCESS,         // Known to be blocking command input (as in G29)
    PAUSED_FOR_USER,    // Blocking pending any input
    PAUSED_FOR_INPUT    // Blocking pending text input (concept)
  };
#endif


/**
 * SD Card
 */
enum LsAction { LS_SerialPrint, LS_Count, LS_GetFilename };

/**
 * Machine State
 */
enum MachineState { waiting_file, running, pause_pending, suspended };

/**
 * Ultra LCD
 */
enum LCDViewAction {
  LCDVIEW_NONE,
  LCDVIEW_REDRAW_NOW,
  LCDVIEW_CALL_REDRAW_NEXT,
  LCDVIEW_CLEAR_CALL_REDRAW,
  LCDVIEW_CALL_NO_REDRAW
};

#endif // __ENUM_H__
