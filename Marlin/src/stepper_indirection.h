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
  stepper_indirection.h - stepper motor driver indirection macros
  to allow some stepper functions to be done via SPI/I2c instead of direct pin manipulation
  Part of Marlin

  Copyright (c) 2015 Dominik Wenger

  Marlin is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Marlin is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Marlin.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPER_INDIRECTION_H
#define STEPPER_INDIRECTION_H

#include "MarlinConfig.h"

// TMC26X drivers have STEP/DIR on normal pins, but ENABLE via SPI
#if ENABLED(HAVE_TMCDRIVER)
  #include <SPI.h>
  #include <TMC26XStepper.h>
  void tmc_init();
#endif

// L6470 has STEP on normal pins, but DIR/ENABLE via SPI
#if ENABLED(HAVE_L6470DRIVER)
  #include <SPI.h>
  #include <L6470.h>
  void L6470_init();
#endif

// X Stepper
#if ENABLED(HAVE_L6470DRIVER) && ENABLED(X_IS_L6470)
  extern L6470 stepperX;
  #define X_ENABLE_INIT NOOP
  #define X_ENABLE_WRITE(STATE) do{if(STATE) stepperX.Step_Clock(stepperX.getStatus() & STATUS_HIZ); else stepperX.softFree();}while(0)
  #define X_ENABLE_READ (stepperX.getStatus() & STATUS_HIZ)
  #define X_DIR_INIT NOOP
  #define X_DIR_WRITE(STATE) stepperX.Step_Clock(STATE)
  #define X_DIR_READ (stepperX.getStatus() & STATUS_DIR)
#else
  #if ENABLED(HAVE_TMCDRIVER) && ENABLED(X_IS_TMC)
    extern TMC26XStepper stepperX;
    #define X_ENABLE_INIT NOOP
    #define X_ENABLE_WRITE(STATE) stepperX.setEnabled(STATE)
    #define X_ENABLE_READ stepperX.isEnabled()
  #else
    #define X_ENABLE_INIT SET_OUTPUT(X_ENABLE_PIN)
    #define X_ENABLE_WRITE(STATE) WRITE(X_ENABLE_PIN,STATE)
    #define X_ENABLE_READ READ(X_ENABLE_PIN)
  #endif
  #define X_DIR_INIT SET_OUTPUT(X_DIR_PIN)
  #define X_DIR_WRITE(STATE) WRITE(X_DIR_PIN,STATE)
  #define X_DIR_READ READ(X_DIR_PIN)
#endif
#define X_STEP_INIT SET_OUTPUT(X_STEP_PIN)
#define X_STEP_WRITE(STATE) WRITE(X_STEP_PIN,STATE)
#define X_STEP_READ READ(X_STEP_PIN)

// Y Stepper
#if ENABLED(HAVE_L6470DRIVER) && ENABLED(Y_IS_L6470)
  extern L6470 stepperY;
  #define Y_ENABLE_INIT NOOP
  #define Y_ENABLE_WRITE(STATE) do{if(STATE) stepperY.Step_Clock(stepperY.getStatus() & STATUS_HIZ); else stepperY.softFree();}while(0)
  #define Y_ENABLE_READ (stepperY.getStatus() & STATUS_HIZ)
  #define Y_DIR_INIT NOOP
  #define Y_DIR_WRITE(STATE) stepperY.Step_Clock(STATE)
  #define Y_DIR_READ (stepperY.getStatus() & STATUS_DIR)
#else
  #if ENABLED(HAVE_TMCDRIVER) && ENABLED(Y_IS_TMC)
    extern TMC26XStepper stepperY;
    #define Y_ENABLE_INIT NOOP
    #define Y_ENABLE_WRITE(STATE) stepperY.setEnabled(STATE)
    #define Y_ENABLE_READ stepperY.isEnabled()
  #else
    #define Y_ENABLE_INIT SET_OUTPUT(Y_ENABLE_PIN)
    #define Y_ENABLE_WRITE(STATE) WRITE(Y_ENABLE_PIN,STATE)
    #define Y_ENABLE_READ READ(Y_ENABLE_PIN)
  #endif
  #define Y_DIR_INIT SET_OUTPUT(Y_DIR_PIN)
  #define Y_DIR_WRITE(STATE) WRITE(Y_DIR_PIN,STATE)
  #define Y_DIR_READ READ(Y_DIR_PIN)
#endif
#define Y_STEP_INIT SET_OUTPUT(Y_STEP_PIN)
#define Y_STEP_WRITE(STATE) WRITE(Y_STEP_PIN,STATE)
#define Y_STEP_READ READ(Y_STEP_PIN)

// Z Stepper
#if ENABLED(HAVE_L6470DRIVER) && ENABLED(Z_IS_L6470)
  extern L6470 stepperZ;
  #define Z_ENABLE_INIT NOOP
  #define Z_ENABLE_WRITE(STATE) do{if(STATE) stepperZ.Step_Clock(stepperZ.getStatus() & STATUS_HIZ); else stepperZ.softFree();}while(0)
  #define Z_ENABLE_READ (stepperZ.getStatus() & STATUS_HIZ)
  #define Z_DIR_INIT NOOP
  #define Z_DIR_WRITE(STATE) stepperZ.Step_Clock(STATE)
  #define Z_DIR_READ (stepperZ.getStatus() & STATUS_DIR)
#else
  #if ENABLED(HAVE_TMCDRIVER) && ENABLED(Z_IS_TMC)
    extern TMC26XStepper stepperZ;
    #define Z_ENABLE_INIT NOOP
    #define Z_ENABLE_WRITE(STATE) stepperZ.setEnabled(STATE)
    #define Z_ENABLE_READ stepperZ.isEnabled()
  #else
    #define Z_ENABLE_INIT SET_OUTPUT(Z_ENABLE_PIN)
    #define Z_ENABLE_WRITE(STATE) WRITE(Z_ENABLE_PIN,STATE)
    #define Z_ENABLE_READ READ(Z_ENABLE_PIN)
  #endif
  #define Z_DIR_INIT SET_OUTPUT(Z_DIR_PIN)
  #define Z_DIR_WRITE(STATE) WRITE(Z_DIR_PIN,STATE)
  #define Z_DIR_READ READ(Z_DIR_PIN)
#endif
#define Z_STEP_INIT SET_OUTPUT(Z_STEP_PIN)
#define Z_STEP_WRITE(STATE) WRITE(Z_STEP_PIN,STATE)
#define Z_STEP_READ READ(Z_STEP_PIN)

// X2 Stepper
#if HAS_X2_ENABLE
  #if ENABLED(HAVE_L6470DRIVER) && ENABLED(X2_IS_L6470)
    extern L6470 stepperX2;
    #define X2_ENABLE_INIT NOOP
    #define X2_ENABLE_WRITE(STATE) do{if(STATE) stepperX2.Step_Clock(stepperX2.getStatus() & STATUS_HIZ); else stepperX2.softFree();}while(0)
    #define X2_ENABLE_READ (stepperX2.getStatus() & STATUS_HIZ)
    #define X2_DIR_INIT NOOP
    #define X2_DIR_WRITE(STATE) stepperX2.Step_Clock(STATE)
    #define X2_DIR_READ (stepperX2.getStatus() & STATUS_DIR)
  #else
    #if ENABLED(HAVE_TMCDRIVER) && ENABLED(X2_IS_TMC)
      extern TMC26XStepper stepperX2;
      #define X2_ENABLE_INIT NOOP
      #define X2_ENABLE_WRITE(STATE) stepperX2.setEnabled(STATE)
      #define X2_ENABLE_READ stepperX2.isEnabled()
    #else
      #define X2_ENABLE_INIT SET_OUTPUT(X2_ENABLE_PIN)
      #define X2_ENABLE_WRITE(STATE) WRITE(X2_ENABLE_PIN,STATE)
      #define X2_ENABLE_READ READ(X2_ENABLE_PIN)
    #endif
    #define X2_DIR_INIT SET_OUTPUT(X2_DIR_PIN)
    #define X2_DIR_WRITE(STATE) WRITE(X2_DIR_PIN,STATE)
    #define X2_DIR_READ READ(X2_DIR_PIN)
  #endif
  #define X2_STEP_INIT SET_OUTPUT(X2_STEP_PIN)
  #define X2_STEP_WRITE(STATE) WRITE(X2_STEP_PIN,STATE)
  #define X2_STEP_READ READ(X2_STEP_PIN)
#endif

// Y2 Stepper
#if HAS_Y2_ENABLE
  #if ENABLED(HAVE_L6470DRIVER) && ENABLED(Y2_IS_L6470)
    extern L6470 stepperY2;
    #define Y2_ENABLE_INIT NOOP
    #define Y2_ENABLE_WRITE(STATE) do{if(STATE) stepperY2.Step_Clock(stepperY2.getStatus() & STATUS_HIZ); else stepperY2.softFree();}while(0)
    #define Y2_ENABLE_READ (stepperY2.getStatus() & STATUS_HIZ)
    #define Y2_DIR_INIT NOOP
    #define Y2_DIR_WRITE(STATE) stepperY2.Step_Clock(STATE)
    #define Y2_DIR_READ (stepperY2.getStatus() & STATUS_DIR)
  #else
    #if ENABLED(HAVE_TMCDRIVER) && ENABLED(Y2_IS_TMC)
      extern TMC26XStepper stepperY2;
      #define Y2_ENABLE_INIT NOOP
      #define Y2_ENABLE_WRITE(STATE) stepperY2.setEnabled(STATE)
      #define Y2_ENABLE_READ stepperY2.isEnabled()
    #else
      #define Y2_ENABLE_INIT SET_OUTPUT(Y2_ENABLE_PIN)
      #define Y2_ENABLE_WRITE(STATE) WRITE(Y2_ENABLE_PIN,STATE)
      #define Y2_ENABLE_READ READ(Y2_ENABLE_PIN)
    #endif
    #define Y2_DIR_INIT SET_OUTPUT(Y2_DIR_PIN)
    #define Y2_DIR_WRITE(STATE) WRITE(Y2_DIR_PIN,STATE)
    #define Y2_DIR_READ READ(Y2_DIR_PIN)
  #endif
  #define Y2_STEP_INIT SET_OUTPUT(Y2_STEP_PIN)
  #define Y2_STEP_WRITE(STATE) WRITE(Y2_STEP_PIN,STATE)
  #define Y2_STEP_READ READ(Y2_STEP_PIN)
#endif

// Z2 Stepper
#if HAS_Z2_ENABLE
  #if ENABLED(HAVE_L6470DRIVER) && ENABLED(Z2_IS_L6470)
    extern L6470 stepperZ2;
    #define Z2_ENABLE_INIT NOOP
    #define Z2_ENABLE_WRITE(STATE) do{if(STATE) stepperZ2.Step_Clock(stepperZ2.getStatus() & STATUS_HIZ); else stepperZ2.softFree();}while(0)
    #define Z2_ENABLE_READ (stepperZ2.getStatus() & STATUS_HIZ)
    #define Z2_DIR_INIT NOOP
    #define Z2_DIR_WRITE(STATE) stepperZ2.Step_Clock(STATE)
    #define Z2_DIR_READ (stepperZ2.getStatus() & STATUS_DIR)
  #else
    #if ENABLED(HAVE_TMCDRIVER) && ENABLED(Z2_IS_TMC)
      extern TMC26XStepper stepperZ2;
      #define Z2_ENABLE_INIT NOOP
      #define Z2_ENABLE_WRITE(STATE) stepperZ2.setEnabled(STATE)
      #define Z2_ENABLE_READ stepperZ2.isEnabled()
    #else
      #define Z2_ENABLE_INIT SET_OUTPUT(Z2_ENABLE_PIN)
      #define Z2_ENABLE_WRITE(STATE) WRITE(Z2_ENABLE_PIN,STATE)
      #define Z2_ENABLE_READ READ(Z2_ENABLE_PIN)
    #endif
    #define Z2_DIR_INIT SET_OUTPUT(Z2_DIR_PIN)
    #define Z2_DIR_WRITE(STATE) WRITE(Z2_DIR_PIN,STATE)
    #define Z2_DIR_READ READ(Z2_DIR_PIN)
  #endif
  #define Z2_STEP_INIT SET_OUTPUT(Z2_STEP_PIN)
  #define Z2_STEP_WRITE(STATE) WRITE(Z2_STEP_PIN,STATE)
  #define Z2_STEP_READ READ(Z2_STEP_PIN)
#endif

#endif // STEPPER_INDIRECTION_H
