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
 * Configuration_adv.h
 *
 * Advanced settings.
 * Only change these if you know exactly what you're doing.
 * Some of these settings can damage your printer if improperly set!
 *
 * Basic settings can be found in Configuration.h
 *
 */
#ifndef CONFIGURATION_ADV_H
#define CONFIGURATION_ADV_H

/**
 *
 *  ***********************************
 *  **  ATTENTION TO ALL DEVELOPERS  **
 *  ***********************************
 *
 * You must increment this version number for every significant change such as,
 * but not limited to: ADD, DELETE RENAME OR REPURPOSE any directive/option.
 *
 * Note: Update also Version.h !
 */
#define CONFIGURATION_ADV_H_VERSION 010100

//===========================================================================
//============================ Mechanical Settings ==========================
//===========================================================================

// @section homing

// If you want endstops to stay on (by default) even when not homing
// enable this option. Override at any time with M120, M121.
//#define ENDSTOPS_ALWAYS_ON_DEFAULT

// @section extras

//#define Z_LATE_ENABLE // Enable Z the last moment. Needed if your Z driver overheats.

// Dual X Steppers
// Uncomment this option to drive two X axis motors.
//#define X_DUAL_STEPPER_DRIVERS
#if ENABLED(X_DUAL_STEPPER_DRIVERS)
  // Set true if the two X motors need to rotate in opposite directions
  #define INVERT_X2_VS_X_DIR true
#endif


// Dual Y Steppers
// Uncomment this option to drive two Y axis motors.
//#define Y_DUAL_STEPPER_DRIVERS
#if ENABLED(Y_DUAL_STEPPER_DRIVERS)
  // Set true if the two Y motors need to rotate in opposite directions
  #define INVERT_Y2_VS_Y_DIR true
#endif

// A single Z stepper driver is usually used to drive 2 stepper motors.
// Uncomment this option to use a separate stepper driver for each Z axis motor.
//#define Z_DUAL_STEPPER_DRIVERS

#if ENABLED(Z_DUAL_STEPPER_DRIVERS)

  // Z_DUAL_ENDSTOPS is a feature to enable the use of 2 endstops for both Z steppers - Let's call them Z stepper and Z2 stepper.
  // That way the machine is capable to align the bed during home, since both Z steppers are homed.
  // There is also an implementation of M666 (software endstops adjustment) to this feature.
  // After Z homing, this adjustment is applied to just one of the steppers in order to align the bed.
  // One just need to home the Z axis and measure the distance difference between both Z axis and apply the math: Z adjust = Z - Z2.
  // If the Z stepper axis is closer to the bed, the measure Z > Z2 (yes, it is.. think about it) and the Z adjust would be positive.
  // Play a little bit with small adjustments (0.5mm) and check the behaviour.
  // The M119 (endstops report) will start reporting the Z2 Endstop as well.

  //#define Z_DUAL_ENDSTOPS

  #if ENABLED(Z_DUAL_ENDSTOPS)
    #define Z2_USE_ENDSTOP _XMAX_
  #endif

#endif // Z_DUAL_STEPPER_DRIVERS

// @section homing

//homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM 5
#define Z_HOME_BUMP_MM 5
#define HOMING_BUMP_DIVISOR {2, 2, 2}  // Re-Bump Speed Divisor (Divides the Homing Feedrate)
//#define QUICK_HOME  //if this is defined, if both x and y are to be homed, a diagonal move will be performed initially.

// When G28 is called, this option will make Y home before X
//#define HOME_Y_BEFORE_X

// @section machine

#define AXIS_RELATIVE_MODES {false, false, false}

// By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false

// Default stepper release if idle. Set to 0 to deactivate.
// Steppers will shut down DEFAULT_STEPPER_DEACTIVE_TIME seconds after the last move when DISABLE_INACTIVE_? is true.
// Time can be set by M18 and M84.
#define DEFAULT_STEPPER_DEACTIVE_TIME 120
#define DISABLE_INACTIVE_X true
#define DISABLE_INACTIVE_Y true
#define DISABLE_INACTIVE_Z true  // set to false if the nozzle will fall down on your printed part when print has finished.

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate

// @section lcd

#if ENABLED(ULTIPANEL)
  #define MANUAL_FEEDRATE {50*60, 50*60, 50*60} // Feedrates for manual moves along X, Y, Z from panel
  #define ULTIPANEL_FEEDMULTIPLY  // Comment to disable setting feedrate multiplier via encoder
#endif

// @section extras

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME        20000

// If defined the movements slow down when the look ahead buffer is only half full
//#define SLOWDOWN

// Frequency limit
// See nophead's blog for more info
// Not working O
//#define XY_FREQUENCY_LIMIT  15

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU.
#define MICROSTEP_MODES {16,16,16,16,16} // [1,2,4,8,16]

// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
#define DIGIPOT_MOTOR_CURRENT {135,135,135,135,135} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)

// Motor Current controlled via PWM (Overridable on supported boards with PWM-driven motor driver current)
//#define PWM_MOTOR_CURRENT {1300, 1300, 1250} // Values in milliamps

// uncomment to enable an I2C based DIGIPOT like on the Azteeg X3 Pro
//#define DIGIPOT_I2C
// Number of channels available for I2C digipot, For Azteeg X3 Pro we have 8
#define DIGIPOT_I2C_NUM_CHANNELS 8
// actual motor currents in Amps, need as many here as DIGIPOT_I2C_NUM_CHANNELS
#define DIGIPOT_I2C_MOTOR_CURRENTS {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

#define ENCODER_RATE_MULTIPLIER         // If defined, certain menu edit operations automatically multiply the steps when the encoder is moved quickly
#define ENCODER_10X_STEPS_PER_SEC 75    // If the encoder steps per sec exceeds this value, multiply steps moved x10 to quickly advance the value
#define ENCODER_100X_STEPS_PER_SEC 160  // If the encoder steps per sec exceeds this value, multiply steps moved x100 to really quickly advance the value

//#define CHDK 4        //Pin for triggering CHDK to take a picture see how to use it here http://captain-slow.dk/2014/03/09/3d-printing-timelapses/
#define CHDK_DELAY 50 //How long in ms the pin should stay HIGH before going LOW again

// @section lcd

// Include a page of printer information in the LCD Main Menu
//#define LCD_INFO_MENU

#if ENABLED(SDSUPPORT)

  // Some RAMPS and other boards don't detect when an SD card is inserted. You can work
  // around this by connecting a push button or single throw switch to the pin defined
  // as SD_DETECT_PIN in your board's pins definitions.
  // This setting should be disabled unless you are using a push button, pulling the pin to ground.
  // Note: This is always disabled for ULTIPANEL (except ELB_FULL_GRAPHIC_CONTROLLER).
  #define SD_DETECT_INVERTED

  #define SD_FINISHED_STEPPERRELEASE true  //if sd support and the file is finished: disable steppers?
  #define SD_FINISHED_RELEASECOMMAND "M84 X Y Z" // You might want to keep the z enabled so your bed stays in place.

  #define SDCARD_RATHERRECENTFIRST  //reverse file order of sd card menu display. Its sorted practically after the file system block order.
  // if a file is deleted, it frees a block. hence, the order is not purely chronological. To still have auto0.g accessible, there is again the option to do that.
  // using:
  //#define MENU_ADDAUTOSTART

  // Show a progress bar on HD44780 LCDs for SD printing
  //#define LCD_PROGRESS_BAR

  #if ENABLED(LCD_PROGRESS_BAR)
    // Amount of time (ms) to show the bar
    #define PROGRESS_BAR_BAR_TIME 2000
    // Amount of time (ms) to show the status message
    #define PROGRESS_BAR_MSG_TIME 3000
    // Amount of time (ms) to retain the status message (0=forever)
    #define PROGRESS_MSG_EXPIRE   0
    // Enable this to show messages for MSG_TIME then hide them
    //#define PROGRESS_MSG_ONCE
  #endif

  // This allows hosts to request long names for files and folders with M33
  //#define LONG_FILENAME_HOST_SUPPORT

  // This option allows you to abort SD printing when any endstop is triggered.
  // This feature must be enabled with "M540 S1" or from the LCD menu.
  // To have any effect, endstops must be enabled during SD printing.
  //#define ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

#endif // SDSUPPORT

// for dogm lcd displays you can choose some additional fonts:
#if ENABLED(DOGLCD)
  // save 3120 bytes of PROGMEM by commenting out #define USE_BIG_EDIT_FONT
  // we don't have a big font for Cyrillic, Kana
  //#define USE_BIG_EDIT_FONT

  // If you have spare 2300Byte of progmem and want to use a
  // smaller font on the Info-screen uncomment the next line.
  //#define USE_SMALL_INFOFONT
#endif // DOGLCD

// @section safety

// The hardware watchdog should reset the microcontroller disabling all outputs,
// in case the firmware gets stuck and doesn't do temperature regulation.
#define USE_WATCHDOG

#if ENABLED(USE_WATCHDOG)
  // If you have a watchdog reboot in an ArduinoMega2560 then the device will hang forever, as a watchdog reset will leave the watchdog on.
  // The "WATCHDOG_RESET_MANUAL" goes around this by not using the hardware reset.
  //  However, THIS FEATURE IS UNSAFE!, as it will only work if interrupts are disabled. And the code could hang in an interrupt routine with interrupts disabled.
  //#define WATCHDOG_RESET_MANUAL
#endif

// @section extras

// Arc interpretation settings:
#define ARC_SUPPORT  // Disabling this saves ~2738 bytes
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

// Support for G5 with XYZ destination and IJPQ offsets. Requires ~2666 bytes.
//#define BEZIER_CURVE_SUPPORT

const unsigned int dropsegments = 5; //everything with less than this number of steps will be ignored as move and joined with the next movement

//===========================================================================
//================================= Buffers =================================
//===========================================================================

// @section hidden

// The number of linear motions that can be in the plan at any give time.
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ring-buffering.
#if ENABLED(SDSUPPORT)
  #define BLOCK_BUFFER_SIZE 32   // SD,LCD,Buttons take more memory, block buffer needs to be smaller
#else
  #define BLOCK_BUFFER_SIZE 16 // maximize block buffer
#endif

// @section serial

// The ASCII buffer for serial input
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

// Transfer Buffer Size
// To save 386 bytes of PROGMEM (and TX_BUFFER_SIZE+3 bytes of RAM) set to 0.
// To buffer a simple "ok" you need 4 bytes.
// For ADVANCED_OK (M105) you need 32 bytes.
// For debug-echo: 128 bytes for the optimal speed.
// Other output doesn't need to be that speedy.
// :[0,2,4,8,16,32,64,128,256]
#define TX_BUFFER_SIZE 0

// Enable an emergency-command parser to intercept certain commands as they
// enter the serial receive buffer, so they cannot be blocked.
// Currently handles M112, M410
// Does not work on boards using AT90USB (USBCON) processors!
//#define EMERGENCY_PARSER

// Bad Serial-connections can miss a received command by sending an 'ok'
// Therefore some clients abort after 30 seconds in a timeout.
// Some other clients start sending commands while receiving a 'wait'.
// This "wait" is only sent when the buffer is empty. 1 second is a good value here.
//#define NO_TIMEOUTS 1000 // Milliseconds

// Some clients will have this feature soon. This could make the NO_TIMEOUTS unnecessary.
//#define ADVANCED_OK

/******************************************************************************\
 * enable this section if you have TMC26X motor drivers.
 * you need to import the TMC26XStepper library into the Arduino IDE for this
 ******************************************************************************/

// @section tmc

//#define HAVE_TMCDRIVER
#if ENABLED(HAVE_TMCDRIVER)

  //#define X_IS_TMC
  #define X_MAX_CURRENT 1000  //in mA
  #define X_SENSE_RESISTOR 91 //in mOhms
  #define X_MICROSTEPS 16     //number of microsteps

  //#define X2_IS_TMC
  #define X2_MAX_CURRENT 1000  //in mA
  #define X2_SENSE_RESISTOR 91 //in mOhms
  #define X2_MICROSTEPS 16     //number of microsteps

  //#define Y_IS_TMC
  #define Y_MAX_CURRENT 1000  //in mA
  #define Y_SENSE_RESISTOR 91 //in mOhms
  #define Y_MICROSTEPS 16     //number of microsteps

  //#define Y2_IS_TMC
  #define Y2_MAX_CURRENT 1000  //in mA
  #define Y2_SENSE_RESISTOR 91 //in mOhms
  #define Y2_MICROSTEPS 16     //number of microsteps

  //#define Z_IS_TMC
  #define Z_MAX_CURRENT 1000  //in mA
  #define Z_SENSE_RESISTOR 91 //in mOhms
  #define Z_MICROSTEPS 16     //number of microsteps

  //#define Z2_IS_TMC
  #define Z2_MAX_CURRENT 1000  //in mA
  #define Z2_SENSE_RESISTOR 91 //in mOhms
  #define Z2_MICROSTEPS 16     //number of microsteps

#endif

/******************************************************************************\
 * enable this section if you have L6470  motor drivers.
 * you need to import the L6470 library into the Arduino IDE for this
 ******************************************************************************/

// @section l6470

//#define HAVE_L6470DRIVER
#if ENABLED(HAVE_L6470DRIVER)

  //#define X_IS_L6470
  #define X_MICROSTEPS 16     //number of microsteps
  #define X_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define X_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define X_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define X2_IS_L6470
  #define X2_MICROSTEPS 16     //number of microsteps
  #define X2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define X2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define X2_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define Y_IS_L6470
  #define Y_MICROSTEPS 16     //number of microsteps
  #define Y_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define Y_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Y_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define Y2_IS_L6470
  #define Y2_MICROSTEPS 16     //number of microsteps
  #define Y2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define Y2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Y2_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define Z_IS_L6470
  #define Z_MICROSTEPS 16     //number of microsteps
  #define Z_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define Z_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Z_STALLCURRENT 1500 //current in mA where the driver will detect a stall

  //#define Z2_IS_L6470
  #define Z2_MICROSTEPS 16     //number of microsteps
  #define Z2_K_VAL 50          // 0 - 255, Higher values, are higher power. Be careful not to go too high
  #define Z2_OVERCURRENT 2000  //maxc current in mA. If the current goes over this value, the driver will switch off
  #define Z2_STALLCURRENT 1500 //current in mA where the driver will detect a stall

#endif

/**
 * TWI/I2C BUS
 *
 * This feature is an EXPERIMENTAL feature so it shall not be used on production
 * machines. Enabling this will allow you to send and receive I2C data from slave
 * devices on the bus.
 *
 * ; Example #1
 * ; This macro send the string "Marlin" to the slave device with address 0x63 (99)
 * ; It uses multiple M155 commands with one B<base 10> arg
 * M155 A99  ; Target slave address
 * M155 B77  ; M
 * M155 B97  ; a
 * M155 B114 ; r
 * M155 B108 ; l
 * M155 B105 ; i
 * M155 B110 ; n
 * M155 S1   ; Send the current buffer
 *
 * ; Example #2
 * ; Request 6 bytes from slave device with address 0x63 (99)
 * M156 A99 B5
 *
 * ; Example #3
 * ; Example serial output of a M156 request
 * echo:i2c-reply: from:99 bytes:5 data:hello
 */

// @section i2cbus

//#define EXPERIMENTAL_I2CBUS

#endif // CONFIGURATION_ADV_H
