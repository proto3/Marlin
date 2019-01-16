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

#ifndef BOARD_NAME
  #define BOARD_NAME "TEENSY 3.6"
#endif

#define X_MIN_PIN          24
#define X_MAX_PIN          25
#define Y_MIN_PIN          26
#define Y_MAX_PIN          27
#define Z_MIN_PIN          30
#define Z_MAX_PIN          29
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  28
#endif

#define X_STEP_PIN         39
#define X_DIR_PIN          38
#define X_ENABLE_PIN       23

#define Y_STEP_PIN         37
#define Y_DIR_PIN          36
#define Y_ENABLE_PIN       22

#define Z_STEP_PIN         35
#define Z_DIR_PIN          34
#define Z_ENABLE_PIN       21

#define LCD_PINS_RS        15 //CS chip select /SS chip slave select
#define LCD_PINS_ENABLE    11 //SID (MOSI)
#define LCD_PINS_D4        13 //SCK (CLK) clock

#define BEEPER_PIN         20
#define BTN_EN1             4
#define BTN_EN2             3
#define BTN_ENC             5

#define SD_DETECT_PIN      -1
#define KILL_PIN            2

#define PLASMA_CONTROL_PIN 33
#define PLASMA_TRANSFER_PIN 6

#define SOFTWARE_RESET *(volatile uint32_t *)0xE000ED0C = 0x5FA0004;
