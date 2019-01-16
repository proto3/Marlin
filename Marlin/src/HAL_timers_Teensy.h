/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
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
 */
#pragma once

/**
 * Description: HAL for
 * Teensy3.5 (__MK64FX512__)
 * Teensy3.6 (__MK66FX1M0__)
 */

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <Arduino.h>
#include <stdint.h>
#include <macros.h>

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define FTM0_TIMER_PRESCALE_BITS 0b100
#define FTM1_TIMER_PRESCALE_BITS 0b010
#define FTM2_TIMER_PRESCALE_BITS 0b100

#define FTM0_TIMER_RATE (F_BUS / (1 << FTM0_TIMER_PRESCALE_BITS)) // 60MHz / 16 = 3750kHz
#define FTM1_TIMER_RATE (F_BUS / (1 << FTM1_TIMER_PRESCALE_BITS)) // 60MHz / 4  = 15MHz
#define FTM2_TIMER_RATE (F_BUS / (1 << FTM2_TIMER_PRESCALE_BITS)) // 60MHz / 16 = 3750MHz

#define STEP_TIMER_NUM 0
#define TEMP_TIMER_NUM 1
#define THC_TIMER_NUM  2

#define TEMP_TIMER_FREQUENCY    1000

#define STEPPER_TIMER_RATE      FTM0_TIMER_RATE
#define THC_TIMER_RATE          FTM2_TIMER_RATE

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT() HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER_NUM)

#define ENABLE_THC_INTERRUPT() HAL_timer_enable_interrupt(THC_TIMER_NUM)
#define DISABLE_THC_INTERRUPT() HAL_timer_disable_interrupt(THC_TIMER_NUM)

#define HAL_STEP_TIMER_ISR  extern "C" void ftm0_isr(void)
#define HAL_TEMP_TIMER_ISR  extern "C" void ftm1_isr(void)
#define HAL_THC_TIMER_ISR   extern "C" void ftm2_isr(void)

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);

static uint16_t ftm0_target_modulus = 0xFFFF, ftm2_target_modulus = 0xFFFF;
static bool ftm0_pending_modulus_update = false, ftm2_pending_modulus_update = false;

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const uint16_t compare) {
  switch (timer_num) {
    case 0:
      FTM0_SC = 0; // clock must be disconnected to update modulus

      if(FTM0_CNT > compare)
      {
        FTM0_MOD = FTM0_CNT;
        ftm0_target_modulus = compare;
        ftm0_pending_modulus_update = true;
      }
      else
      {
        FTM0_MOD = compare;
      }

      FTM0_SC = (FTM_SC_CLKS(0b1) & FTM_SC_CLKS_MASK) | (FTM_SC_PS(FTM0_TIMER_PRESCALE_BITS) & FTM_SC_PS_MASK) | FTM_SC_TOIE;
      break;
    case 1:
      FTM1_C0V = compare;
      break;
    case 2:
      FTM2_SC = 0; // clock must be disconnected to update modulus

      if(FTM2_CNT > compare)
      {
        FTM2_MOD = FTM2_CNT;
        ftm2_target_modulus = compare;
        ftm2_pending_modulus_update = true;
      }
      else
      {
        FTM2_MOD = compare;
      }

      FTM2_SC = (FTM_SC_CLKS(0b1) & FTM_SC_CLKS_MASK) | (FTM_SC_PS(FTM2_TIMER_PRESCALE_BITS) & FTM_SC_PS_MASK) | FTM_SC_TOIE;
      break;
  }
}

FORCE_INLINE static uint32_t HAL_timer_get_compare(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: return FTM0_MOD;
    case 1: return FTM1_C0V;
    case 2: return FTM2_MOD;
  }
  return 0;
}

FORCE_INLINE static uint32_t HAL_timer_get_count(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: return FTM0_CNT;
    case 1: return FTM1_CNT;
    case 2: return FTM2_CNT;
  }
  return 0;
}

void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

void HAL_timer_isr_prologue(const uint8_t timer_num);
#define HAL_timer_isr_epilogue(TIMER_NUM)
