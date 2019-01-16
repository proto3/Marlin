/* **************************************************************************

 Marlin 3D Printer Firmware
 Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************************************/


/**
 * Teensy3.5 __MK64FX512__
 * Teensy3.6 __MK66FX1M0__
 */

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

// #include "HAL.h"
#include "HAL_timers_Teensy.h"

/** \brief Instruction Synchronization Barrier
  Instruction Synchronization Barrier flushes the pipeline in the processor,
  so that all instructions following the ISB are fetched from cache or
  memory, after the instruction has been completed.
*/
FORCE_INLINE static void __ISB(void) {
  __asm__ __volatile__("isb 0xF":::"memory");
}

/** \brief Data Synchronization Barrier
  This function acts as a special kind of Data Memory Barrier.
  It completes when all explicit memory accesses before this instruction complete.
*/
FORCE_INLINE static void __DSB(void) {
  __asm__ __volatile__("dsb 0xF":::"memory");
}

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  switch (timer_num) {
    case 0:
      FTM0_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
      FTM0_SC = 0x00; // Set this to zero before changing the modulus
      FTM0_CNTIN = 0x0001; // Set default value for count reset
      FTM0_CNT = 0x1234; // Reset count by writing any number
      FTM0_MOD = FTM0_TIMER_RATE / frequency <  0xFFFF ? FTM0_TIMER_RATE / frequency : 0xFFFF;
      FTM0_SC = (FTM_SC_CLKS(0b1) & FTM_SC_CLKS_MASK) | (FTM_SC_PS(FTM0_TIMER_PRESCALE_BITS) & FTM_SC_PS_MASK) | FTM_SC_TOIE; // Bus clock 60MHz divided by prescaler 16
      FTM0_C0SC =  FTM_CSC_MSA | FTM_CSC_ELSA;
      break;
    case 1:
      FTM1_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN; // Disable write protection, Enable FTM1
      FTM1_SC = 0x00; // Set this to zero before changing the modulus
      FTM1_CNT = 0x0000; // Reset the count to zero
      FTM1_MOD = 0xFFFF; // max modulus = 65535
      FTM1_C0V = FTM1_TIMER_RATE / frequency; // Initial FTM Channel 0 compare value 65535
      FTM1_SC = (FTM_SC_CLKS(0b1) & FTM_SC_CLKS_MASK) | (FTM_SC_PS(FTM1_TIMER_PRESCALE_BITS) & FTM_SC_PS_MASK); // Bus clock 60MHz divided by prescaler 4
      FTM1_C0SC = FTM_CSC_CHIE | FTM_CSC_MSA | FTM_CSC_ELSA;
      break;
    case 2:
      FTM2_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
      FTM2_SC = 0x00; // Set this to zero before changing the modulus
      FTM2_CNTIN = 0x0001; // Set default value for count reset
      FTM2_CNT = 0x1234; // Reset count by writing any number
      FTM2_MOD = FTM2_TIMER_RATE / frequency <  0xFFFF ? FTM2_TIMER_RATE / frequency : 0xFFFF;
      FTM2_SC = (FTM_SC_CLKS(0b1) & FTM_SC_CLKS_MASK) | (FTM_SC_PS(FTM2_TIMER_PRESCALE_BITS) & FTM_SC_PS_MASK) | FTM_SC_TOIE; // Bus clock 60MHz divided by prescaler 16
      FTM2_C0SC =  FTM_CSC_MSA | FTM_CSC_ELSA;
      break;
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  switch(timer_num) {
    case 0: NVIC_ENABLE_IRQ(IRQ_FTM0); break;
    case 1: NVIC_ENABLE_IRQ(IRQ_FTM1); break;
    case 2: NVIC_ENABLE_IRQ(IRQ_FTM2); break;
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: NVIC_DISABLE_IRQ(IRQ_FTM0); break;
    case 1: NVIC_DISABLE_IRQ(IRQ_FTM1); break;
    case 2: NVIC_DISABLE_IRQ(IRQ_FTM2); break;
  }

  // We NEED memory barriers to ensure Interrupts are actually disabled!
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: return NVIC_IS_ENABLED(IRQ_FTM0);
    case 1: return NVIC_IS_ENABLED(IRQ_FTM1);
    case 2: return NVIC_IS_ENABLED(IRQ_FTM2);
  }
  return false;
}

void HAL_timer_isr_prologue(const uint8_t timer_num) {
  switch(timer_num) {
    case 0:
      FTM0_SC &= ~FTM_SC_TOF; // Clear FTM Overflow flag
      if(ftm0_pending_modulus_update)
      {
        FTM0_SC = 0; // clock must be disconnected to update modulus
        FTM0_MOD = ftm0_target_modulus;
        FTM0_SC = (FTM_SC_CLKS(0b1) & FTM_SC_CLKS_MASK) | (FTM_SC_PS(FTM0_TIMER_PRESCALE_BITS) & FTM_SC_PS_MASK) | FTM_SC_TOIE;
      }
      break;
    case 1:
      FTM1_CNT = 0x0000;
      FTM1_SC &= ~FTM_SC_TOF; // Clear FTM Overflow flag
      FTM1_C0SC &= ~FTM_CSC_CHF; // Clear FTM Channel Compare flag
      break;
    case 2:
      FTM2_SC &= ~FTM_SC_TOF; // Clear FTM Overflow flag
      if(ftm2_pending_modulus_update)
      {
        FTM2_SC = 0; // clock must be disconnected to update modulus
        FTM2_MOD = ftm2_target_modulus;
        FTM2_SC = (FTM_SC_CLKS(0b1) & FTM_SC_CLKS_MASK) | (FTM_SC_PS(FTM2_TIMER_PRESCALE_BITS) & FTM_SC_PS_MASK) | FTM_SC_TOIE;
      }
      break;
  }
}

#endif // Teensy3.5 or Teensy3.6
