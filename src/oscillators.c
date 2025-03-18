/***********************************************************************
 * @file      oscillator.c
 * @version   1.0
 * @brief     oscillator and Clock configuration.
 *
 * @author    Lokesh Senthil Kumar, lokesh.senthilkumar@colorado.edu
 * @date      30-01-2025
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823: IoT Embedded Firmware
 * @instructor  Chris Choi
 *
 *
 * @resources  Utilized Silicon Labs' Peripheral Libraries (em_cmu.h)
 *             and Course slides
 *
 *
 ***********************************************************************/


#include "em_cmu.h"
#include "app.h"

void configure_oscillators(void)
{

  // If operating in Energy Mode 3, use the ULFRCO oscillator
  if (LOWEST_ENERGY_MODE == 3) {
      CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);   // Enable ULFRCO
      CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO); // Select ULFRCO as LFA clock
  }
  // Otherwise, use the LFXO oscillator
  else {
      CMU_OscillatorEnable(cmuOsc_LFXO, true, true);     // Enable LFXO
      CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);  // Select LFXO as LFA clock
  }

  // Set the LETIMER0 clock prescaler to divide the clock by 2
  CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_2);

  // Enable the clock for LETIMER0 to ensure it is ready for use
  CMU_ClockEnable(cmuClock_LETIMER0, true);

} // configure_oscillators()
