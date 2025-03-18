/***********************************************************************
 * @file      oscillator.h
 * @version   1.0
 * @brief     Header file for oscillator configuration.
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
 ***********************************************************************/



#ifndef OSCILLATORS_H
#define OSCILLATORS_H

/**
 * @brief Configures and selects the appropriate oscillator for the system.
 *
 * This function enables and selects either the **ULFRCO** (Ultra Low Frequency RC Oscillator)
 * or **LFXO** (Low-Frequency Crystal Oscillator) based on the configured energy mode.
 * It also sets the clock prescaler and enables the LETIMER0 clock.
 *
 * @note - **ULFRCO** is used when `LOWEST_ENERGY_MODE == 3` (for low power applications).
 *       - **LFXO** is used otherwise (for higher precision timing).
 */
void configure_oscillators(void);

#endif // OSCILLATORS_H
