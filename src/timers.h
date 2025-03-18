/***********************************************************************
 * @file      timers.h
 * @version   1.0
 * @brief     Header file for LETIMER Configuration.
 *
 * @author    Lokesh Senthil Kumar, lokesh.senthilkumar@colorado.edu
 * @date      30-01-2025
 *
 * @institution University of Colorado Boulder (UCB)
 * @course      ECEN 5823: IoT Embedded Firmware
 * @instructor  Chris Choi
 *
 *
 * @resources  Utilized Silicon Labs' Peripheral Libraries (em_letimer.h)
 *             and Course slides
 *
 ***********************************************************************/


#ifndef TIMERS_H
#define TIMERS_H

// LETIMER configurations
#define LETIMER_PERIOD_MS 3000

// Define clock frequencies for different energy modes
#define LOW_POWER_CLOCK_FREQ  1000    // Clock frequency in EM3
#define HIGH_POWER_CLOCK_FREQ 32768   // Clock frequency in EM0, EM1, EM2

// Define prescaler value
#define PRESCALER_DIVISOR     2       // Prescaler divides the clock by 2

#define min_us_delay 10000 //min blocking delay for timer wait
#define max_us_delay 100000 //max blocking delay for timer wait


// Define conversion factor for milliseconds to ticks
#define MS_TO_TICKS(ms, clk)  (((ms) * (clk)) / 1000)

// Define conversion factor for microseconds to ticks
#define uS_TO_TICKS(us, clk)  (((us) * (clk)) / 1000000)

/**
 * @brief Configures and initializes the LETIMER0 peripheral.
 *
 * This function sets up the LETIMER0 low-energy timer with the required settings.
 * It configures clock frequency, timer period, and enables interrupts for proper timing.
 *
 * @note The timer runs in free-running mode and supports low-energy operation.
 */
void configure_timer(void);

/**
 * @brief Delays execution for a specified number of microseconds.
 *
 * This function creates a blocking delay by waiting for a specified
 * duration in microseconds, converted into LETIMER0 clock ticks.
 *
 * @param us_wait The delay duration in microseconds (Valid range: 10,000 - 100,000 µs).
 *
 * @note
 * - The function verifies that `us_wait` is within a valid range.
 * - If `us_wait` is out of bounds, an error is logged, and the function exits.
 * - The function assumes `CLOCK_FREQUENCY` is correctly configured.
 */
void timerWaitUs(uint32_t us_wait);

/**
 * @brief Creates a non-blocking delay in microseconds using LETIMER0 and interrupts.
 *
 * This function configures **LETIMER0 Compare Register 1 (COMP1)** to generate an
 * interrupt after the specified delay in microseconds. It validates the input delay
 * and enables the COMP1 interrupt for asynchronous processing.
 *
 * @param us_wait Delay duration in microseconds (Valid range: 10 µs - 100,000 µs).
 */
void timerWaitUs_irq(uint32_t us_wait);

#endif // TIMERS_H
