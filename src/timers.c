/***********************************************************************
 * @file      timers.c
 * @version   1.0
 * @brief     LETIMER configuration.
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


#include "em_letimer.h"
#include "timers.h"
#include "app.h"
#include "gpio.h"

// Include logging specifically for this .c file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


//clock frequency for both letimer and timerwait_us
uint32_t CLOCK_FREQUENCY = ((LOWEST_ENERGY_MODE == 3) ? LOW_POWER_CLOCK_FREQ : HIGH_POWER_CLOCK_FREQ)/PRESCALER_DIVISOR;

/**
 * @brief Configures and initializes the LETIMER0 peripheral.
 *
 * This function sets up the LETIMER0 low-energy timer with the required settings.
 * It configures clock frequency, timer period, and enables interrupts for proper timing.
 *
 * @note The timer runs in free-running mode and supports low-energy operation.
 */
void configure_timer(void)
{
      uint32_t Enable_int; // To enable interrupt (UF, COMP1)

      // Configure LETIMER initialization structure
      const LETIMER_Init_TypeDef letimerInitData = {
          false,          // enable: Don't enable timer at init, will enable last
          true,           // debugRun: Keep running while debugging
          true,           // comp0Top: Load COMP0 into CNT on underflow
          false,          // bufTop: Don't load COMP1 into COMP0
          0,              // out0Pol: Default output pin value
          0,              // out1Pol: Default output pin value
          letimerUFOANone,// ufoa0: No underflow output action
          letimerUFOANone,// ufoa1: No underflow output action
          letimerRepeatFree, // repMode: Free-running mode (timer repeats indefinitely)
          0               // COMP0 (top) value, will be set below
      };

      // Determine the base clock frequency based on energy mode
      //uint32_t base_clock = (LOWEST_ENERGY_MODE == 3) ? LOW_POWER_CLOCK_FREQ : HIGH_POWER_CLOCK_FREQ;

      // Apply prescaler division to the base clock
      //uint32_t CLOCK_FREQUENCY = base_clock / PRESCALER_DIVISOR;

      // Calculate timer ticks for the configured period
      uint32_t period_ticks = MS_TO_TICKS(LETIMER_PERIOD_MS, CLOCK_FREQUENCY);

      // Initialize LETIMER with the configuration
      LETIMER_Init(LETIMER0, &letimerInitData);

      // Set COMP0 for total period duration
      LETIMER_CompareSet(LETIMER0, 0, period_ticks);

      // Clear any pending interrupts before enabling
      LETIMER_IntClear(LETIMER0, 0xFFFFFFFF); // Clears all LETIMER0 interrupts

      // Enable interrupts for COMP1 (LED off event) and Underflow (restart cycle)
      Enable_int = LETIMER_IEN_UF | LETIMER_IEN_COMP1;
      LETIMER_IntEnable(LETIMER0, Enable_int);

      // Enable LETIMER0 to start counting
      LETIMER_Enable(LETIMER0, true);
} // configure_timer()

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
void timerWaitUs(uint32_t us_wait)
{
    // Validate input range (10,000 µs - 100,000 µs)
      if ((us_wait < 10000) || (us_wait > 100000)) {
          LOG_ERROR("Microsecond value is out of range");
          return;
      }

      // Convert microseconds to LETIMER ticks
      uint32_t ticks = uS_TO_TICKS(us_wait, CLOCK_FREQUENCY);

      // Capture current LETIMER0 counter value
      uint32_t start = LETIMER_CounterGet(LETIMER0);

      // Wait for the required number of ticks to elapse
      while ((start - LETIMER_CounterGet(LETIMER0)) <= ticks);
} //timerWaitUs()

/**
 * @brief Creates a non-blocking delay in microseconds using LETIMER0 and interrupts.
 *
 * This function configures **LETIMER0 Compare Register 1 (COMP1)** to generate an
 * interrupt after the specified delay in microseconds. It validates the input delay
 * and enables the COMP1 interrupt for asynchronous processing.
 *
 * @param us_wait Delay duration in microseconds (Valid range: 10 µs - 100,000 µs).
 */
void timerWaitUs_irq(uint32_t us_wait)
{
        // Validate input range (10 µs - 100,000 µs)
        if ((us_wait < 10) || (us_wait > 100000)) {
            LOG_ERROR("Microsecond value is out of range");
            return;
        }
        // Convert microseconds to LETIMER ticks
        uint32_t ticks = uS_TO_TICKS(us_wait, CLOCK_FREQUENCY);

        // Get the current counter value of LETIMER0
        uint32_t current_count = LETIMER_CounterGet(LETIMER0);

        // Calculate the target value for COMP1
        ticks = current_count - ticks;

        // Set COMP1 for the required delay
        LETIMER_CompareSet(LETIMER0, 1, ticks);

        // Enable COMP1 interrupt for the delay
        LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);
} //timerWaitUs_irq()
