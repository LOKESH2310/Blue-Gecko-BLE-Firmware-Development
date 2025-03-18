/***********************************************************************
 * @file      irq.c
 * @version   1.0
 * @brief     IRQ configuration.
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
#include "irq.h"
#include "gpio.h"
#include "em_core.h"
#include "scheduler.h"
#include "timers.h"
#include "em_i2c.h"
#include "app.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

// ---------------------------------------------------------------------
// Static Variables
// ---------------------------------------------------------------------
/** @brief Millisecond counter to track elapsed time. */
static volatile uint32_t millisecond_count = 0;

/** @brief Clock frequency for LETIMER based on the selected energy mode. */
uint32_t CLOCK = ((LOWEST_ENERGY_MODE == 3) ? LOW_POWER_CLOCK_FREQ : HIGH_POWER_CLOCK_FREQ) / PRESCALER_DIVISOR;

// ---------------------------------------------------------------------
// Function Definitions
// ---------------------------------------------------------------------

/**
 * @brief Returns the total elapsed time in milliseconds.
 *
 * This function calculates the elapsed time by combining the millisecond count
 * with the number of milliseconds since the last LETIMER underflow.
 *
 * @return Total elapsed time in milliseconds.
 */
uint32_t letimerMilliseconds(void) {
    uint32_t current_count = LETIMER_CounterGet(LETIMER0);
    uint32_t ticks = Micro_TO_TICKS(LETIMER_PERIOD_MS, CLOCK);
    uint32_t elapsed_ticks = ticks - current_count;
    uint32_t ms_since_last_overflow = (elapsed_ticks * 1000) / CLOCK;
    return millisecond_count + ms_since_last_overflow;
}

/**
 * @brief LETIMER0 interrupt handler.
 *
 * This function handles LETIMER0 interrupts for both **Underflow (UF)** and
 * **Compare Match 1 (COMP1)** events. It sets the appropriate scheduler events
 * for each interrupt and disables the COMP1 interrupt after handling it.
 */
void LETIMER0_IRQHandler(void)
{

      //Determining the source of the interrupt (only enabled interrupts)
      uint32_t reason = LETIMER_IntGetEnabled(LETIMER0);

      //Clearing the pending interrupts immediately
      LETIMER_IntClear(LETIMER0, reason);

      // Handle Underflow (UF) event
      if (reason & LETIMER_IF_UF) {
          schedulerSetEventUF(); // Set event for UF
          millisecond_count += LETIMER_PERIOD_MS; //update for timestamp
      }
      if (reason & LETIMER_IF_COMP1) {
          schedulerSetEventComp1();    // Set event for COMP1
          LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);  // Disable COMP1 interrupt after handling
      }


} // LETIMER0_IRQHandler()

/**
 * @brief I2C0 interrupt handler.
 *
 * This function handles I2C0 interrupts. It checks the status of the transfer
 * and sets the scheduler event for a successful transfer. If the transfer fails,
 * it logs an error with the failure status.
 */
void I2C0_IRQHandler(void)
{
    I2C_TransferReturn_TypeDef transferStatus = I2C_Transfer(I2C0);
    if (transferStatus == i2cTransferDone) {
        schedulerSetEventI2CTransferDone(); //set I2C event
    } else if(transferStatus<0) {
        LOG_ERROR("I2C Failed status: %d", transferStatus);
    }
}

/**
 * @brief Handles even GPIO interrupts (e.g., button presses).
 *
 * This function:
 * - **Reads enabled interrupt flags** using `GPIO_IntGetEnabled()`.
 * - **Clears the handled interrupts** using `GPIO_IntClear()`.
 * - **Checks if PB0 caused the interrupt** and determines its state.
 * - **Calls the appropriate scheduler function** based on press or release.
 */
void GPIO_EVEN_IRQHandler(void)
{
  uint32_t flags = GPIO_IntGetEnabled();
  GPIO_IntClear(flags);
  //LOG_INFO("C");
  if (flags & (1 << PB0_PIN)) {
    bool pressed = (GPIO_PinInGet(PB0_PORT, PB0_PIN) == 0);
    if (pressed) {
        //LOG_INFO("C1");
      schedulerSetEventPB0_Pressed();
    } else {
        //LOG_INFO("C2");
      schedulerSetEventPB0_Released();
    }
  }
}

