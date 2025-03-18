/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Date:        02-25-2022
 * Author:      Dave Sluiter
 * Description: This code was created by the Silicon Labs application wizard
 *              and started as "Bluetooth - SoC Empty".
 *              It is to be used only for ECEN 5823 "IoT Embedded Firmware".
 *              The MSLA referenced above is in effect.
 *
 *
 *
 * Student edit: Add your name and email address here:
 * @student    Lokesh Senthil Kumar, lokesh.senthilkumar@Colorado.edu
 *
 *
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "sl_power_manager.h"

#include "sl_status.h"             // for sl_status_print()
#include "src/ble_device_type.h"
#include "src/gpio.h"
#include "src/lcd.h"
#include "src/timers.h" // LETIMER configuration file
#include "src/oscillators.h" //Oscillator configuration file
#include "src/scheduler.h"
#include "src/i2c.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"




// *************************************************
// Power Manager
// *************************************************

// See: https://docs.silabs.com/gecko-platform/latest/service/power_manager/overview
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)

// -----------------------------------------------------------------------------
// defines for power manager callbacks
// -----------------------------------------------------------------------------
// Return values for app_is_ok_to_sleep():
//   Return false to keep sl_power_manager_sleep() from sleeping the MCU.
//   Return true to allow system to sleep when you expect/want an IRQ to wake
//   up the MCU from the call to sl_power_manager_sleep() in the main while (1)
//   loop.
//
//Based on the LOWEST_ENERGY_MODE it will set false for EM0 and True for others
#define APP_IS_OK_TO_SLEEP (LOWEST_ENERGY_MODE != 0)

// Return values for app_sleep_on_isr_exit():
//   SL_POWER_MANAGER_IGNORE; // The module did not trigger an ISR and it doesn't want to contribute to the decision
//   SL_POWER_MANAGER_SLEEP;  // The module was the one that caused the system wakeup and the system SHOULD go back to sleep
//   SL_POWER_MANAGER_WAKEUP; // The module was the one that caused the system wakeup and the system MUST NOT go back to sleep
//
// Notes:
//       SL_POWER_MANAGER_IGNORE, we see calls to app_process_action() on each IRQ. This is the
//       expected "normal" behavior.
//
//       SL_POWER_MANAGER_SLEEP, the function app_process_action()
//       in the main while(1) loop will not be called! It would seem that sl_power_manager_sleep()
//       does not return in this case.
//
//       SL_POWER_MANAGER_WAKEUP, doesn't seem to allow ISRs to run. Main while loop is
//       running continuously, flooding the VCOM port with printf text with LETIMER0 IRQs
//       disabled somehow, LED0 is not flashing.

#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_IGNORE)
//#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_SLEEP)
//#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_WAKEUP)

#endif // defined(SL_CATALOG_POWER_MANAGER_PRESENT)




// *************************************************
// Power Manager Callbacks
// The values returned by these 2 functions AND
// adding and removing power manage requirements is
// how we control when EM mode the MCU goes to when
// sl_power_manager_sleep() is called in the main
// while (1) loop.
// *************************************************

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)

bool app_is_ok_to_sleep(void)
{
  return APP_IS_OK_TO_SLEEP;
} // app_is_ok_to_sleep()

sl_power_manager_on_isr_exit_t app_sleep_on_isr_exit(void)
{
  return APP_SLEEP_ON_ISR_EXIT;
} // app_sleep_on_isr_exit()

#endif // defined(SL_CATALOG_POWER_MANAGER_PRESENT)



/**************************************************************************//**
 * Application Init.
 *****************************************************************************/

/**
 * @brief Initializes the application components.
 *
 * This function is called once during system startup and initializes:
 * - **GPIO** for LED control and other peripherals.
 * - **Oscillators** for system clock configuration.
 * - **LETIMER0** for periodic system timing.
 * - **I2C communication** for sensor interfacing.
 * - **Power management settings** to optimize energy consumption.
 * - **Interrupts for LETIMER0** in the Nested Vector Interrupt Controller (NVIC).
 *
 * @note This function should not call any Bluetooth API functions until
 *       after the system has completed the boot event.
 */
SL_WEAK void app_init(void)
{
    // One-time initialization code for the application

    // Initialize GPIOs for LED control and other peripherals
    gpioInit();

    // Configure system oscillators for clock management
    configure_oscillators();


    // Set the system's lowest power mode requirement based on configuration
    if (LOWEST_ENERGY_MODE == 1 || LOWEST_ENERGY_MODE == 2) {
        sl_power_manager_add_em_requirement((sl_power_manager_em_t)LOWEST_ENERGY_MODE);
    }


} // app_init()



/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/

/**
 * @brief Processes system events using a state machine.
 *
 * This function implements an **event-driven state machine** that transitions between
 * different states based on the retrieved system events. The current state is stored
 * in a static variable and updated as necessary.
 *
 * @note This function is executed continuously in the main loop.
 *
 * @see process_state_machine()
 */
SL_WEAK void app_process_action(void)
{
  /*static app_state_t current_state = STATE_IDLE; // Initial state
  uint32_t events = schedulerGetEvents();        // Retrieve current events

  // Process events using the state machine
  process_state_machine(&current_state, events);*/
}

/**
 * @brief State machine for processing temperature measurement using Si7021 sensor.
 *
 * This function implements a state machine that transitions through various states
 * to manage the power-on, temperature measurement, and power-off sequence of the Si7021 sensor.
 * It reacts to events retrieved from the scheduler and ensures appropriate power management.
 *
 * @param current_state Pointer to the current state of the state machine.
 * @param events        Event flags retrieved from the scheduler.
 *
 * @note This state machine uses critical sections and event-driven logic to manage the sensor.
 */
void process_state_machine(app_state_t *current_state, uint32_t events)
{
    switch (*current_state) {

        case STATE_IDLE:
            if (events & EVENT_UF) {  // Underflow event detected
                Si7021_PowerOn();     // Power on the Si7021 sensor
                timerWaitUs_irq(80000);  // Wait 80ms for sensor stabilization
                *current_state = STATE_POWER_ON;  // Transition to STATE_POWER_ON
            }
            break;

        case STATE_POWER_ON:
            if (events & EVENT_COMP1) {  // Compare Match 1 event
                //sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);  // Prevent sleep below EM1
                Si7021_StartTempMeasurement();  // Start temperature measurement
                *current_state = STATE_WAIT_TEMP_READY;  // Transition to STATE_WAIT_TEMP_READY
            }
            break;

        case STATE_WAIT_TEMP_READY:
            if (events & EVENT_I2C_TRANSFER_DONE) {  // I2C transfer completed
                //sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);  // Allow lower energy mode
                NVIC_DisableIRQ(I2C0_IRQn);  // Disable I2C interrupt
                timerWaitUs_irq(10800);  // Wait 10.8ms for temperature conversion
                *current_state = STATE_I2C_WRITE;  // Transition to STATE_I2C_WRITE
            }
            break;

        case STATE_I2C_WRITE:
            if (events & EVENT_COMP1) {  // Compare Match 1 event
                //sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);  // Prevent sleep below EM1
                Si7021_ReadTemperature();  // Read the temperature from the sensor
                *current_state = STATE_I2C_READ;  // Transition to STATE_I2C_READ
            }
            break;

        case STATE_I2C_READ:
            if (events & EVENT_I2C_TRANSFER_DONE) {  // I2C transfer completed
                //sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);  // Allow lower energy mode
                NVIC_DisableIRQ(I2C0_IRQn);  // Disable I2C interrupt
                print_temperature();  // Print the measured temperature
                Si7021_PowerOff();  // Power off the sensor
                *current_state = STATE_IDLE;  // Return to STATE_IDLE
            }
            break;

        default:
            *current_state = STATE_IDLE;  // Handle unexpected states by returning to STATE_IDLE
            break;
    }
} //process_state_machine()



/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *
 * The code here will process events from the Bluetooth stack. This is the only
 * opportunity we will get to act on an event.
 *
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{

  //handling the BLE stack events
   handle_ble_event(evt);
   //if the server is enables then temperature state machine runs.
#if DEVICE_IS_BLE_SERVER
  // sequence through states driven by events
   Temperature_state_machine(evt);

#else
   discovery_state_machine(evt);
#endif
} // sl_bt_on_event()

