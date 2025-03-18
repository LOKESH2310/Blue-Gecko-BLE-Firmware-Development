/***********************************************************************
 * @file      scheduler.c
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
 * @note This module implements an **event-driven scheduling mechanism** using a
 *   volatile event flag. Events are set and retrieved in a thread-safe manner
 *   using **critical sections**.
 *
 ***********************************************************************/
#include "scheduler.h"
#include "em_core.h"
#include "em_letimer.h"
#include "ble.h"
#include "app.h"
#include "i2c.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "timers.h"
#include "ble.h"
#include "lcd.h"
// ---------------------------------------------------------------------
// GLobal Variables
// ---------------------------------------------------------------------
//used to remove 1st unwanted interrupt.
int debug=0;
// For the Health Thermometer Service (0x1809)
static const uint8_t thermometer_service_uuid[2] = { 0x09, 0x18 };

// For the Temperature Measurement Characteristic (0x2A1C)
static const uint8_t temperature_characteristics_uuid[2] = { 0x1C, 0x2A };

// ---------------------------------------------------------------------
// Function Definitions
// ---------------------------------------------------------------------

//used to skip initial comp1 interrupts

/**
 * @brief Sets the Underflow (UF) event flag.
 *
 * This function signals the **EVENT_UF** using `sl_bt_external_signal()`
 * inside a **critical section** to ensure atomic updates.
 */
void schedulerSetEventUF(void)
{
    CORE_DECLARE_IRQ_STATE;  // Declare IRQ state variable
    CORE_ENTER_CRITICAL();   // Enter critical section
    sl_bt_external_signal(EVENT_UF); //Bluetooth API to set UF
    CORE_EXIT_CRITICAL();    // Exit critical section (Re-enable IRQs)
} // schedulerSetEventUF()

/**
 * @brief Sets the Compare Match 1 (COMP1) event flag.
 *
 * If **debug mode** is enabled, this function signals **EVENT_COMP1** using
 * `sl_bt_external_signal()`. If **debug mode is disabled**, it enables it.
 */
void schedulerSetEventComp1(void)
{
  /* My firmware is showing some error by generarting the comp1 interrupts
   * without calling it. TA Mr.Vishnu checked and i checked the code
   * but couldn't able to find where the problem lies. So he suggested to
   * implement a logic to skip the 1st unwanted comp1 interrupt.
   *
   * Thats why i am using a variable "debug" to skip it
   */
    if (debug) {
        CORE_DECLARE_IRQ_STATE;
        CORE_ENTER_CRITICAL();
        sl_bt_external_signal(EVENT_COMP1); //Bluetooth API to set Comp1
        CORE_EXIT_CRITICAL();
    } else {
        debug = 1;
    }
}

/**
 * @brief Sets the I2C Transfer Done event flag.
 *
 * This function signals **EVENT_I2C_TRANSFER_DONE** using `sl_bt_external_signal()`
 * inside a **critical section** to ensure atomic updates.
 */
void schedulerSetEventI2CTransferDone(void)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    sl_bt_external_signal(EVENT_I2C_TRANSFER_DONE); //Bluetooth API to set i2c
    CORE_EXIT_CRITICAL();
}//schedulerSetEventI2CTransferDone()

/**
 * @brief Retrieves and clears the next scheduled event.
 *
 * This function checks for active events in priority order and clears
 * the corresponding flag before returning the event.
 *
 * @return `uint32_t` containing the next active event.
 *         - `EVENT_UF` for Underflow event.
 *         - `EVENT_COMP1` for Compare Match 1 event.
 *         - `EVENT_I2C_TRANSFER_DONE` for I2C Transfer Done event.
 *         - `0` if no event is active.
 */
uint32_t schedulerGetEvents(void)
{

   /* uint32_t get_event=0; // Read event flags

    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    if (event_flags & EVENT_UF) {
            get_event = EVENT_UF;
            event_flags &= ~EVENT_UF; // Clear the event flag
        } else if (event_flags & EVENT_COMP1) {
            get_event = EVENT_COMP1;
            event_flags &= ~EVENT_COMP1;// Clear the event flag
        } else if (event_flags & EVENT_I2C_TRANSFER_DONE) {
            get_event = EVENT_I2C_TRANSFER_DONE;
            event_flags &= ~EVENT_I2C_TRANSFER_DONE;// Clear the event flag
        }
    CORE_EXIT_CRITICAL();

    return get_event; // Return active events*/
    return 0;
} //schedulerGetEvents()



/**
 * @brief Signals a **Push Button (PB0) Press Event** to the Bluetooth stack.
 *
 * This function:
 * - Uses **critical sections** to ensure **atomic event signaling**.
 * - Calls `sl_bt_external_signal(EVENT_PB0_PRESS)` to inform the BLE stack.
 * - Allows **safe handling of PB0 presses** in the event-driven architecture.
 */
void schedulerSetEventPB0_Pressed(void) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(EVENT_PB0_PRESS); // Send event to Bluetooth stack
  CORE_EXIT_CRITICAL();
}

/**
 * @brief Signals a **Push Button (PB0) Release Event** to the Bluetooth stack.
 *
 * This function:
 * - Uses **critical sections** to ensure **atomic event signaling**.
 * - Calls `sl_bt_external_signal(EVENT_PB0_RELEASE)` to notify the BLE stack.
 * - Enables **event-driven processing** for PB0 release detection.
 */
void schedulerSetEventPB0_Released(void) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(EVENT_PB0_RELEASE); // Send event to Bluetooth stack
  CORE_EXIT_CRITICAL();
}


/**
 * @brief State machine for processing BLE-connected temperature measurement.
 *
 * This function handles the temperature measurement process based on external events.
 * It follows a **state machine approach**, managing power states, sensor interactions,
 * and BLE updates.
 *
 * @param evt Pointer to the Bluetooth event structure (`sl_bt_msg_t`).
 */
void Temperature_state_machine(sl_bt_msg_t *evt) {

  // Retrieve BLE data pointer
  ble_data_struct_t *ble = getBleDataPtr();

  // Ensure BLE connection is open and OK to send HTM indications
  //used this for extra credit.
  if (ble->connection_open && ble->ok_to_send_htm_indications) {
  // Initialize and enable required peripherals
      configure_timer();
      I2C_init();
  // Enable the LETIMER0 interrupt in the Nested Vector Interrupt Controller (NVIC)
     NVIC_ClearPendingIRQ(LETIMER0_IRQn);
     NVIC_EnableIRQ(LETIMER0_IRQn);

  // State machine tracking
  static app_state_t current_state = STATE_IDLE;
  uint32_t eventValue = 0;  // Store the event

  // Ensure event is from external signal
  if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_system_external_signal_id) {
      eventValue = evt->data.evt_system_external_signal.extsignals;
  } else {
      return;  // Ignore non-external events
  }

  switch (current_state) {
      case STATE_IDLE:
          if (eventValue == EVENT_UF) {
              //removed for LCD
              //Si7021_PowerOn(); //Temperature sensor power on
              timerWaitUs_irq(80000); // 80ms stabilization
              current_state = STATE_POWER_ON;
          }
          break;

      case STATE_POWER_ON:
          if (eventValue == EVENT_COMP1) {
              sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
              Si7021_StartTempMeasurement(); //Write command i2c
              current_state = STATE_WAIT_TEMP_READY;
          }
          break;

      case STATE_WAIT_TEMP_READY:
          if (eventValue == EVENT_I2C_TRANSFER_DONE) {
              sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
              NVIC_DisableIRQ(I2C0_IRQn);
              timerWaitUs_irq(10800); //conversion time
              current_state = STATE_I2C_WRITE;
          }
          break;

      case STATE_I2C_WRITE:
          if (eventValue == EVENT_COMP1) {
              sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
              Si7021_ReadTemperature(); //i2c Read command
              current_state = STATE_I2C_READ;
          }
          break;

      case STATE_I2C_READ:
          if (eventValue == EVENT_I2C_TRANSFER_DONE) {
              sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
              NVIC_DisableIRQ(I2C0_IRQn);
              float raw_temp = Si7021_GetTemperature();
              GATT_Update(raw_temp); //update the temperature dat to the GATT
              //removed for LCD
              //Si7021_PowerOff();
              current_state = STATE_IDLE;
          }
          break;

      default:
          current_state = STATE_IDLE;
          break;
  }
  }else {
      NVIC_ClearPendingIRQ(LETIMER0_IRQn);
      LETIMER_Enable(LETIMER0, false);
  }
}


/**
 * @brief Handles BLE service and characteristic discovery state machine.
 *
 * This function manages the BLE **GATT discovery process**:
 * - **DISCOVERY_IDLE**: Initiates **primary service discovery** when a connection opens.
 * - **DISCOVERING_SERVICE**: Proceeds to **characteristic discovery** upon service discovery.
 * - **DISCOVERING_CHAR**: Enables **notifications/indications** when the characteristic is found.
 * - **CLOSED**: Restarts scanning upon disconnection.
 *
 * @param evt Pointer to the **Bluetooth event** (`sl_bt_msg_t`).
 */

void discovery_state_machine(sl_bt_msg_t *evt) {
    static discovery_state_t state = OPENED;
    sl_status_t status;
    ble_data_struct_t *ble = getBleDataPtr();
    switch (state) {
      // ------------------------------------------------------
      // IDLE STATE - Initiate Service Discovery upon Connection
      // ------------------------------------------------------
        case OPENED:
          //checks for open connections
          if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_opened_id)
          {
              //start discovering services
            status=sl_bt_gatt_discover_primary_services_by_uuid(ble->connection_handle,
                                  sizeof(thermometer_service_uuid),thermometer_service_uuid);
            if(status!= SL_STATUS_OK)
              {
                LOG_ERROR("Service discovery failed: 0x%04x", (int)status);
              }
            state = DISCOVERING_SERVICE;
          }
            break;

        // ------------------------------------------------------
        // DISCOVERING SERVICE - Move to Characteristic Discovery
        // ------------------------------------------------------
        case DISCOVERING_SERVICE:

          //checks for procedure completed id
          if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
            {
              if(ble->ServiceHandle!=0)
              {
                  //start discovering characteristics
                  status=sl_bt_gatt_discover_characteristics_by_uuid(ble->connection_handle,
                                      ble->ServiceHandle,sizeof(temperature_characteristics_uuid),temperature_characteristics_uuid);
                  if(status!=SL_STATUS_OK)
                   {
                      LOG_ERROR("Char discovery failed: 0x%04x", (int)status);
                   }
                  state= DISCOVERING_CHAR;
               }}
            break;
        // ------------------------------------------------------
        // DISCOVERING CHARACTERISTIC - Enable Notifications/Indications
        // ------------------------------------------------------
        case DISCOVERING_CHAR:
          //checks for characteristics completed id
            if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id)
              {
                if(ble->CharHandle!=0)
                {
                    //start indications
                    status=sl_bt_gatt_set_characteristic_notification(ble->connection_handle,ble->CharHandle, sl_bt_gatt_indication);
                    if(status!=SL_STATUS_OK)
                     {
                        LOG_ERROR("Char discovery failed: 0x%04x", (int)status);
                     }
                    state= CLOSED;
                 }}
            break;

        // ------------------------------------------------------
        // CLOSED STATE - Restart Scanning upon Disconnection
        // ------------------------------------------------------
        case CLOSED:
          //checks for closed id
            if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id) {
                // Restart BLE scanning for new devices
                status = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
                if (status != SL_STATUS_OK) {
                  LOG_ERROR("scanner_start failed: 0x%x", status);
                }
                displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");
                displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
                displayPrintf(DISPLAY_ROW_BTADDR2, " ");
                state = OPENED;
            }
            break;
    }
}


