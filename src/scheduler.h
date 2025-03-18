/***********************************************************************
 * @file      scheduler.h
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

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include "ble.h"
// ---------------------------------------------------------------------
// Event Flag Definitions
// ---------------------------------------------------------------------

/** @brief LETIMER0 Underflow event flag. */
#define EVENT_UF                 (1 << 0)

/** @brief LETIMER0 Compare Match 1 event flag. */
#define EVENT_COMP1              (1 << 1)

/** @brief I2C Transfer Done event flag. */
#define EVENT_I2C_TRANSFER_DONE  (1 << 2)

#define EVENT_PB0_PRESS  (1 << 3)

#define EVENT_PB0_RELEASE  (1 << 4)

typedef enum {
    OPENED,              // Initial state, waiting for connection
    DISCOVERING_SERVICE,         // Discovering the HTM service
    DISCOVERING_CHAR,            // Discovering the temperature characteristic
    CLOSED
} discovery_state_t;
// ---------------------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------------------

/**
 * @brief Sets the Underflow (UF) event flag.
 */
void schedulerSetEventUF(void);

/**
 * @brief Sets the Compare Match 1 (COMP1) event flag.
 */
void schedulerSetEventComp1(void);

/**
 * @brief Sets the I2C Transfer Done event flag.
 */
void schedulerSetEventI2CTransferDone(void);

/**
 * @brief Retrieves and clears the next scheduled event.
 *
 * This function checks for active events in priority order and returns
 * the next available event. The corresponding event flag is cleared after retrieval.
 *
 * @return `uint32_t` representing the next active event flag, or `0` if no event is active.
 */
uint32_t schedulerGetEvents(void);

/**
 * @brief State machine for processing BLE-connected temperature measurement.
 *
 * This function handles the temperature measurement process based on external events.
 * It follows a **state machine approach**, managing power states, sensor interactions,
 * and BLE updates.
 *
 * @param evt Pointer to the Bluetooth event structure (`sl_bt_msg_t`).
 */
void Temperature_state_machine(sl_bt_msg_t *evt);

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
void discovery_state_machine(sl_bt_msg_t *evt);

/**
 * @brief Signals a **Push Button (PB0) Press Event** to the Bluetooth stack.
 *
 * This function:
 * - Uses **critical sections** to ensure **atomic event signaling**.
 * - Calls `sl_bt_external_signal(EVENT_PB0_PRESS)` to inform the BLE stack.
 * - Allows **safe handling of PB0 presses** in the event-driven architecture.
 */
void schedulerSetEventPB0_Pressed(void);

/**
 * @brief Signals a **Push Button (PB0) Release Event** to the Bluetooth stack.
 *
 * This function:
 * - Uses **critical sections** to ensure **atomic event signaling**.
 * - Calls `sl_bt_external_signal(EVENT_PB0_RELEASE)` to notify the BLE stack.
 * - Enables **event-driven processing** for PB0 release detection.
 */
void schedulerSetEventPB0_Released(void);
#endif
