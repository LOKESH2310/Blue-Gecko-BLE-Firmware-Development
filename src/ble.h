/***********************************************************************
 * @file      ble,h
 * @version   1.0
 * @brief     ble configuration.
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
 *
 ***********************************************************************/

#ifndef BLE_H
#define BLE_H

#include "sl_bluetooth.h"
#include "gatt_db.h"


// BLE Data Structure
typedef struct {
    bd_addr myAddress;                // Device Bluetooth address
    uint8_t myAddressType;            // Bluetooth address type

    // Server-Specific Values
    uint8_t advertisingSetHandle;     // Handle for advertising set
    uint8_t connection_handle;
    bool connection_open;             // True if a BLE connection is open
    bool ok_to_send_htm_indications;  // True if client has enabled indications
    bool ok_to_send_btn_indications;
    bool indication_in_flight;        // True if an indication is currently in-flight
    uint32_t ServiceHandle;
    uint32_t CharHandle;
} ble_data_struct_t;

/**
 * @brief Initializes Bluetooth (BLE) advertising and settings.
 *
 * This function configures the BLE stack, retrieves the device's identity address,
 * sets up an advertising set, and starts BLE advertising.
 */
void initialization_BLE(void);

/**
 * @brief Configures BLE scanning and starts discovery.
 *
 * This function:
 * 1. Sets default connection parameters (~75ms interval, 4 latency, 8.3s supervision timeout).
 * 2. Configures passive scanning parameters (50ms interval, 25ms window).
 * 3. Starts BLE scanning on 1M PHY in generic discovery mode.
 */
void client_scan(void);

/**
 * @brief Processes scanned BLE advertisement reports and attempts connection.
 *
 * This function:
 * - Extracts the advertiser's **Bluetooth address** and **address type**.
 * - Compares the scanned address with a known **server address**.
 * - **Stops scanning** if a match is found.
 * - **Initiates a BLE connection** with the server device.
 *
 * @param evt Pointer to the **BLE event** (`sl_bt_msg_t`).
 */
void report_id(sl_bt_msg_t *evt);

/**
 * @brief Converts IEEE-11073 32-bit float format to a standard floating-point value.
 *
 * This function decodes a **32-bit IEEE-11073 floating point value** into a standard
 * **float** representation. The format consists of:
 * - **8-bit exponent** (signed)
 * - **24-bit mantissa** (signed)
 *
 * @param buffer_ptr Pointer to a **5-byte buffer** containing the encoded value.
 *                   - `buffer_ptr[0]` (Flags, usually ignored)
 *                   - `buffer_ptr[1-3]` (24-bit mantissa, **little-endian**)
 *                   - `buffer_ptr[4]` (8-bit exponent, signed)
 *
 * @return Decoded **float** value.
 */
float FLOAT_TO_INT32(const uint8_t *buffer_ptr);
/**
 * @brief Handles BLE events and manages BLE connection states.
 *
 * This function processes BLE stack events and manages:
 * - **System Boot**: Starts advertising.
 * - **Connection Opened**: Stops advertising and sets connection parameters.
 * - **Connection Closed**: Restarts advertising.
 * - **Connection Parameter Updates**: Logs connection parameters.
 * - **GATT Indication Handling**: Manages HTM indications.
 *
 * @param evt Pointer to the received Bluetooth event (`sl_bt_msg_t`).
 */
void handle_ble_event(sl_bt_msg_t *evt);

/**
 * @brief Get pointer to the BLE data structure
 */
ble_data_struct_t* get_ble_data(void);

/**
 * @brief Updates the GATT database and sends a temperature indication.
 *
 * This function converts the temperature into the **IEEE-11073** format, updates
 * the **GATT temperature measurement attribute**, and sends an indication if
 * the BLE connection allows it.
 *
 * @param temperature_c Temperature in **Celsius** as a float.
 */
void GATT_Update(float temperature_c);



/**
 * @brief Updates the button state in the GATT database.
 *
 * This function:
 * - Writes the **new button state value** to the **GATT server attribute**.
 * - **Ensures proper BLE characteristic updates** by handling return status.
 * - **Logs an error** if the write operation fails.
 *
 * @param newValue The new button state (0 = released, 1 = pressed).
 */
static void update_button_state(uint8_t newValue);
#endif // BLE_H
