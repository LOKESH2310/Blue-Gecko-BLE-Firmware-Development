/***********************************************************************
 * @file      ble.c
 * @version   1.0
 * @brief     ble configuration.
 *
 * @author    Lokesh Senthil Kumar, lokesh.senthilkumar@colorado.edu
 * @date      20-02-2025
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


#include "ble.h"
#include "app.h"
#include "gatt_db.h"
#include "sl_bt_api.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "lcd.h"
#include "ble_device_type.h"
#include "math.h"
#include "scheduler.h"
#include "gpio.h"

// BLE Data Structure
static ble_data_struct_t ble_data;
//server address hard coded
static bd_addr server_addr = SERVER_BT_ADDRESS;
//service UUID
uint8_t lsb = 0x09;
uint8_t msb = 0x18;
//characteristics UUID
uint8_t Clsb = 0x1C;
uint8_t Cmsb = 0x2A;
//report check for connectable and scannable
uint8_t report_id_check = 0x03;

//Converts a temperature value to IEEE 11073-20601 floating point format.
#define convertToIEEE11073(temp) \
  ( ((uint32_t)((uint8_t)-2) << 24) | (((int32_t)((temp) * 100)) & 0x00FFFFFF) )



// We'll store if we're bonded, and if we are waiting for passkey confirmation
static bool bonded = false;
static bool waitingForPasskeyConfirm = false;
uint8_t press = 0x01;
uint8_t release = 0x00;

/*
Bit 0:
0: Allow bonding without authentication
1: Bonding requires authentication (Man-in-the-Middle protection)

Bit 1:
0: Allow encryption without bonding
1: Encryption requires bonding. Note that this setting will also enable bonding.

Bit 2:
0: Allow bonding with legacy pairing
1: Secure connections only

Bit 3:
0: Bonding request does not need to be confirmed
1: Bonding requests need to be confirmed. Received bonding requests are notified by sl_bt_evt_sm_confirm_bonding*/
#define MY_SM_CONFIG_FLAGS  (0x0F)

// ---------------------------------------------------------------------
// Circular Buffer Configuration
// ---------------------------------------------------------------------
#define QUEUE_DEPTH       (16)  ///< Maximum number of entries in the circular buffer
#define USE_ALL_ENTRIES   (1)   ///< Set to 1 if allowing full buffer usage, 0 if leaving 1 slot empty

#define MAX_BUFFER_LENGTH  (5)  ///< Maximum buffer size per entry
#define MIN_BUFFER_LENGTH  (1)  ///< Minimum buffer size per entry

// ---------------------------------------------------------------------
// Queue Entry Structure
// ---------------------------------------------------------------------
/**
 * @struct queue_struct_t
 * @brief Defines a single entry in the circular buffer.
 */
typedef struct {
    uint16_t charHandle;                 ///< GATT DB handle associated with the entry
    uint32_t bufLength;                   ///< Length of data stored in 'buffer'
    uint8_t buffer[MAX_BUFFER_LENGTH];    ///< Data buffer (up to 5 bytes for HTM, 1 byte for button)
} queue_struct_t;

// ---------------------------------------------------------------------
// Circular Buffer Variables
// ---------------------------------------------------------------------
static queue_struct_t my_queue[QUEUE_DEPTH]; ///< Circular buffer storage
static uint32_t wptr = 0;   ///< Write pointer
static uint32_t rptr = 0;   ///< Read pointer
static bool Wflag = false;  ///< Flag indicating buffer is full
static bool Rflag = true;   ///< Flag indicating buffer is empty


// Private helper to get the next pointer index in a circular manner
static uint32_t nextPtr(uint32_t ptr) {
  return (ptr + 1) % QUEUE_DEPTH;
}

/**
 * @brief Resets the circular buffer to its initial state.
 *
 * This function:
 * - Resets the **write (wptr)** and **read (rptr) pointers** to **0**.
 * - Clears the **full (`Wflag`)** and **empty (`Rflag`)** status flags.
 * - Ensures the buffer is ready for new data storage.
 */
static void reset_queue(void) {
  Wflag = false;  // not full
  Rflag = true;   // is empty
  wptr = 0; // Reset write pointer
  rptr = 0;       // Reset read pointer
}

/**
 * @brief Writes a new entry to the circular buffer.
 *
 * This function:
 * - Validates the buffer length before writing.
 * - Stores **GATT handle, data length, and data** into the queue.
 * - Updates the **write pointer (`wptr`)** and **full (`Wflag`) / empty (`Rflag`)** flags.
 *
 * @param charHandle GATT handle associated with the data.
 * @param bufLength Length of the data being stored (**must be within valid range**).
 * @param data Pointer to the data buffer.
 *
 * @return **false** on success, **true** if the queue is full or input is invalid.
 */
static bool write_queue(uint16_t charHandle, uint32_t bufLength, const uint8_t *data) {
  if (bufLength < MIN_BUFFER_LENGTH || bufLength > MAX_BUFFER_LENGTH) {
    return true; // invalid length => fail
  }
  if (Wflag) {
    return true; // queue is full
  }

  uint32_t nxt = nextPtr(wptr);
  my_queue[wptr].charHandle = charHandle;
  my_queue[wptr].bufLength  = bufLength;
  memcpy(my_queue[wptr].buffer, data, bufLength);

  Rflag = false;         // definitely not empty now
  if (nxt == rptr) {
    Wflag = true;        // just became full
  }
  wptr = nxt;
  return false; // success
}

/**
 * @brief Reads an entry from the circular buffer.
 *
 * This function:
 * - **Checks if the queue is empty** before attempting a read.
 * - **Retrieves** the stored **GATT handle, data length, and data buffer**.
 * - **Updates the read pointer (`rptr`)** and **full (`Wflag`) / empty (`Rflag`) flags**.
 *
 * @param charHandle Pointer to store the retrieved GATT handle.
 * @param bufLength Pointer to store the retrieved buffer length.
 * @param data Pointer to store the retrieved data.
 *
 * @return **false** on success, **true** if the queue is empty.
 */
static bool read_queue(uint16_t *charHandle, uint32_t *bufLength, uint8_t *data) {
  if (Rflag) {
    return true; // empty
  }

  uint32_t nxt = nextPtr(rptr);
  *charHandle = my_queue[rptr].charHandle;
  *bufLength  = my_queue[rptr].bufLength;
  memcpy(data, my_queue[rptr].buffer, *bufLength);

  Wflag = false;       // not full now
  if (nxt == wptr) {
    Rflag = true;      // now empty
  }
  rptr = nxt;
  return false; // success
}

/**
 * @brief Queues an indication for transmission via GATT.
 *
 * This function:
 * - Calls `write_queue()` to store the **GATT characteristic handle and data**.
 * - **Logs an error** if the queue is **full** or **data length is invalid**.
 *
 * @param charHandle GATT handle associated with the data.
 * @param data Pointer to the indication data.
 * @param len Length of the data being stored.
 */
static void queueIndication(uint16_t charHandle, const uint8_t *data, uint32_t len)
{
  bool rc = write_queue(charHandle, len, data);
  if (rc == true) {
    // queue is full or invalid length
    // you can log or handle an error
    LOG_ERROR("Indication queue full or invalid len=%ld", len);
  }
}

/**
 * @brief Attempts to send the next queued GATT indication.
 *
 * This function:
 * - **Checks if an indication is already in flight.**
 * - **Reads the next queued indication** from the circular buffer.
 * - **Sends the indication** using `sl_bt_gatt_server_send_indication()`.
 * - **Updates `indication_in_flight` flag** on successful transmission.
 * - Logs an error if indication sending fails.
 *
 * @param connection The BLE connection handle.
 */
static void trySendNextIndication(uint8_t connection)
{
  if (ble_data.indication_in_flight) {
    return; // we must wait for confirmation
  }

  // read next from queue
  uint16_t cHandle;
  uint32_t length;
  uint8_t  buffer[MAX_BUFFER_LENGTH];

  bool rc = read_queue(&cHandle, &length, buffer);
  if (rc == true) {
    // queue is empty
    return;
  }

  // we have an entry to send
  sl_status_t sc = sl_bt_gatt_server_send_indication(
      connection,
      cHandle,
      length,
      buffer
  );
  if (sc == SL_STATUS_OK) {
      ble_data.indication_in_flight = true;
  } else {
    LOG_ERROR("send_indication fail:0x%x", sc);
    // optionally reinsert the item if you want
  }
}




/**
 * @brief Get pointer to the BLE data structure
 */
ble_data_struct_t* getBleDataPtr(void) {
    return &ble_data;
}



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
float FLOAT_TO_INT32(const uint8_t *buffer_ptr) {
    uint8_t     signByte = 0;
    // Extract mantissa
    int32_t mantissa = (int32_t) (buffer_ptr[1]  << 0)  |
        (buffer_ptr[2]  << 8)  |
        (buffer_ptr[3]  << 16) |
        (signByte       << 24) ;

    // Extract the exponent
    int8_t exponent = (int8_t)buffer_ptr[4];

    return pow(10, exponent) * mantissa;  // Return float value
}



/**
 * @brief Updates the GATT database and sends a temperature indication.
 *
 * This function converts the temperature into the **IEEE-11073** format, updates
 * the **GATT temperature measurement attribute**, and sends an indication if
 * the BLE connection allows it.
 *
 * @param temperature_c Temperature in **Celsius** as a float.
 */
void GATT_Update(float temperature_c)
{
    sl_status_t sc;

    // Temperature buffer for GATT update (5 bytes: 1 flag + 4 temperature)
    uint8_t htm_temperature_buffer[5] = {0};

    // Insert flags byte (Celsius, no timestamp, no type)
    htm_temperature_buffer[0] = 0x00;  // 0 = Celsius

    // Convert temperature to IEEE-11073 format
    uint32_t temp = convertToIEEE11073(temperature_c);

    // Copy the temperature value into the buffer
    memcpy(&htm_temperature_buffer[1], &temp, sizeof(temp));

    // Write temperature to the local GATT database
    sc = sl_bt_gatt_server_write_attribute_value(
        gattdb_temperature_measurement, // Handle from gatt_db.h
        0,  // Offset
        sizeof(htm_temperature_buffer),  // Length (5 bytes)
        htm_temperature_buffer  // Data buffer
    );

    if (sc != SL_STATUS_OK) {
        LOG_ERROR("GATT Database update failed: 0x%x", sc);
        return;  // Exit if the GATT update fails
    }

    // Ensure BLE connection is open, indications are enabled, and no indication is in flight
    if (ble_data.connection_open && ble_data.ok_to_send_htm_indications)
    {
        // Instead of calling sl_bt_gatt_server_send_indication() directly,
        // enqueue the data and then try to send from the queue.
        queueIndication(gattdb_temperature_measurement,
                        htm_temperature_buffer,
                        sizeof(htm_temperature_buffer));

        trySendNextIndication(ble_data.connection_handle);

        // For user feedback, show the temperature on the LCD
        // (Even if the indication is queued, we still can show the local reading.)
        displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp=%.2f C", temperature_c);
    }
}


/**
 * @brief Initializes Bluetooth (BLE) advertising and settings.
 *
 * This function configures the BLE stack, retrieves the device's identity address,
 * sets up an advertising set, and starts BLE advertising.
 */
void initialization_BLE(void) {
    sl_status_t status;
    displayInit();
    //display the device server/client
    displayPrintf(DISPLAY_ROW_NAME, "%s", BLE_DEVICE_TYPE_STRING);
    // Get unique Bluetooth device address
    status = sl_bt_system_get_identity_address(&ble_data.myAddress, &ble_data.myAddressType);
    if (status != SL_STATUS_OK) {
        LOG_ERROR("Failed to get BLE device address: 0x%x", status);
    } else {
        // Display Bluetooth Address in readable format
        displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                      ble_data.myAddress.addr[0], ble_data.myAddress.addr[1], ble_data.myAddress.addr[2],
                      ble_data.myAddress.addr[3], ble_data.myAddress.addr[4], ble_data.myAddress.addr[5]);
    }
    displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A8");
    // Create an advertising set
    status = sl_bt_advertiser_create_set(&ble_data.advertisingSetHandle);
    if (status != SL_STATUS_OK) {
        LOG_ERROR("BLE Advertiser creation failed: 0x%x", status);
        return;
    }

    // Generate advertising data
    status = sl_bt_legacy_advertiser_generate_data(ble_data.advertisingSetHandle,
               sl_bt_advertiser_general_discoverable);
    if (status != SL_STATUS_OK) {
        LOG_ERROR("BLE Advertiser Data Generation Failed: 0x%x", status);
        return;
    }

    // Set advertising timing (min and max Connection_interval = 250ms)
    // 1) min_interval = 400  => 400 * 0.625 ms = 250 ms
    // 2) max_interval = 400  => 400 * 0.625 ms = 250 ms
    //    (So min_interval == max_interval => we always advertise every 250 ms.)
    //
    // 3) duration = 0 => no time limit on how long we advertise
    // 4) max_events = 0 => no limit on number of advertising events
    status = sl_bt_advertiser_set_timing(ble_data.advertisingSetHandle, 400, 400, 0, 0);
    if (status != SL_STATUS_OK) {
        LOG_ERROR("BLE Advertiser timing failed: 0x%x", status);
        return;
    }

    // Start BLE legacy advertising
    status = sl_bt_legacy_advertiser_start(ble_data.advertisingSetHandle,
                   sl_bt_legacy_advertiser_connectable);
    if (status != SL_STATUS_OK) {
        LOG_ERROR("BLE Legacy Advertising Start Failed: 0x%x", status);
        return;
    }
    displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
}

/**
 * @brief Configures BLE scanning and starts discovery.
 *
 * This function:
 * 1. Sets default connection parameters (~75ms interval, 4 latency, 8.3s supervision timeout).
 * 2. Configures passive scanning parameters (50ms interval, 25ms window).
 * 3. Starts BLE scanning on 1M PHY in generic discovery mode.
 */
void client_scan(void){
  sl_status_t status;
  // 1) min_interval = 60
  //    - Specified in 1.25 ms increments.
  //    - 60 × 1.25 ms = 75 ms (minimum connection interval).
  //
  // 2) max_interval = 60
  //    - Also 75 ms (maximum connection interval).
  //
  // 3) slave_latency = 4
  //    - The slave (your device) can skip up to 4 connection intervals if it has no data.
  //
  // 4) supervision_timeout = 83
  //    - Specified in units of 10 ms.
  //    - greater than (1*latency) * (interval *2)
  //
  // 5) min_ce_length = 0
  //
  // 6) max_ce_length = 4
  status = sl_bt_connection_set_default_parameters(
          60,  // min_interval  (60 * 1.25 ms = 75 ms)
          60,  // max_interval  (same = 75 ms)
          4,   // slave latency
          83, // supervision timeout
          0,   // min CE length
          4    // max CE length
        );
    if (status != SL_STATUS_OK) {
      LOG_ERROR("set_default_params failed: 0x%x", status);
    }
    // 2) Configure scanning parameters
    //    - phy = sl_bt_gap_1m_phy (standard 1M PHY)
    //    - scan_type = sl_bt_scanner_scan_mode_passive for passive scanning
    //    - interval = 50 ms / 0.625 ms = 80
    //    - window  = 25 ms / 0.625 ms = 40
    //    - disc_mode is typically ignored here (set to 0),
    //      because the actual discovery mode is specified in sl_bt_scanner_start().
    status = sl_bt_scanner_set_parameters(
      sl_bt_scanner_scan_mode_passive,  // mode: passive
      80,                               // interval = 50 ms / 0.625 ms
      40                                // window  = 25 ms / 0.625 ms
    );
      if (status != SL_STATUS_OK) {
        LOG_ERROR("scanner_set_parameters failed: 0x%x", status);
      }
      //start the scannner
      status = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
        if (status != SL_STATUS_OK) {
          LOG_ERROR("scanner_start failed: 0x%x", status);
          return;
        }
        displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering");
}

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
void report_id(sl_bt_msg_t *evt){

  // Extract the advertiser’s address from the event
  bd_addr foundAddr = evt->data.evt_scanner_legacy_advertisement_report.address;
  uint8_t addrType  = evt->data.evt_scanner_legacy_advertisement_report.address_type;

  // Compare with the server address we want
  if (memcmp(foundAddr.addr, server_addr.addr, 6) == 0)
  {

    // 1) Stop scanning
    sl_status_t sc = sl_bt_scanner_stop();
    if (sc != SL_STATUS_OK) {
      LOG_ERROR("scanner_stop failed: 0x%x", sc);
    }

    // 2) Open a connection to this device
    //    Use 1M PHY or whichever PHY you prefer
    sc = sl_bt_connection_open(foundAddr, addrType, sl_bt_gap_1m_phy, NULL);
    if (sc != SL_STATUS_OK) {
      LOG_ERROR("connection_open failed: 0x%x", sc);
    }
  }

}

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
static void update_button_state(uint8_t newValue)
{
  sl_status_t sc = sl_bt_gatt_server_write_attribute_value(
      gattdb_button_state,   // The handle for your custom "button_state" characteristic
      0,                     // offset
      sizeof(newValue),
      &newValue
  );
  if (sc != SL_STATUS_OK) {
    LOG_ERROR("update_button_state fail: 0x%x", sc);
  }
}

/**
 * @brief Handles BLE events for server and client
 *
 * @param evt Pointer to the received Bluetooth event (`sl_bt_msg_t`).
 */
void handle_ble_event(sl_bt_msg_t *evt) {

    sl_status_t status; //status check
    sl_status_t sc; //status check

    //use this for connection parameter id
    //uint16_t Connection_interval;
    //uint16_t slave_latency;
    //uint16_t supervision;

//server code starts here
#if DEVICE_IS_BLE_SERVER
   uint8_t status_flags = evt->data.evt_gatt_server_characteristic_status.status_flags;
   uint16_t client_config_flags = evt->data.evt_gatt_server_characteristic_status.client_config_flags;
   uint16_t characteristic = evt->data.evt_gatt_server_characteristic_status.characteristic;

    switch (SL_BT_MSG_ID(evt->header)) {


        // System Boot Event - Start BLE Advertising
        case sl_bt_evt_system_boot_id:
            // 1) Delete any old bondings (forces re-pair each time)
            status = sl_bt_sm_delete_bondings();
            if (status != SL_STATUS_OK) {
              LOG_ERROR("delete_bondings fail:0x%x", status);
            }

            // 2) Configure the security manager
            //    For numeric comparison with MITM + bonding, flags often = 0x0C
            //    Check docs for the correct bits for your scenario
            status = sl_bt_sm_configure(MY_SM_CONFIG_FLAGS, sl_bt_sm_io_capability_displayyesno);
            if (status != SL_STATUS_OK) {
              LOG_ERROR("sm_configure fail:0x%x", status);
            }

            //initialize the BLE
            initialization_BLE();

            //display the initial state
            update_button_state(release);
            displayPrintf(DISPLAY_ROW_9, "Button released");


            break;

        // Connection Opened Event - Stop Advertising
        case sl_bt_evt_connection_opened_id:
            ble_data.connection_open = true;
            bonded = false;
            ble_data.connection_handle = evt->data.evt_connection_opened.connection;
            displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");

            // Stop advertising once a connection is established
            status = sl_bt_advertiser_stop(ble_data.advertisingSetHandle);
            if (status != SL_STATUS_OK) {
                LOG_ERROR("BLE Advertiser Stop Failed: 0x%x", status);
            }

            // Set connection parameters (including event lengths)
            status = sl_bt_connection_set_parameters(
                ble_data.connection_handle,  // Connection Handle
                60,  // Min Connection_interval (75ms) -> 60 * 1.25 ms = 75
                60,  // Max Connection_interval (75ms) -> 60 * 1.25 ms = 75
                4,    // Slave latency (300ms off-air)
                76,  // Supervision timeout -> (1+4) * (75ms * 2) = 750
                0,    // Min connection event length (use default)
                65535 // Max connection event length (use max allowed)
            );
            if (status != SL_STATUS_OK) {
                LOG_ERROR("BLE Connection Parameters Set Failed: 0x%x", status);
            }

            break;

        // Connection Closed Event - Restart Advertising
        case sl_bt_evt_connection_closed_id:
            ble_data.connection_open = false;
            //turn off indications
            ble_data.ok_to_send_htm_indications = false;
            ble_data.ok_to_send_btn_indications = false;
            ble_data.indication_in_flight = false;
            ble_data.connection_handle = 0xFF;  // Reset connection handle
            waitingForPasskeyConfirm = false;
            bonded = false;
            //turn off led while closing
            gpioLed1SetOff();
            gpioLed0SetOff();

            // Delete bondings on close (like we do on boot)
            status = sl_bt_sm_delete_bondings();
            if (status != SL_STATUS_OK) {
              LOG_ERROR("delete_bondings fail:0x%x", status);
            }

            // Restart BLE legacy advertising
            status = sl_bt_legacy_advertiser_start(ble_data.advertisingSetHandle, sl_bt_legacy_advertiser_connectable);
            if (status != SL_STATUS_OK) {
                LOG_ERROR("BLE Advertising Restart Failed: 0x%x", status);
            }
            displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
            displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
            reset_queue();
            break;

        // Connection Parameters Updated Event
        case sl_bt_evt_connection_parameters_id:
          //displayPrintf(DISPLAY_ROW_ACTION, "Params Updated");
          //uncomment the below code to check for parameter update

          /*Connection_interval = evt->data.evt_connection_parameters.interval; // Connection Connection_interval
          slave_latency = evt->data.evt_connection_parameters.latency;   // Slave latency
          supervision = evt->data.evt_connection_parameters.timeout;   // Supervision timeout

          LOG_INFO("BLE Connection Parameters Updated:");
          LOG_INFO("\r\n Connection_interval: %d * 1.25ms = %d ms \r\n", Connection_interval, (Connection_interval * 125) / 100);
          LOG_INFO("slave_latency: %d \r\n", slave_latency);
          LOG_INFO("Supervision: %d / 10ms = %d ms \r\n", supervision, supervision / 10);*/
            break;

        // Handle Indication Confirmation from Client
        case sl_bt_evt_gatt_server_characteristic_status_id:
            if ( status_flags == sl_bt_gatt_server_client_config) {
                if (characteristic == gattdb_temperature_measurement) {
                    //check for HTM
                  if (client_config_flags & sl_bt_gatt_indication) {
                    ble_data.ok_to_send_htm_indications = true;
                    gpioLed0SetOn();   // LED0 on if HTM indications are enabled
                  } else {
                    ble_data.ok_to_send_htm_indications = false;
                    gpioLed0SetOff();  // LED0 off if disabled
                    displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
                  }
                  //Check for button
                }else if (characteristic == gattdb_button_state) {
                    if (client_config_flags & sl_bt_gatt_indication) {
                      ble_data.ok_to_send_btn_indications = true;
                      gpioLed1SetOn();   // LED1 on if button indications are enabled
                    } else {
                      ble_data.ok_to_send_btn_indications = false;
                      gpioLed1SetOff();  // LED1 off if disabled
                    }
                  }

            }else if (status_flags == sl_bt_gatt_server_confirmation) {
                // Client confirmed receipt of indication
                ble_data.indication_in_flight = false;
                //send next indication
                trySendNextIndication(ble_data.connection_handle);
            }
            break;
        case sl_bt_evt_gatt_server_indication_timeout_id:
            //LOG_ERROR("Indication Timeout Occurred");
            ble_data.indication_in_flight = false;
            break;
        //to handle 1Hz LCD update
        case sl_bt_evt_system_soft_timer_id:
          displayUpdate(); //LCD update function
          break;

          // ----------------------------------------------------------------
          // Security manager events for numeric comparison
          // ----------------------------------------------------------------
          case sl_bt_evt_sm_confirm_bonding_id:
          {
            // The phone is asking to bond; confirm it
            sc = sl_bt_sm_bonding_confirm(evt->data.evt_sm_confirm_bonding.connection, 1);
            if (sc != SL_STATUS_OK) {
              LOG_ERROR("bonding_confirm fail:0x%x", sc);
            }
          }
          break;

          case sl_bt_evt_sm_confirm_passkey_id:
          {
            // The phone wants numeric comparison
            uint32_t passkey = evt->data.evt_sm_confirm_passkey.passkey;
            displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey %06lu", passkey);
            displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");
            waitingForPasskeyConfirm = true;  // confirm on PB0 press
          }
          break;

          case sl_bt_evt_sm_bonded_id:
          {
            // The link is now bonded
            bonded = true;
            displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
            // Clear passkey lines if any
            displayPrintf(DISPLAY_ROW_PASSKEY, "");
            displayPrintf(DISPLAY_ROW_ACTION, "");
            waitingForPasskeyConfirm = false;
          }
          break;

          case sl_bt_evt_sm_bonding_failed_id:
          {
            //if bonding failed
            displayPrintf(DISPLAY_ROW_CONNECTION, "BondFail");
            waitingForPasskeyConfirm = false;
            //CLear the pending bonding.
            status = sl_bt_sm_delete_bondings();
            if (status != SL_STATUS_OK) {
              LOG_ERROR("delete_bondings fail:0x%x", status);
            }
          }
          break;

          // ----------------------------------------------------------------
          // External signals from PB0 press or release
          // Step 4 code + step 5 passkey confirm
          // ----------------------------------------------------------------
          case sl_bt_evt_system_external_signal_id:
          {
            uint32_t signals = evt->data.evt_system_external_signal.extsignals;

            // If PB0 is pressed
            if (signals == EVENT_PB0_PRESS) {

              // Step 5: If we're waiting for passkey confirm
              if (waitingForPasskeyConfirm) {
                // Confirm passkey
                sc = sl_bt_sm_passkey_confirm(ble_data.connection_handle, 1);
                if (sc != SL_STATUS_OK) {
                  LOG_ERROR("passkey_confirm fail:0x%x", sc);
                }
                // Clear passkey lines
                displayPrintf(DISPLAY_ROW_PASSKEY, "");
                displayPrintf(DISPLAY_ROW_ACTION, "");
                waitingForPasskeyConfirm = false;
              }
              else {
                // Normal button pressed logic (Assignment 4):
                // e.g., write 0x01 to button_state in local GATT
                update_button_state(press);  // your function from step 4
                displayPrintf(DISPLAY_ROW_9, "Button Pressed");
                if (bonded && ble_data.ok_to_send_btn_indications){
                    // If bonded & button indications are enabled:
                    queueIndication(gattdb_button_state, &press, 1);
                    //send indications
                    trySendNextIndication(ble_data.connection_handle);
                }
              }
            }

            // If PB0 is released
            if (signals == EVENT_PB0_RELEASE) {
                // Normal release logic
                update_button_state(release);
                displayPrintf(DISPLAY_ROW_9, "Button Released");
                if (bonded && ble_data.ok_to_send_btn_indications){
                    // If bonded & button indications are enabled:
                    queueIndication(gattdb_button_state, &release, 1);
                    //send indications
                    trySendNextIndication(ble_data.connection_handle);
                }
            }
          }
          break;

        default:
            break;
    }
#else

//client code starts here
    uint8_t flags;
    uint16_t length;
    uint16_t charHandle;
    switch (SL_BT_MSG_ID(evt->header)) {

      // ------------------------------------------------------
      // System Boot Event - Initialize Display and Start Scanning
      // ------------------------------------------------------
        case sl_bt_evt_system_boot_id:

          displayInit();
          displayPrintf(DISPLAY_ROW_NAME, "%s", BLE_DEVICE_TYPE_STRING);
          status = sl_bt_system_get_identity_address(&ble_data.myAddress, &ble_data.myAddressType);
          if (status != SL_STATUS_OK) {
              LOG_ERROR("Failed to get BLE device address: 0x%x", status);
          } else {
              // Display Bluetooth Address in readable format
              displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",
                            ble_data.myAddress.addr[0], ble_data.myAddress.addr[1], ble_data.myAddress.addr[2],
                            ble_data.myAddress.addr[3], ble_data.myAddress.addr[4], ble_data.myAddress.addr[5]);
          }
          displayPrintf(DISPLAY_ROW_ASSIGNMENT, "A7");
          client_scan();

          break;


          // ------------------------------------------------------
          // Advertisement Report - Check and Connect to Server
          // ------------------------------------------------------
        case sl_bt_evt_scanner_legacy_advertisement_report_id:

          // check for connectable and scannable id
          flags = evt->data.evt_scanner_legacy_advertisement_report.event_flags;
          if (flags == report_id_check){
          report_id(evt);
          }

          break;

          // ------------------------------------------------------
          // Connection Opened - Update Display and Store Connection Handle
          // ------------------------------------------------------
        case sl_bt_evt_connection_opened_id:
          displayPrintf(DISPLAY_ROW_BTADDR2, "%02X:%02X:%02X:%02X:%02X:%02X",
              server_addr.addr[0],
              server_addr.addr[1],
              server_addr.addr[2],
              server_addr.addr[3],
              server_addr.addr[4],
              server_addr.addr[5]);

          displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
          ble_data.connection_handle = evt->data.evt_connection_opened.connection;

          break;

          // ------------------------------------------------------
          // GATT Service Discovery - Store HTM Service Handle
          // ------------------------------------------------------
        case sl_bt_evt_gatt_service_id:
        {
          // Check if this is the 16-bit HTM Service (0x1809)
          // The event structure provides uuid.len and uuid.data[]
          if ((evt->data.evt_gatt_service.uuid.data[0] == lsb) &&  // LSB
              (evt->data.evt_gatt_service.uuid.data[1] == msb))    // MSB
          {
            // It's the Health Thermometer service
              ble_data.ServiceHandle = evt->data.evt_gatt_service.service;
          }
        }

        break;


        // ------------------------------------------------------
        // GATT Characteristic Discovery - Store Temperature Char Handle
        // ------------------------------------------------------
        case sl_bt_evt_gatt_characteristic_id:

          // Check if this is the 16-bit Temperature Measurement Characteristic (0x2A1C)
          if ((evt->data.evt_gatt_characteristic.uuid.data[0] == Clsb) && // LSB
              (evt->data.evt_gatt_characteristic.uuid.data[1] == Cmsb))   // MSB
          {
            // It's the Temperature Measurement characteristic
              ble_data.CharHandle = evt->data.evt_gatt_characteristic.characteristic;
          }

         break;

         // ------------------------------------------------------
         // GATT Characteristic Value - Handle Temperature Indications
         // ------------------------------------------------------
        case sl_bt_evt_gatt_characteristic_value_id:

          displayPrintf(DISPLAY_ROW_CONNECTION, "Handling Indications");
          charHandle = evt->data.evt_gatt_characteristic_value.characteristic;
          length     = evt->data.evt_gatt_characteristic_value.value.len;

          // Check if this is the HTM (Temperature Measurement) characteristic handle
          if (charHandle == ble_data.CharHandle){
            // 1) Parse the indicated value
            //    Typically, for HTM, we have 5 bytes: [0]=flags, [1..3]=mantissa, [4]=exponent
            if (length >= 5) {
              // Convert from IEEE-11073 format
              // 2) Display the temp value.
              displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp=%.2fC",
                            FLOAT_TO_INT32(evt->data.evt_gatt_characteristic_value.value.data));
            } else {
              LOG_ERROR("HTM Indication too short: %d bytes", length);
            }

            // 3) Acknowledge the indication
            sl_status_t sc = sl_bt_gatt_send_characteristic_confirmation(
                                evt->data.evt_gatt_characteristic_value.connection);
            if (sc != SL_STATUS_OK) {
              LOG_ERROR("Confirm indication failed: 0x%x", sc);
            }
          }

        break;


        // ------------------------------------------------------
        // Soft Timer Event - Update Display Periodically
        // ------------------------------------------------------
        case sl_bt_evt_system_soft_timer_id:
          displayUpdate(); //LCD update function
          break;

        default:
            break;
    }
#endif
}


