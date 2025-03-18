/*
 * ble_device_type.h
 *
 *  Created on: Feb 16, 2019
 *      Author: dan walkes
 *
 *      Editor: Mar 14, 2021, Dave Sluiter
 *      Change: Added some vertical white space and bd_addr array indices
 *
 * Editor: Feb 26, 2022, Dave Sluiter
 * Change: Added comment about use of .h files.
 *
 *
 * Student edit: Add your name and email address here:
 * @student    Awesome Student, Awesome.Student@Colorado.edu
 *
 */

// Students: Remember, a header file (a .h file) generally defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.

#ifndef SRC_BLE_DEVICE_TYPE_H_
#define SRC_BLE_DEVICE_TYPE_H_
#include <stdbool.h>

/*
 * Set to 1 to configure this build as a BLE server.
 * Set to 0 to configure as a BLE client
 */
#define DEVICE_IS_BLE_SERVER 1


//Server address hard coded.
#define SERVER_BT_ADDRESS {{ 0x76, 0x6D, 0xF8, 0x50, 0x4B, 0x80 }}
// This also can work:
//#define SERVER_BT_ADDRESS (bd_addr) { .addr = { 0x85, 0x61, 0x17, 0x57, 0x0b, 0x00 } }



#if DEVICE_IS_BLE_SERVER

#define BUILD_INCLUDES_BLE_SERVER 1
#define BUILD_INCLUDES_BLE_CLIENT 0
#define BLE_DEVICE_TYPE_STRING "Server"
static inline bool IsServerDevice() { return true; }
static inline bool IsClientDevice() { return false; }

#else

#define BUILD_INCLUDES_BLE_SERVER 0
#define BUILD_INCLUDES_BLE_CLIENT 1
#define BLE_DEVICE_TYPE_STRING "Client"
static inline bool IsClientDevice() { return true;}
static inline bool IsServerDevice() { return false; }

#endif

#endif /* SRC_BLE_DEVICE_TYPE_H_ */
