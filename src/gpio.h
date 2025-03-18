/*
   gpio.h
  
    Created on: Dec 12, 2018
        Author: Dan Walkes

    Updated by Dave Sluiter Sept 7, 2020. moved #defines from .c to .h file.
    Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

    Editor: Feb 26, 2022, Dave Sluiter
    Change: Added comment about use of .h files.

 *
 * Student edit: Add your name and email address here:
 * @student    Lokesh Senthil Kumar, lokesh.senthilkumar@Colorado.edu
 *
 
 */


// Students: Remember, a header file (a .h file) generally defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.


#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_

#include "em_gpio.h"

#define PB0_PORT  gpioPortF
#define PB0_PIN   6
// Function prototypes

/*
 * @brief Initializes GPIO pins and sets their drive strengths and modes of operation.
 *
 * This function configures the GPIO drive strength and mode for the LEDs connected to
 * the specified ports and pins. The drive strength is set to weak (1mA) for all pins
 * in the LED port.
 */
void gpioInit();

/*
 * @brief Turns ON LED 0.
 *
 * This function sets the GPIO pin corresponding to LED 0 to a high state,
 * turning the LED ON.
 */
void gpioLed0SetOn();

/*
 * @brief Turns OFF LED 0.
 *
 * This function clears the GPIO pin corresponding to LED 0,
 * turning the LED OFF.
 */
void gpioLed0SetOff();

/*
 * @brief Turns ON LED 1.
 *
 * This function sets the GPIO pin corresponding to LED 1 to a high state,
 * turning the LED ON.
 */
void gpioLed1SetOn();

/*
 * @brief Turns OFF LED 1.
 *
 * This function clears the GPIO pin corresponding to LED 1,
 * turning the LED OFF.
 */
void gpioLed1SetOff();


void sensorEnable();

void sensorDisable();

void gpioSetDisplayExtcomin(bool value);

#endif /* SRC_GPIO_H_ */
