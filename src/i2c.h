/***********************************************************************
 * @file      i2c.h
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
 * @note This module provides functions for initializing and communicating
 *       with the Si7021 temperature sensor using I2C.
 *
 ***********************************************************************/
#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "em_i2c.h"

// ---------------------------------------------------------------------
// Macro Definitions
// ---------------------------------------------------------------------
/** @brief I2C address of the Si7021 sensor. */
#define SI7021_I2C_ADDR        (0x40)   // I2C address of Si7021
#define Measure_command   (0xF3)   // Command to measure temperature


//macro to convert raw data to celcius
#define raw_to_celcius(temp) (((175.72 * temp) / 65536) - 46.85)


// ---------------------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------------------

/**
 * @brief Initializes the I2C peripheral.
 *
 * This function configures **I2C0** for communication with the **Si7021**
 * temperature sensor using the predefined `I2C_Config` settings.
 */
void I2C_init(void);

/**
 * @brief Powers on the Si7021 sensor.
 *
 * This function enables the **sensor power**, waits for **80ms** for
 * stabilization before measurements can be taken.
 */
void Si7021_PowerOn(void);

/**
 * @brief Powers off the Si7021 sensor.
 *
 * Disables sensor power to **reduce energy consumption** when not in use.
 */
void Si7021_PowerOff(void);

/**
 * @brief Initiates a temperature measurement on the Si7021 sensor.
 *
 * This function sends the **0xF3 command** to request a **temperature measurement**.
 * The measurement is performed in **No Hold Mode**, allowing non-blocking operation.
 */
void Si7021_StartTempMeasurement(void);

/**
 * @brief Reads the temperature value from the Si7021 sensor.
 *
 * This function retrieves the **16-bit raw temperature data** over I2C and
 * returns it as an **integer**. The raw value must be converted to **Celsius**.
 *
 * @return Raw temperature data (to be converted to Celsius).
 */
void Si7021_ReadTemperature(void);

/**
 * @brief Converts and logs the measured temperature in Celsius.
 *
 * This function reads the raw temperature data, converts it to Celsius,
 * and logs the result using the `LOG_INFO` macro.
 */
void print_temperature(void);

/**
 * @brief Retrieves the temperature value from the Si7021 sensor.
 *
 * This function processes the **raw temperature data** stored in `buffer_for_temp`,
 * converts it to **Celsius**, and returns the result.
 *
 * @return Temperature in Celsius as a floating-point value.
 */
float Si7021_GetTemperature();
#endif
