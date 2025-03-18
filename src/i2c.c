/***********************************************************************
 * @file      i2c.c
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
 * okokok
 * @resources  Utilized Silicon Labs' Peripheral Libraries (em_letimer.h)
 *             and Course slides
 *
 * @note This module provides functions for initializing and communicating
 *       with the Si7021 temperature sensor using I2C.
 *
 ***********************************************************************/
#include "i2c.h"
#include "scheduler.h"
#include "timers.h"
#include "gpio.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "sl_i2cspm.h"


// ---------------------------------------------------------------------
// Static Variables
// ---------------------------------------------------------------------

static I2C_TransferSeq_TypeDef transfer; //I2C transfer sequence
static uint8_t buffer_for_temp[2];  // Buffer to hold temperature data

// ---------------------------------------------------------------------
// Function Definitions
// ---------------------------------------------------------------------

/**
 * @brief Initializes I2C communication for the Si7021 sensor.
 *
 * This function configures the I2C peripheral using predefined GPIO pins and
 * initializes the I2C bus for communication at standard speed (100 kHz).
 */
void I2C_init(void)
{

    I2CSPM_Init_TypeDef i2cInit = {
        .port = I2C0,
        .sclPort = gpioPortC,
        .sclPin = 10,
        .sdaPort = gpioPortC,
        .sdaPin = 11,
        .portLocationScl = 14,
        .portLocationSda = 16,
        .i2cRefFreq = 0,  // default reference clock
        .i2cMaxFreq = I2C_FREQ_STANDARD_MAX,  // Standard I2C speed (100 kHz)
        .i2cClhr = i2cClockHLRStandard,
    };

    I2CSPM_Init(&i2cInit);
} //I2C_init()


/**
 * @brief Powers on the Si7021 sensor.
 *
 * This function enables the sensor power by calling `sensorEnable()`.
 */
void Si7021_PowerOn(void)
{
    sensorEnable();
}//Si7021_PowerOn()

/**
 * @brief Powers off the Si7021 sensor.
 *
 * This function disables the sensor power by calling `sensorDisable()`.
 */
void Si7021_PowerOff(void)
{
    sensorDisable();
} //Si7021_PowerOn()

/**
 * @brief Initiates temperature measurement on Si7021 using I2C (Non-Blocking).
 *
 * This function sends a measurement command to the Si7021 sensor. It enables
 * the I2C interrupt and uses `I2C_TransferInit` for non-blocking communication.
 */
void Si7021_StartTempMeasurement(void)
{
     static uint8_t command = Measure_command; //No hold command
     //Creating the sequence
     transfer.addr = SI7021_I2C_ADDR << 1;
     transfer.flags = I2C_FLAG_WRITE;
     transfer.buf[0].data = &command;
     transfer.buf[0].len = sizeof(command);
     //enabling interrupts
     NVIC_ClearPendingIRQ(I2C0_IRQn);
     NVIC_EnableIRQ(I2C0_IRQn);
     //LOG_INFO("S");
     if (I2C_TransferInit(I2C0, &transfer) != i2cTransferInProgress) {
        LOG_ERROR("Transfer Failed: Write");
     }
} //Si7021_StartTempMeasurement()

/**
 * @brief Reads temperature data from Si7021 using I2C (Non-Blocking).
 *
 * This function sets up a read transfer for 2 bytes of temperature data. It
 * enables the I2C interrupt and initiates non-blocking communication.
 */
void Si7021_ReadTemperature(void)
{
      //Creating the sequence
      //LOG_INFO("A");
      transfer.addr = SI7021_I2C_ADDR << 1;
      transfer.flags = I2C_FLAG_READ;
      transfer.buf[0].data = buffer_for_temp;
      transfer.buf[0].len = sizeof(buffer_for_temp);
      //enabling interrupts
      NVIC_ClearPendingIRQ(I2C0_IRQn);
      NVIC_EnableIRQ(I2C0_IRQn);
     if (I2C_TransferInit(I2C0, &transfer) != i2cTransferInProgress) {
            LOG_ERROR("Transfer Failed: Read");
     }
} //Si7021_ReadTemperature()

/**
 * @brief Converts and prints the measured temperature in Celsius.
 *
 * This function reads the raw temperature data from the global buffer, converts
 * it to Celsius using `raw_to_celcius()`, and logs the result.
 */
void print_temperature(void)
{
  uint16_t Register_value = (buffer_for_temp[0] << 8) | buffer_for_temp[1]; // Combine the two bytes
  int16_t celsius = raw_to_celcius(Register_value); //macro
  LOG_INFO("Temperature: %" PRId32 " ' C\r\n", (int32_t)celsius);
} //print_temperature()


/**
 * @brief Retrieves the temperature value from the Si7021 sensor.
 *
 * This function processes the **raw temperature data** stored in `buffer_for_temp`,
 * converts it to **Celsius**, and returns the result.
 *
 * @return Temperature in Celsius as a floating-point value.
 */
float Si7021_GetTemperature(){

  // Combine two 8-bit temperature bytes into a 16-bit raw value
  uint16_t Register_value = (buffer_for_temp[0] << 8) | buffer_for_temp[1];

  // Convert the raw value to Celsius using the provided macro
  float celsius = raw_to_celcius(Register_value);
  return celsius;
}

