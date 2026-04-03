/**
 * @file Hardware.h
 * @brief Hardware Abstraction Layer for the eChook Telemetry Board.
 *
 * This file defines the interface for hardware-specific functions that are
 * implemented differently depending on the target microcontroller architecture
 * (e.g. ATmega328P vs ATmega4809).
 */

#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

/**
 * @brief Initialize hardware-specific serial communications.
 */
void hardwareSerialSetup();

/**
 * @brief Print a setup completion message with the current uptime.
 */
void hardwarePrintSetupComplete();

/**
 * @brief Attach hardware-specific interrupts for speed sensing.
 */
void hardwareAttachInterrupts();

/**
 * @brief Calculates and returns the internal temperature of the microcontroller.
 * @return The internal temperature in Celsius.
 */
float hardwareReadTempInternal();

/**
 * @brief Updates and returns the reference voltage for ADC calculations.
 * @return The calculated reference voltage.
 */
float hardwareUpdateReferenceVoltage();

/**
 * @brief Write hardware-specific configuration data to the Serial port.
 * @param identifier The data identifier
 * @param dataByte1 First byte of data
 * @param dataByte2 Second byte of data
 */
void hardwareSerialWriteConfig(char identifier, byte dataByte1, byte dataByte2);

/**
 * @brief Write a string to the debugging serial port.
 * @param text The text to print.
 */
void hardwareSerialPrint(const char *text);
void hardwareSerialPrint(const __FlashStringHelper *text);

/**
 * @brief Write a line to the debugging serial port.
 * @param text The text to print.
 */
void hardwareSerialPrintln(const char *text);
void hardwareSerialPrintln(const __FlashStringHelper *text);

#endif // HARDWARE_H
