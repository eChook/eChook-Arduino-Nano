/**
 * @file Hardware_ATmega328P.cpp
 * @brief ATmega328P (Original Arduino Nano) specific hardware implementations.
 */

#if defined(__AVR_ATmega328P__)

#include "Calibration.h"
#include "Globals.h"
#include "Hardware.h"
#include "Pinout.h"
#include <Arduino.h>

extern void motorSpeedISR();
extern void wheelSpeedISR();

void hardwareSerialSetup() {
  // The original Nano uses the same Serial port for USB and Bluetooth
  // We only need to initialize SerialA here, which is aliased to Serial in eChookCode.ino
}

void hardwarePrintSetupComplete() {
  // Original Nano didn't print this
}

void hardwareAttachInterrupts() {
  attachInterrupt(0, motorSpeedISR, RISING);
  attachInterrupt(1, wheelSpeedISR, RISING);
}

// Reading the interanl arduino tempreature - notes
// on the accuracy and calibration here:
// https://playground.arduino.cc/Main/InternalTemperatureSensor/
float hardwareReadTempInternal() {
  unsigned int wADC;
  float t;
  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.
  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN); // enable the ADC
  delay(20);           // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC); // Start the ADC
  // Detect end-of-conversion
  while (bit_is_set(ADCSRA, ADSC))
    ;
  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;
  // The offset is specific to each device. This is a general figure
  t = (wADC - 324.31) / 1.22;
  // The returned temperature is in degrees Celsius.
  // Measured offset
  t = t - 7;
  return (t > 0 ? t : 0);
}

float hardwareUpdateReferenceVoltage() {
  // This function uses the internal 1v1 reference to back calucalate the 5V
  // rail voltage. It measures the stable reference voltage, using the 5V rail
  // as the ADC reference, then uses the result to calculate an accurate value
  // for the 5V ADC reference.

  // Set the analog reference to DEFAULT (AVcc == Vcc power rail)
  // REFS1 REFS0          --> 0b01   -Selects DEFAULT (AVcc) reference
  // Set the analog input to channel 14: the INTERNAL bandgap reference (1.1V
  // +/- 10%) MUX3 MUX2 MUX1 MUX0  --> 0b1110 -Selects channel 14, bandgap
  // voltage, to measure
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) |
          (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

  delay(2); // Let mux settle a little to get a more stable A/D conversion

  // Start a conversion to measure the INTERNAL reference relative to the
  // DEFAULT (Vcc) reference.
  ADCSRA |= _BV(ADSC);
  // Wait for it to complete
  while (ADCSRA & (1 << ADSC)) {
  };

  // Calculate the power rail voltage (reference voltage) relative to the known
  // voltage
  return (float)((CAL_INTERNAL_REFERENCE_VOLTAGE * 1024UL) / ADC);
}

void hardwareSerialWriteConfig(char identifier, byte dataByte1, byte dataByte2) {
  // OG Nano doesn't do anything extra here
}

void hardwareSerialPrint(const char *text) {
  // Not needed on standard Nano as debug goes out via SerialA anyway
}

void hardwareSerialPrint(const __FlashStringHelper *text) {
  // Not needed on standard Nano as debug goes out via SerialA anyway
}

void hardwareSerialPrintln(const char *text) {
  // Not needed on standard Nano as debug goes out via SerialA anyway
}

void hardwareSerialPrintln(const __FlashStringHelper *text) {
  // Not needed on standard Nano as debug goes out via SerialA anyway
}

#endif // __AVR_ATmega328P__
