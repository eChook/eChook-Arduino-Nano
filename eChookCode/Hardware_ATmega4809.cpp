/**
 * @file Hardware_ATmega4809.cpp
 * @brief ATmega4809 (Arduino Nano Every) specific hardware implementations.
 */

#if defined(__AVR_ATmega4809__)

#include "Calibration.h"
#include "Globals.h"
#include "Hardware.h"
#include "Pinout.h"
#include <Arduino.h>

extern void motorSpeedISR();
extern void wheelSpeedISR();
extern const float CODE_VERSION;
extern int inConfig;

void hardwareSerialSetup() {
  // Starts USB Serial as well on Arduino Nano Every at the same baud rate as
  // set for Bluetooth
  Serial.begin(CAL_BT_BAUDRATE);
  Serial.println(("\n\n\neChook Nano Starting Setup"));
  Serial.print(("Firmware Version: "));
  Serial.println(CODE_VERSION);
}

void hardwarePrintSetupComplete() {
  Serial.print(("\nSetup complete in "));
  Serial.print(millis());
  Serial.println(("ms."));
}

void hardwareAttachInterrupts() {
  attachInterrupt(2, motorSpeedISR, RISING);
  attachInterrupt(3, wheelSpeedISR, RISING);
}

float hardwareReadTempInternal() {
  // Arduino Nano Every: Read internal temperature sensor
  // Based on ATmega4809 datasheet and Arduino core implementation
  uint16_t adc;
  float temperature;

  // Save current ADC settings
  uint8_t oldSAMPCTRL = ADC0.SAMPCTRL;
  uint8_t oldCTRLC = ADC0.CTRLC;
  uint8_t oldMUXPOS = ADC0.MUXPOS;

  // Configure ADC for internal temperature sensor
  ADC0.CTRLC = ADC_PRESC_DIV4_gc | ADC_REFSEL_INTREF_gc; // 1.1V reference
  ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;                 // Select temperature sensor
  ADC0.SAMPCTRL = 0x3F;                                  // Maximum sampling time

  ADC0.COMMAND = ADC_STCONV_bm; // Start conversion
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
    ; // Wait for result ready

  adc = ADC0.RES; // Read ADC result

  // Restore ADC settings
  ADC0.SAMPCTRL = oldSAMPCTRL;
  ADC0.CTRLC = oldCTRLC;
  ADC0.MUXPOS = oldMUXPOS;

  // Convert ADC value to temperature (approximate, not factory calibrated)
  // Formula from ATmega4809 datasheet, section 27.3.1
  // Typical: 1 LSB = 1°C, 300 = 25°C
  temperature = (float)adc - 300.0f;
  temperature += 25.0f;

  return temperature; // Note: This is an approximate value and may need calibration for accuracy.
}

float hardwareUpdateReferenceVoltage() {
  // TODO - implement properly for Arduino Nano Every
  return CAL_REFERENCE_VOLTAGE;
}

void hardwareSerialWriteConfig(char identifier, byte dataByte1, byte dataByte2) {
  if (inConfig) {
    Serial.write(123);
    Serial.write(identifier);
    Serial.write(dataByte1);
    Serial.write(dataByte2);
    Serial.write(125);
  }
}

void hardwareSerialPrint(const String &text) {
  Serial.print(text);
}

void hardwareSerialPrintln(const String &text) {
  Serial.println(text);
}

#endif // __AVR_ATmega4809__
