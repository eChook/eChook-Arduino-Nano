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
// extern int inConfig;

void hardwareSerialSetup() {
  // Starts USB Serial as well on Arduino Nano Every at the same baud rate as
  // set for Bluetooth
  Serial.begin(CAL_BT_BAUDRATE);
  Serial.println(("\n\n\neChook Nano Starting Setup"));
}

void hardwarePrintSetupComplete() {
  Serial.print(("\nSetup complete in "));
  Serial.print(millis());
  Serial.println(("ms."));
}

void hardwareAttachInterrupts() {
  // Motor speed is on Pin 2 -> PA0
  // Wheel speed is on Pin 3 -> PF5

  // 1. Configure Event System Generators
  // Route Port A, Pin 0 (Motor) to EVSYS Channel 0
  EVSYS.CHANNEL0 = EVSYS_GENERATOR_PORT0_PIN0_gc;

  // Route Port F, Pin 5 (Wheel) to EVSYS Channel 1
  EVSYS.CHANNEL1 = EVSYS_GENERATOR_PORT1_PIN5_gc;

  // 2. Configure Event System Users (The Timers)
  // Connect TCB0 (Motor) to Event Channel 0
  EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL0_gc;

  // Connect TCB1 (Wheel) to Event Channel 1
  EVSYS.USERTCB1 = EVSYS_CHANNEL_CHANNEL1_gc;

  // 3. Configure Timer B 0 (Motor)
  TCB0.CTRLB = TCB_CNTMODE_FRQ_gc;                   // Frequency/Pulse measurement mode
  TCB0.EVCTRL = TCB_CAPTEI_bm;                       // Enable Input Capture Event
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use TCA0 clock (250kHz, 4us ticks), Enable TCB0

  // 4. Configure Timer B 1 (Wheel)
  TCB1.CTRLB = TCB_CNTMODE_FRQ_gc;                   // Frequency/Pulse measurement mode
  TCB1.EVCTRL = TCB_CAPTEI_bm;                       // Enable Input Capture Event
  TCB1.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use TCA0 clock (250kHz, 4us ticks), Enable TCB1

  // 5. Normal Interrupts
  // We still attach the standard interrupts. The ISR will read the TCB hardware registers.
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

void hardwareSerialPrint(const char *text) {
  Serial.print(text);
}

void hardwareSerialPrint(const __FlashStringHelper *text) {
  Serial.print(text);
}

void hardwareSerialPrintln(const char *text) {
  Serial.println(text);
}

void hardwareSerialPrintln(const __FlashStringHelper *text) {
  Serial.println(text);
}

#endif // __AVR_ATmega4809__
