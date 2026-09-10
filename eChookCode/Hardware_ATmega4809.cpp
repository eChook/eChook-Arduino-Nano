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

/*
 * Internal temperature on the ATmega4809
 * --------------------------------------
 * Unlike the 328P, whose temperature sensor has no factory calibration and has
 * to be characterised by hand, the 4809 ships with per-device correction
 * factors in the signature row: SIGROW.TEMPSENSE0 is a gain/slope correction
 * and SIGROW.TEMPSENSE1 an offset correction, both written during production
 * test. Datasheet section 29.3.2.6 gives both the measurement procedure and
 * the conversion, and this follows it exactly.
 *
 * The measurement conditions are not optional. The sensor output is referenced
 * to the internal 1.1V reference, needs SAMPCAP set, and needs at least 32us of
 * both initialisation delay and sample time. Getting any of those wrong gives a
 * plausible looking number that is completely wrong.
 */
float hardwareReadTempInternal() {
  // Save everything this function touches so normal analogRead() is unaffected
  uint8_t oldMUXPOS = ADC0.MUXPOS;
  uint8_t oldCTRLC = ADC0.CTRLC;
  uint8_t oldCTRLD = ADC0.CTRLD;
  uint8_t oldSAMPCTRL = ADC0.SAMPCTRL;
  uint8_t oldVrefCtrlA = VREF.CTRLA;

  // Step 1: the VREF peripheral must actually be set to 1v1. Selecting INTREF
  // in ADC0.CTRLC alone is not enough - the Arduino core calls
  // analogReference(VDD) at startup, which clears ADC0REFSEL and leaves the
  // internal reference selected as 0v55.
  VREF.CTRLA = (oldVrefCtrlA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_1V1_gc;

  // Steps 2, 3 and 6: internal reference, temperature sensor channel, SAMPCAP
  // set as required for a reference above 1V. The core's prescaler is left
  // alone, which keeps CLK_ADC in the 50-200kHz band the ADC expects.
  ADC0.CTRLC = (oldCTRLC & ~ADC_REFSEL_gm) | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm;
  ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;

  // Steps 4 and 5: at least 32us of settling and 32us of sampling. At 125kHz
  // one CLK_ADC cycle is 8us, so 32 cycles of INITDLY and the maximum SAMPLEN
  // both clear that comfortably.
  ADC0.CTRLD = (oldCTRLD & ~ADC_INITDLY_gm) | ADC_INITDLY_DLY32_gc;
  ADC0.SAMPCTRL = ADC_SAMPLEN_gm;

  // Step 7: convert
  ADC0.INTFLAGS = ADC_RESRDY_bm; // Clear any stale result
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {
  }
  uint16_t adcReading = ADC0.RES;

  // Put the ADC and VREF back exactly as they were found
  ADC0.SAMPCTRL = oldSAMPCTRL;
  ADC0.CTRLD = oldCTRLD;
  ADC0.CTRLC = oldCTRLC;
  ADC0.MUXPOS = oldMUXPOS;
  VREF.CTRLA = oldVrefCtrlA;

  // Step 8: apply the factory calibration. This is the datasheet's own
  // sequence, which yields Kelvin.
  int8_t sigrowOffset = SIGROW.TEMPSENSE1; // Signed offset correction
  uint8_t sigrowGain = SIGROW.TEMPSENSE0;  // Unsigned gain/slope correction

  uint32_t temperature = adcReading - sigrowOffset;
  temperature *= sigrowGain; // 10 bit + 8 bit, so this needs the 32 bit type
  temperature += 0x80;       // Round rather than truncate on the shift below
  temperature >>= 8;         // Result is now in Kelvin

  return (float)temperature - 273.15f;
}

/*
 * Measuring the 5V rail on the ATmega4809
 * ---------------------------------------
 * The ATmega328P points the ADC mux straight at its 1.1V bandgap (channel 14)
 * and measures it against AVcc. The 4809 has no equivalent mux channel - its
 * MUXPOS options are the 16 analog pins, the temperature sensor, GND, and the
 * Analog Comparator's DAC reference (DACREF). There is no "internal reference"
 * channel, which is why the 328P trick does not port across directly.
 *
 * DACREF is the way in. The VREF peripheral has two independent outputs: one
 * feeding the ADC and one feeding the Analog Comparator's 8 bit DAC ladder.
 * Pointing the AC ladder at the internal 1.1V reference and setting it to full
 * scale puts a known voltage on the DACREF node, and the ADC can select that
 * node as its input while still using VDD as its reference. Same trick as the
 * 328P, just routed one peripheral further round the loop:
 *
 *   VDD (unknown) ------------------------------> ADC reference
 *   1.1V internal --> AC0 DAC ladder --> DACREF --> ADC input
 *
 *   V(DACREF) = CAL_INTERNAL_REFERENCE_VOLTAGE * (AC0.DACREF / 256)
 *   VDD       = V(DACREF) * 1024 / adcReading
 *
 * The ladder is used at full scale, so only its gain matters and not its
 * linearity. Any gain error folds into CAL_INTERNAL_REFERENCE_VOLTAGE in
 * exactly the same way the 328P's bandgap tolerance does, so the calibration
 * procedure and the stored EEPROM value keep the same meaning on both chips.
 */

// Samples accumulated per reading. Must match the ADC_SAMPNUM_ACCn_gc below.
// 8 samples gives a 1v1/5V reading of ~1800 counts, so roughly 2.8mV of
// resolution on the calculated rail voltage, and takes about 3ms.
#define REF_VOLTAGE_SAMPLE_COUNT 8
#define REF_VOLTAGE_SAMPLE_GROUP ADC_SAMPNUM_ACC8_gc

float hardwareUpdateReferenceVoltage() {
  // Save everything this function touches, so normal analogRead() calls are
  // unaffected by it.
  uint8_t oldMUXPOS = ADC0.MUXPOS;
  uint8_t oldCTRLB = ADC0.CTRLB;
  uint8_t oldCTRLC = ADC0.CTRLC;
  uint8_t oldCTRLD = ADC0.CTRLD;
  uint8_t oldSAMPCTRL = ADC0.SAMPCTRL;
  uint8_t oldVrefCtrlA = VREF.CTRLA;
  uint8_t oldVrefCtrlB = VREF.CTRLB;
  uint8_t oldAcCtrlA = AC0.CTRLA;
  uint8_t oldAcMuxCtrlA = AC0.MUXCTRLA;
  uint8_t oldAcDacRef = AC0.DACREF;

  // Feed the AC's DAC ladder from the internal 1.1V reference. AC0REFEN force
  // enables that reference: it is a separate VREF output to the one the ADC is
  // using, so both can be live at once.
  VREF.CTRLA = (oldVrefCtrlA & ~VREF_AC0REFSEL_gm) | VREF_AC0REFSEL_1V1_gc;
  VREF.CTRLB = oldVrefCtrlB | VREF_AC0REFEN_bm;

  AC0.DACREF = 255;                   // Full scale: 1.1V * 255/256 = 1.0957V
  AC0.MUXCTRLA = AC_MUXNEG_DACREF_gc; // Ladder output onto the DACREF node
  AC0.CTRLA = AC_ENABLE_bm;           // Comparator on, output buffer off

  // Point the ADC at the DACREF node, still referenced to the 5V rail. Only the
  // reference bits are changed, so the core's prescaler and sample capacitance
  // settings are left alone.
  ADC0.CTRLC = (oldCTRLC & ~ADC_REFSEL_gm) | ADC_REFSEL_VDDREF_gc;
  ADC0.MUXPOS = ADC_MUXPOS_DACREF_gc;
  ADC0.CTRLB = REF_VOLTAGE_SAMPLE_GROUP;
  ADC0.CTRLD = (oldCTRLD & ~ADC_INITDLY_gm) | ADC_INITDLY_DLY64_gc; // Let the reference settle
  ADC0.SAMPCTRL = ADC_SAMPLEN_gm;                                   // Longest sample time, the ladder is high impedance

  ADC0.INTFLAGS = ADC_RESRDY_bm; // Clear any stale result
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {
  }
  uint16_t accumulator = ADC0.RES; // Sum of REF_VOLTAGE_SAMPLE_COUNT conversions

  // Put the ADC, VREF and AC back exactly as they were found
  ADC0.SAMPCTRL = oldSAMPCTRL;
  ADC0.CTRLD = oldCTRLD;
  ADC0.CTRLB = oldCTRLB;
  ADC0.CTRLC = oldCTRLC;
  ADC0.MUXPOS = oldMUXPOS;
  AC0.CTRLA = oldAcCtrlA;
  AC0.MUXCTRLA = oldAcMuxCtrlA;
  AC0.DACREF = oldAcDacRef;
  VREF.CTRLB = oldVrefCtrlB;
  VREF.CTRLA = oldVrefCtrlA;

  if (accumulator == 0) {
    return CAL_REFERENCE_VOLTAGE; // Nothing measured, fall back to the stored value
  }

  float dacRefVoltage = CAL_INTERNAL_REFERENCE_VOLTAGE * (255.0f / 256.0f);
  float railVoltage = (dacRefVoltage * 1024.0f * REF_VOLTAGE_SAMPLE_COUNT) / (float)accumulator;

  // A result outside this band means the measurement is not trustworthy, so
  // fall back rather than scaling every other reading by a bad number.
  if (railVoltage < 3.0f || railVoltage > 6.0f) {
    return CAL_REFERENCE_VOLTAGE;
  }

  return railVoltage;
}

/*
 * The 4809's ADC can accumulate several conversions in hardware and leave the
 * sum in the result register, so oversampling costs one conversion sequence
 * rather than a loop of separate analogRead() calls. The core's analogRead()
 * returns the whole 16 bit result register, so it can be reused as is: 16
 * accumulated samples peak at 16368, comfortably inside a signed int.
 */
#define ANALOG_OVERSAMPLE_COUNT 16
#define ANALOG_OVERSAMPLE_GROUP ADC_SAMPNUM_ACC16_gc

float hardwareAnalogReadOversampled(uint8_t pin) {
  uint8_t oldCTRLB = ADC0.CTRLB;

  // Throwaway conversion to settle the mux on this pin. It matters most on the
  // reading straight after hardwareUpdateReferenceVoltage(), which leaves the
  // ADC pointed at the DACREF node.
  analogRead(pin);

  ADC0.CTRLB = ANALOG_OVERSAMPLE_GROUP;
  uint16_t accumulator = (uint16_t)analogRead(pin);
  ADC0.CTRLB = oldCTRLB;

  return (float)accumulator / (float)ANALOG_OVERSAMPLE_COUNT;
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
