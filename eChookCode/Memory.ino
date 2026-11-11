/**
 * @file Memory.ino
 * @brief EEPROM storage management for calibration and settings.
 *
 * This file handles saving and reading settings and calibrations from the
 * Arduino's EEPROM memory.
 *
 * EEPROM Schema (125 Bytes used / 1kB total):
 * - [0]: Marker. 0xAB = protected by the checksum in [124]. 0xAA = v2.5 beta
 *        layout with no checksum, upgraded in place. Anything else = reset to defaults.
 * - [1-4]: Binary Configuration (Byte A, B, C, D).
 * - [5-84]: Float Calibration Storage (20 floats * 4 bytes).
 * - [86-115]: Bluetooth Name Storage (30 chars).
 * - [124]: Checksum, 8-bit sum of bytes [1-123].
 *
 * Configuration Byte A Bits:
 * - 0: Use EEPROM settings (High) / Use hardcoded (Low).
 * - 1: Variable Throttle (High) / Push Button (Low).
 * - 2: Thermistors (High) / Linear Sensors (Low).
 * - 3: PWM Output Enable (High) / Disable (Low).
 * - 4: Ramped Throttle (High) / Direct Mapping (Low).
 */

#include <EEPROM.h>

/** @brief Byte address for the EEPROM verification marker. */
#define VERIFICATION_BYTE 0
/** @brief Byte address for Configuration Byte A. */
#define CAL_A 1
/** @brief Byte address for Configuration Byte B (Spare). */
#define CAL_B 2
/** @brief Byte address for Configuration Byte C (Spare). */
#define CAL_C 3
/** @brief Byte address for Configuration Byte D (Experimental). */
#define CAL_D 4
/** @brief Starting address for the float calibration array. */
#define FLOAT_ARRAY_START 5
/** @brief Starting address for the Bluetooth name string. */
#define NAME_ARRAY_START 86
/** @brief Byte address for the checksum over bytes 1-123. */
#define CHECKSUM_BYTE 124
/** @brief Marker value: settings protected by the checksum. */
#define MARKER_CHECKSUMMED 0xAB
/** @brief Marker value: older layout with no checksum (v2.5 beta). */
#define MARKER_LEGACY 0xAA

// Calibration Byte A Bit-masks
/** @brief Mask for EEPROM enable bit in Byte A. */
#define A_EEPROM_ENABLE 0x80
/** @brief Mask for throttle mode bit in Byte A. */
#define A_THROTTLE_MODE 0x40
/** @brief Mask for temperature sensor mode bit in Byte A. */
#define A_TEMP_SENSOR_MODE 0x20
/** @brief Mask for PWM enable bit in Byte A. */
#define A_PWM_ENABLE 0x10
/** @brief Mask for throttle ramp bit in Byte A. */
#define A_THROTTLE_RAMP 0x08

// Calibration Byte D Bit-masks
/** @brief Mask for new RPM calculation bit in Byte D. */
#define D_RPM_NEW 0x80
/** @brief Mask for new speed calculation bit in Byte D. */
#define D_SPEED_NEW 0x40

// Float Array Index Definitions
#define INDEX_TRANSMIT_INTERVAL 0
#define INDEX_WHEEL_MAGNETS 1
#define INDEX_MOTOR_MAGNETS 2
#define INDEX_REF_VOLTAGE 3
#define INDEX_24_VOLTAGE 4
#define INDEX_12_VOLTAGE 5
#define INDEX_CURRENT 6
#define INDEX_TEMP_1_A 7
#define INDEX_TEMP_1_B 8
#define INDEX_TEMP_1_C 9
#define INDEX_TEMP_2_A 10
#define INDEX_TEMP_2_B 11
#define INDEX_TEMP_2_C 12
#define INDEX_THROTTLE_LOW 13
#define INDEX_THROTTLE_HIGH 14
#define INDEX_WHEEL_CIRCUMFERENCE 15
#define INDEX_INTERNAL_REFERENCE_VOLTAGE 16

/**
 * @brief Initializes EEPROM settings.
 * Loads calibration from EEPROM if the marker and checksum are valid, otherwise
 * saves defaults. Prints a one line status on boards with a spare serial port.
 */
void EEPROMSetup() {
  if (!FORCE_USE_HARDCODED_CAL) {
    uint8_t marker = EEPROM.read(VERIFICATION_BYTE);
    if (marker == MARKER_CHECKSUMMED && verifyEEPROMChecksum()) {
      loadEepromCalibration();
      hardwareSerialPrintln(F("EEPROM checksum OK - settings loaded"));
    } else if (marker == MARKER_LEGACY) {
      // Saved without a checksum by v2.5 beta: keep the settings and add one.
      // Checksum goes in before the marker, so a power cut here just repeats the upgrade.
      writeEEPROMChecksum();
      EEPROM.write(VERIFICATION_BYTE, MARKER_CHECKSUMMED);
      loadEepromCalibration();
      hardwareSerialPrintln(F("EEPROM checksum added - settings loaded"));
    } else {
      saveCurrCalToEeprom();
      hardwareSerialPrintln(F("EEPROM checksum bad - resetting to defaults"));
    }
  }
}

/**
 * @brief Saves current global calibration values into EEPROM.
 */
void saveCurrCalToEeprom() {
  // Invalidate first and mark valid last, so a power cut mid-save resets to defaults next boot.
  clearVerificationByte();

  setBinaryCal(CAL_A, A_EEPROM_ENABLE, CAL_USE_EEPROM);
  setBinaryCal(CAL_A, A_THROTTLE_MODE, CAL_THROTTLE_VARIABLE);
  setBinaryCal(CAL_A, A_THROTTLE_RAMP, CAL_THROTTLE_RAMP);
  setBinaryCal(CAL_A, A_PWM_ENABLE, CAL_THROTTLE_OUTPUT_EN);
  setBinaryCal(CAL_A, A_TEMP_SENSOR_MODE, CAL_LINEAR_TEMPERATURE);

  setBinaryCal(CAL_D, D_RPM_NEW, CAL_USE_IMPROVED_RPM_CALCULATION);
  setBinaryCal(CAL_D, D_SPEED_NEW, CAL_USE_IMPROVED_SPEED_CALCULATION);

  setFloatCal(INDEX_TRANSMIT_INTERVAL, (float)CAL_DATA_TRANSMIT_INTERVAL);
  setFloatCal(INDEX_WHEEL_MAGNETS, (float)CAL_WHEEL_MAGNETS);
  setFloatCal(INDEX_MOTOR_MAGNETS, (float)CAL_MOTOR_MAGNETS);
  setFloatCal(INDEX_REF_VOLTAGE, (float)CAL_REFERENCE_VOLTAGE);
  setFloatCal(INDEX_24_VOLTAGE, (float)CAL_BATTERY_TOTAL);
  setFloatCal(INDEX_12_VOLTAGE, (float)CAL_BATTERY_LOWER);
  setFloatCal(INDEX_CURRENT, (float)CAL_CURRENT);
  setFloatCal(INDEX_TEMP_1_A, (float)CAL_THERM1_A);
  setFloatCal(INDEX_TEMP_1_B, (float)CAL_THERM1_B);
  setFloatCal(INDEX_TEMP_1_C, (float)CAL_THERM1_C);
  setFloatCal(INDEX_TEMP_2_A, (float)CAL_THERM2_A);
  setFloatCal(INDEX_TEMP_2_B, (float)CAL_THERM2_B);
  setFloatCal(INDEX_TEMP_2_C, (float)CAL_THERM2_C);
  setFloatCal(INDEX_THROTTLE_LOW, (float)CAL_THROTTLE_LOW);
  setFloatCal(INDEX_THROTTLE_HIGH, (float)CAL_THROTTLE_HIGH);
  setFloatCal(INDEX_WHEEL_CIRCUMFERENCE, (float)CAL_WHEEL_CIRCUMFERENCE);
  setFloatCal(INDEX_INTERNAL_REFERENCE_VOLTAGE, (float)CAL_INTERNAL_REFERENCE_VOLTAGE);

  writeBTName();
  writeEEPROMChecksum();
  EEPROM.write(VERIFICATION_BYTE, MARKER_CHECKSUMMED);
}

/**
 * @brief Loads calibration values from EEPROM into global variables.
 */
void loadEepromCalibration() {
  CAL_USE_EEPROM = readBinaryCal(CAL_A, A_EEPROM_ENABLE);
  CAL_THROTTLE_VARIABLE = readBinaryCal(CAL_A, A_THROTTLE_MODE);
  CAL_THROTTLE_OUTPUT_EN = readBinaryCal(CAL_A, A_PWM_ENABLE);
  CAL_THROTTLE_RAMP = readBinaryCal(CAL_A, A_THROTTLE_RAMP);
  CAL_USE_IMPROVED_RPM_CALCULATION = readBinaryCal(CAL_D, D_RPM_NEW);
  CAL_USE_IMPROVED_SPEED_CALCULATION = readBinaryCal(CAL_D, D_SPEED_NEW);

  // Settings that the throttle and timing depend on are only taken when they make
  // sense; otherwise the current value (the default at boot, or the last good
  // value) is kept. NaN fails every comparison, so corrupt bytes are rejected too.
  float value = getFloatCal(INDEX_TRANSMIT_INTERVAL);
  if (value >= 10 && value <= 10000)
    CAL_DATA_TRANSMIT_INTERVAL = (unsigned long)value;
  value = getFloatCal(INDEX_WHEEL_MAGNETS);
  if (value >= 0 && value <= 100)
    CAL_WHEEL_MAGNETS = (int)value;
  value = getFloatCal(INDEX_MOTOR_MAGNETS);
  if (value >= 0 && value <= 100)
    CAL_MOTOR_MAGNETS = (int)value;
  value = getFloatCal(INDEX_WHEEL_CIRCUMFERENCE);
  if (value > 0 && value < 10)
    CAL_WHEEL_CIRCUMFERENCE = value;

  CAL_REFERENCE_VOLTAGE = getFloatCal(INDEX_REF_VOLTAGE);
  CAL_BATTERY_TOTAL = getFloatCal(INDEX_24_VOLTAGE);
  CAL_BATTERY_LOWER = getFloatCal(INDEX_12_VOLTAGE);
  CAL_CURRENT = getFloatCal(INDEX_CURRENT);
  CAL_THERM1_A = getFloatCal(INDEX_TEMP_1_A);
  CAL_THERM1_B = getFloatCal(INDEX_TEMP_1_B);
  CAL_THERM1_C = getFloatCal(INDEX_TEMP_1_C);
  CAL_THERM2_A = getFloatCal(INDEX_TEMP_2_A);
  CAL_THERM2_B = getFloatCal(INDEX_TEMP_2_B);
  CAL_THERM2_C = getFloatCal(INDEX_TEMP_2_C);
  // Checked as a pair: LOW must sit below HIGH, or the throttle maths divides by zero.
  float throttleLow = getFloatCal(INDEX_THROTTLE_LOW);
  float throttleHigh = getFloatCal(INDEX_THROTTLE_HIGH);
  if (throttleLow >= 0 && throttleHigh > throttleLow && throttleHigh <= 6) {
    CAL_THROTTLE_LOW = throttleLow;
    CAL_THROTTLE_HIGH = throttleHigh;
  }

  // Default the internal reference if missing or implausible (the 1.1V bandgap is +/-10%)
  value = getFloatCal(INDEX_INTERNAL_REFERENCE_VOLTAGE);
  if (value >= 0.9 && value <= 1.3) {
    CAL_INTERNAL_REFERENCE_VOLTAGE = value;
  } else {
    CAL_INTERNAL_REFERENCE_VOLTAGE = 1.1;
    setFloatCal(INDEX_INTERNAL_REFERENCE_VOLTAGE, (float)CAL_INTERNAL_REFERENCE_VOLTAGE);
    writeEEPROMChecksum();
  }

  getBTName();
}

/**
 * @brief Reads a single binary flag from a calibration byte.
 * @param byte The EEPROM address of the byte.
 * @param bit The bitmask of the flag.
 * @return 1 if true, 0 if false.
 */
uint8_t readBinaryCal(char byte, char bit) {
  char temp = EEPROM.read(byte);
  temp = temp & bit;
  return temp ? 1 : 0;
}

/**
 * @brief Reads an entire calibration byte from EEPROM.
 * @param byte The EEPROM address.
 * @return The 8-bit byte value.
 */
byte getBinaryCalByte(char byte) { return EEPROM.read(byte); }

/**
 * @brief Sets a binary flag in a calibration byte to 1.
 * @param byte The EEPROM address.
 * @param bit The bitmask to set.
 */
void setBinaryCal(char byte, char bit) {
  char temp = EEPROM.read(byte);
  temp = temp | bit;
  EEPROM.write(byte, temp);
}

/**
 * @brief Sets or clears a binary flag in a calibration byte.
 * @param byte The EEPROM address.
 * @param bit The bitmask to modify.
 * @param value 1 to set (High), 0 to clear (Low).
 */
void setBinaryCal(char byte, char bit, uint8_t value) {
  char temp = EEPROM.read(byte);
  if (value) {
    temp = temp | bit;
  } else {
    temp = temp & ~bit;
  }
  EEPROM.write(byte, temp);
}

/**
 * @brief Clears a binary flag in a calibration byte (sets to 0).
 * @param byte The EEPROM address.
 * @param bit The bitmask to clear.
 */
void clearBinaryCal(char byte, char bit) {
  char temp = EEPROM.read(byte);
  temp = temp & ~bit;
  EEPROM.write(byte, temp);
}

/**
 * @brief Stores a float value in the EEPROM calibration array.
 * @param index The index in the float array (0-19).
 * @param value The float value to save.
 */
void setFloatCal(uint8_t index, float value) {
  uint8_t address = FLOAT_ARRAY_START + (index * 4);
  EEPROM.put(address, value);
}

/**
 * @brief Retrieves a float value from the EEPROM calibration array.
 * @param index The index in the float array (0-19).
 * @return The stored float value.
 */
float getFloatCal(uint8_t index) {
  float temp = 0;
  uint8_t address = index * 4 + FLOAT_ARRAY_START;
  EEPROM.get(address, temp);
  return temp;
}

/**
 * @brief Reads a single raw byte from a float's address in EEPROM.
 * @param index byte index relative to float array start.
 * @return The raw byte value.
 */
byte getFloatByte(uint8_t index) {
  byte temp = 0;
  uint8_t address = index + FLOAT_ARRAY_START;
  EEPROM.get(address, temp);
  return temp;
}

/**
 * @brief Reads a single raw byte from the name array in EEPROM.
 * @param index byte index relative to name array start.
 * @return The raw byte value.
 */
byte getNameByte(uint8_t index) {
  byte temp = 0;
  uint8_t address = index + NAME_ARRAY_START;
  EEPROM.get(address, temp);
  return temp;
}

/**
 * @brief Clears the validity marker (sets to 0xFF), forcing defaults on next boot.
 */
void clearVerificationByte() { EEPROM.write(VERIFICATION_BYTE, 0xFF); }

/**
 * @brief Calculates an 8-bit checksum (sum of all bytes) over the settings, bytes 1 to 123.
 * Excludes the marker byte [0] and the checksum byte itself.
 * @return The calculated checksum.
 */
uint8_t calculateEEPROMChecksum() {
  uint8_t checksum = 0;
  for (uint8_t i = 1; i < CHECKSUM_BYTE; i++) {
    checksum += EEPROM.read(i);
  }
  return checksum;
}

/**
 * @brief Writes the checksum to EEPROM[CHECKSUM_BYTE]. Call after every settings write.
 */
void writeEEPROMChecksum() { EEPROM.update(CHECKSUM_BYTE, calculateEEPROMChecksum()); }

/**
 * @brief Checks the stored checksum against the settings.
 * @return true if they match.
 */
bool verifyEEPROMChecksum() { return EEPROM.read(CHECKSUM_BYTE) == calculateEEPROMChecksum(); }

/**
 * @brief Writes the current Bluetooth name to EEPROM.
 */
void writeBTName() {
  char buff[30] = {0};
  strncpy(buff, CAL_BT_NAME, 29);
  EEPROM.put(NAME_ARRAY_START, buff);
}

void getBTName() {
  uint8_t outIdx = 0;
  for (uint8_t i = 0; i < 30 && outIdx < 31; i++) {
    char tmpChar = getNameByte(i);
    if (tmpChar != (char)0x00 && tmpChar != (char)0xff) {
      CAL_BT_NAME[outIdx++] = tmpChar;
    }
  }
  CAL_BT_NAME[outIdx] = '\0';
}
