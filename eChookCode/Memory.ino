/**
 * @file Memory.ino
 * @brief EEPROM storage management for calibration and settings.
 *
 * This file handles saving and reading settings and calibrations from the
 * Arduino's EEPROM memory.
 *
 * EEPROM Schema (125 Bytes used / 1kB total):
 * - [0]: Validity Check (0xAA if valid).
 * - [1-4]: Binary Configuration (Byte A, B, C, D).
 * - [5-84]: Float Calibration Storage (20 floats * 4 bytes).
 * - [86-115]: Bluetooth Name Storage (30 chars).
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
 * Loads calibration from EEPROM if valid, otherwise saves defaults.
 */
void EEPROMSetup() {
  if (!FORCE_USE_HARDCODED_CAL) {
    if (getVerificationByte()) {
      loadEepromCalibration();
    } else {
      saveCurrCalToEeprom();
    }
  }
}

/**
 * @brief Saves current global calibration values into EEPROM.
 */
void saveCurrCalToEeprom() {
  if (!getVerificationByte()) {
    setVerificationByte();
  }

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

  CAL_DATA_TRANSMIT_INTERVAL = (unsigned long)getFloatCal(INDEX_TRANSMIT_INTERVAL);
  CAL_WHEEL_MAGNETS = (int)getFloatCal(INDEX_WHEEL_MAGNETS);
  CAL_MOTOR_MAGNETS = (int)getFloatCal(INDEX_MOTOR_MAGNETS);
  CAL_WHEEL_CIRCUMFERENCE = getFloatCal(INDEX_WHEEL_CIRCUMFERENCE);
  CAL_REFERENCE_VOLTAGE = getFloatCal(INDEX_REF_VOLTAGE);
  CAL_INTERNAL_REFERENCE_VOLTAGE = getFloatCal(INDEX_INTERNAL_REFERENCE_VOLTAGE);
  CAL_BATTERY_TOTAL = getFloatCal(INDEX_24_VOLTAGE);
  CAL_BATTERY_LOWER = getFloatCal(INDEX_12_VOLTAGE);
  CAL_CURRENT = getFloatCal(INDEX_CURRENT);
  CAL_THERM1_A = getFloatCal(INDEX_TEMP_1_A);
  CAL_THERM1_B = getFloatCal(INDEX_TEMP_1_B);
  CAL_THERM1_C = getFloatCal(INDEX_TEMP_1_C);
  CAL_THERM2_A = getFloatCal(INDEX_TEMP_2_A);
  CAL_THERM2_B = getFloatCal(INDEX_TEMP_2_B);
  CAL_THERM2_C = getFloatCal(INDEX_TEMP_2_C);
  CAL_THROTTLE_LOW = (int)getFloatCal(INDEX_THROTTLE_LOW);
  CAL_THROTTLE_HIGH = (int)getFloatCal(INDEX_THROTTLE_HIGH);

  // Default internal reference if not set
  if (CAL_INTERNAL_REFERENCE_VOLTAGE == 0 || CAL_INTERNAL_REFERENCE_VOLTAGE == 0xFF) {
    CAL_INTERNAL_REFERENCE_VOLTAGE = 1.1;
    setFloatCal(INDEX_INTERNAL_REFERENCE_VOLTAGE, (float)CAL_INTERNAL_REFERENCE_VOLTAGE);
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
 * @brief Checks if the EEPROM contains valid calibration data.
 * @return 1 if valid (header matches 0xAA), 0 otherwise.
 */
uint8_t getVerificationByte() {
  byte temp = EEPROM.read(0);
  return temp == 0xAA;
}

/**
 * @brief Writes the validity header (0xAA) to EEPROM position [0].
 */
void setVerificationByte() { EEPROM.write(0, 0xAA); }

/**
 * @brief Clears the validity header (sets to 0xFF).
 */
void clearVerificationByte() { EEPROM.write(0, 0xFF); }

/**
 * @brief Writes the current Bluetooth name to EEPROM.
 */
void writeBTName() {
  char buff[30] = {0};
  CAL_BT_NAME.toCharArray(buff, 30);
  EEPROM.put(NAME_ARRAY_START, buff);
}

/**
 * @brief Reads the Bluetooth name from EEPROM into the global variable.
 */
void getBTName() {
  String temp = "";
  for (uint8_t i = 0; i < 30; i++) {
    char tmpChar = getNameByte(i);
    if (tmpChar != 0x00 && tmpChar != 0xff)
      temp += tmpChar;
  }
  CAL_BT_NAME = temp;
}
