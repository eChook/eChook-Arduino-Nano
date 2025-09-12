// This file holds all functions relating to saving and reading settings and calibrations from the Arduino's EEPROM memory.

//  The saved data is broken into a few parts:
//  - First byte [0] of EEPROM set to 0xAA to indicate EEPROM data is valid, anything else will result in eeprom being overwritten
//    with default settings.
//  - 4 Chars used for binary settings - eg. feature enable/disable.
//  - An array of floats for the numerical calibrations values. (OK, not really an array, just a series of 20 floats, but thinking of it as an array may help)
//  - A 20 char array to hold the name for the Bluetooth device.

// The Atmega328P has 1Kb of memory. The Schema below uses:
// 1 Byte for Validity Check.
// 4 Bytes for binary configuration.
// 80 Bytes for float calibrations storage. A Float is 4 Bytes, 4*20 = 80
// 30 Bytes for Bluetooth Name Storage.
//
// Total = 125 Bytes of 1kB total. Plenty of space!

// The configuraration bytes are as follows: (Big Endian Notation)
// Byte A, General Settings (EEPROM[1]):-
//          0 - HIGH Use EEPROM settings | LOW Use hard coded settings
//          1 - HIGH Variable Throttle | LOW Push Button digital throttle
//          2 - HIGH Thermistor Temperature Sensors | LOW Linear Temperature Sensors
//          3 - HIGH Enable PWM Output | LOW Disable PWM Output
//          4 - HIGH Enable Ramped Throttle | LOW map throttle directly

// Byte B and C - Spare for future upgrades ;)

// Byte D, Experimental Features, subject to change:
//          0 - HIGH Use new RPM Calculation
//          1 - HIGH Use new Wheel Speed Calculation

// The float array is indexed as follows:
//  0 - Data Transmit Interval in Ms
//  1 - # Wheel Magnets
//  2 - # Motor Magnets
//  3 - Measured Reference Voltage (0 for auto calibration)
//  4 - 24v Measurement Multiplier
//  5 - 12v Measurement Multiplier
//  6 - Current Measurement Multiplier
//  7 - Temp1 Steinhart Hart Coefficient A / Linear Sensor 1 Multiplier
//  8 - Temp1 Steinhart Hart Coefficient B
//  9 - Temp1 Steinhart Hart Coefficient C
//  10 - Temp2 Steinhart Hart Coefficient A / Linear Sensor 2 Multiplier
//  11 - Temp2 Steinhart Hart Coefficient B
//  12 - Temp2 Steinhart Hart Coefficient C
//  13 - Throttle Low Voltage Threshold
//  14 - Throttle High Voltage Threshold
//  15 - Wheel Circumference in Meters
//  16 - Internal Reference voltage
//  17 - SPARE
//  18 - SPARE
//  19 - SPARE

// Code:
#include <EEPROM.h>

#define EEPROM_USED_SIZE 125 // Total bytes used for config, floats, BT name, etc.
#define CHECKSUM_BYTE 0      // Store checksum in EEPROM[0]

// Calibration Bit Definitions:
// Reading bits in bytes requires some binary logic

#define VERIFICATION_BYTE 0
#define CAL_A 1
#define CAL_B 2
#define CAL_C 3
#define CAL_D 4
#define FLOAT_ARRAY_START 5
#define NAME_ARRAY_START 86 // 5 + 20*4 + 1

// Calibration Byte A Locations
#define A_EEPROM_ENABLE 0x80
#define A_THROTTLE_MODE 0x40 // Analogue or digital
#define A_TEMP_SENSOR_MODE 0x20
#define A_PWM_ENABLE 0x10
#define A_THROTTLE_RAMP 0x08

// Calibration Byte D Locations
#define D_RPM_NEW 0x80
#define D_SPEED_NEW 0x40

// Float Locations
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

// Functions

/**
 * @brief Sets up EEPROM by checking validity and loading or saving calibration data.
 * If EEPROM is valid, loads calibration; otherwise, writes default calibration.
 */
void EEPROMSetup()
{

    // For testing purposes, write eeprom at each boot:
    // saveCurrCalToEeprom();

    if (!FORCE_USE_HARDCODED_CAL)
    {
        // First check if EEPROM is valid by reading the validity Byte [0]
        if (getVerificationByte())
        {
            // EEPROM contains valid data
            // Set calibration values from EEPROM
            loadEepromCalibration();
        }
        else
        {
            // Fresh arduino, need to initialise EEPROM
            saveCurrCalToEeprom();
        }
    }
}

/**
 * @brief Saves current calibration and settings to EEPROM.
 * Sets binary calibration, float values, Bluetooth name, and writes checksum.
 */
void saveCurrCalToEeprom()
{

    // Check and Set Verification Byte
    if (!getVerificationByte())
    {
        setVerificationByte();
    }

    // Set Binary Cals:
    setBinaryCal(CAL_A, A_EEPROM_ENABLE, CAL_USE_EEPROM);
    setBinaryCal(CAL_A, A_THROTTLE_MODE, CAL_THROTTLE_VARIABLE);
    setBinaryCal(CAL_A, A_THROTTLE_RAMP, CAL_THROTTLE_RAMP);
    setBinaryCal(CAL_A, A_PWM_ENABLE, CAL_THROTTLE_OUTPUT_EN);
    setBinaryCal(CAL_A, A_TEMP_SENSOR_MODE, CAL_LINEAR_TEMPERATURE);

    setBinaryCal(CAL_D, D_RPM_NEW, CAL_USE_IMPROVED_RPM_CALCULATION);
    setBinaryCal(CAL_D, D_SPEED_NEW, CAL_USE_IMPROVED_SPEED_CALCULATION);

    // setFloats
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
}

/**
 * @brief Loads calibration and settings from EEPROM into global variables.
 * Reads binary calibration, float values, and Bluetooth name.
 */
void loadEepromCalibration()
{

    CAL_USE_EEPROM = readBinaryCal(CAL_A, A_EEPROM_ENABLE);
    CAL_THROTTLE_VARIABLE = readBinaryCal(CAL_A, A_THROTTLE_MODE);
    CAL_THROTTLE_OUTPUT_EN = readBinaryCal(CAL_A, A_PWM_ENABLE);
    CAL_THROTTLE_RAMP = readBinaryCal(CAL_A, A_THROTTLE_RAMP);
    CAL_USE_IMPROVED_RPM_CALCULATION = readBinaryCal(CAL_D, D_RPM_NEW);
    CAL_USE_IMPROVED_SPEED_CALCULATION = readBinaryCal(CAL_D, D_SPEED_NEW); // Will work best with one magnet on the wheel

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

    // Catch function for Int Ref Voltage on firmware upgrade
    if(CAL_INTERNAL_REFERENCE_VOLTAGE == 0 || CAL_INTERNAL_REFERENCE_VOLTAGE == 0xFF ){
      CAL_INTERNAL_REFERENCE_VOLTAGE = 1.1;
      setFloatCal(INDEX_INTERNAL_REFERENCE_VOLTAGE, (float)CAL_INTERNAL_REFERENCE_VOLTAGE);
    }

    getBTName();
}

/**
 * @brief Reads a specific bit from a calibration byte in EEPROM.
 * @param byte The EEPROM address of the calibration byte.
 * @param bit The bit mask to isolate the desired bit.
 * @return 1 if the bit is set, 0 otherwise.
 */
uint8_t readBinaryCal(char byte, char bit)
{
    char temp = EEPROM.read(byte);
    temp = temp & bit;   // Ands the whole calibration byte with the one bit we want to look at, isolating it.
    return temp ? 1 : 0; // This can now be treated as a binary HIGH / LOW, as all the bits we aren't interesting are LOW. Simplified to 1 or 0 for return.
}

/**
 * @brief Reads a full calibration byte from EEPROM.
 * @param byte The EEPROM address of the calibration byte.
 * @return The value of the calibration byte.
 */
byte getBinaryCalByte(char byte)
{
    char temp = EEPROM.read(byte);
    return temp; // This can now be treated as a binary HIGH / LOW, as all the bits we aren't interesting are LOW. Simplified to 1 or 0 for return.
}

/**
 * @brief Sets a specific bit in a calibration byte in EEPROM.
 * @param byte The EEPROM address of the calibration byte.
 * @param bit The bit mask to set.
 */
void setBinaryCal(char byte, char bit)
{
    char temp = EEPROM.read(byte);
    temp = temp | bit; // OR desired bit with existing data. Only targeted bit will flip to 1.
    EEPROM.write(byte, temp);
}

/**
 * @brief Sets or clears a specific bit in a calibration byte in EEPROM.
 * @param byte The EEPROM address of the calibration byte.
 * @param bit The bit mask to set or clear.
 * @param value If nonzero, sets the bit; if zero, clears the bit.
 */
void setBinaryCal(char byte, char bit, uint8_t value)
{
    char temp = EEPROM.read(byte);
    if (value)
    {
        temp = temp | bit; // OR desired bit with existing data. Only targeted bit will flip to 1.
    }
    else
    {
        temp = temp & ~bit; // Binary AND existing byte with binary inverted target byte. Only terget bit will AND with 0 and therefore be set to 0.
    }
    EEPROM.write(byte, temp);
}

/**
 * @brief Clears a specific bit in a calibration byte in EEPROM.
 * @param byte The EEPROM address of the calibration byte.
 * @param bit The bit mask to clear.
 */
void clearBinaryCal(char byte, char bit)
{
    char temp = EEPROM.read(byte);
    temp = temp & ~bit; // Binary AND existing byte with binary inverted target byte. Only terget bit will AND with 0 and therefore be set to 0.
    EEPROM.write(byte, temp);
}

/**
 * @brief Writes a float value to EEPROM at the specified index.
 * @param index The index in the float array.
 * @param value The float value to write.
 */
void setFloatCal(uint8_t index, float value)
{
    uint8_t address = FLOAT_ARRAY_START + (index * 4);
    EEPROM.put(address, value);
}

/**
 * @brief Reads a float value from EEPROM at the specified index.
 * @param index The index in the float array.
 * @return The float value read from EEPROM.
 */
float getFloatCal(uint8_t index)
{
    float temp = 0;                                  // Pre define float to write to
    uint8_t address = index * 4 + FLOAT_ARRAY_START; // Each float is 4 bytes, plus the array start position to locate the float.
    EEPROM.get(address, temp);
    return temp;
}

/**
 * @brief Reads a byte from the float array in EEPROM at the specified index.
 * @param index The byte index in the float array.
 * @return The byte value read from EEPROM.
 */
byte getFloatByte(uint8_t index)
{
    byte temp = 0;                               // Pre define float to write to
    uint8_t address = index + FLOAT_ARRAY_START; // Each float is 4 bytes, plus the array start position to locate the float,
    EEPROM.get(address, temp);
    return temp;
}

/**
 * @brief Reads a byte from the Bluetooth name array in EEPROM at the specified index.
 * @param index The byte index in the name array.
 * @return The byte value read from EEPROM.
 */
byte getNameByte(uint8_t index)
{
    byte temp = 0;                              // Pre define float to write to
    uint8_t address = index + NAME_ARRAY_START; // Each float is 4 bytes, plus the array start position to locate the float,
    EEPROM.get(address, temp);
    return temp;
}

/**
 * @brief Checks if the verification byte in EEPROM is set to 0xAA.
 * @return 1 if verification byte is 0xAA, 0 otherwise.
 */
uint8_t getVerificationByte()
{
    byte temp = EEPROM.read(0);
    return temp == 0xAA;
}

/**
 * @brief Sets the verification byte in EEPROM to 0xAA.
 */
void setVerificationByte()
{
    EEPROM.write(0, 0xAA);
}

/**
 * @brief Clears the verification byte in EEPROM (sets to 0xFF).
 */
void clearVerificationByte()
{
    EEPROM.write(0, 0xFF);
}

/**
 * @brief Writes the Bluetooth device name to EEPROM.
 *
 * Converts the CAL_BT_NAME String to a char buffer (max 30 bytes) and writes it to EEPROM
 * starting at NAME_ARRAY_START. Any unused bytes in the buffer are set to 0xff.
 * This overwrites any previous Bluetooth name stored in EEPROM.
 */
void writeBTName()
{
    // Overwrite any previous data:
    // for (uint8_t i = NAME_ARRAY_START; i < NAME_ARRAY_START + 30; i++)
    // {
    //     EEPROM.write(i, 0xff);
    // }

    char buff[30] = {0xff};
    CAL_BT_NAME.toCharArray(buff, 30);
    // Serial.println(buff);

    EEPROM.put(NAME_ARRAY_START, buff);
}

/**
 * @brief Reads the Bluetooth device name from EEPROM and stores it in CAL_BT_NAME.
 * 
 * Reads up to 30 bytes from EEPROM starting at NAME_ARRAY_START. 
 * Ignores bytes with value 0xff (unused/empty).
 * Concatenates valid characters into a String and assigns it to CAL_BT_NAME.
 */
void getBTName()
{
    String temp = "";
    for (uint8_t i = 0; i < 30; i++)
    {
        char tmpChar = getNameByte(i);
        if (tmpChar != 0xff)
            temp += tmpChar;
    }
    CAL_BT_NAME = temp;
}

/**
 * @brief Calculates a very simple 8-bit checksum (sum of all bytes) over EEPROM data.
 * Excludes the checksum byte itself (EEPROM[0]).
 * @return The calculated checksum.
 */
uint8_t calculateEEPROMChecksum() {
    uint8_t checksum = 0;
    for (uint8_t i = 1; i < EEPROM_USED_SIZE; i++) { // Start from 1, skip checksum byte
        checksum += EEPROM.read(i);
    }
    return checksum;
}

/**
 * @brief Writes the checksum to EEPROM[0].
 */
void writeEEPROMChecksum() {
    uint8_t checksum = calculateEEPROMChecksum();
    EEPROM.write(CHECKSUM_BYTE, checksum);
}

/**
 * @brief Verifies the EEPROM checksum.
 * @return true if checksum matches, false otherwise.
 */
bool verifyEEPROMChecksum() {
    uint8_t stored = EEPROM.read(CHECKSUM_BYTE);
    uint8_t calculated = calculateEEPROMChecksum();
    return stored == calculated;
}
