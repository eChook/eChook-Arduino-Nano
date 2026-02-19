/**
 * @file Calibration.h
 * @brief Calibration values for the eChook board.
 *
 * This file contains all the values used to calibrate the eChook board.
 * Separating these allows for updating the main code without losing
 * custom calibrations.
 *
 * Every variable in this file begins with CAL_ to indicate it is a
 * calibration-related setting.
 */

/**
 * @brief Force use of hardcoded calibration settings.
 * If 1, bypasses any settings stored in EEPROM and uses these file values.
 * If 0, uses the online configuration/calibration tool settings.
 */
uint8_t FORCE_USE_HARDCODED_CAL = 0;

/** @brief Enable reading/writing of calibration values to EEPROM. */
uint8_t CAL_USE_EEPROM = 1;

// Bluetooth Settings
/** @brief The bluetooth name for the car. */
String CAL_BT_NAME = "eChook";
/** @brief The bluetooth pairing password. */
String CAL_BT_PASSWORD = "1234";
/** @brief Serial communication baud rate. Must match both BT module and Arduino
 * settings. */
long CAL_BT_BAUDRATE =
    115200; // Baud Rate to run at. Must match Arduino's baud rate.

// Data Read and Transmit Interval:
/** @brief Interval (in ms) between data transmissions. */
unsigned long CAL_DATA_TRANSMIT_INTERVAL = 100; // transmit interval in ms

// Car Specific Settings
/** @brief Number of magnets mounted on the wheel for speed sensing. */
int CAL_WHEEL_MAGNETS = 2; // Number of magnets on wheel
/** @brief Number of magnets on the motor shaft for RPM sensing. */
int CAL_MOTOR_MAGNETS =
    1; // Number of magnets on motor shaft for hall effect sensor
/** @brief Outer circumference of the tyre in meters. Used for speed
 * calculation. */
float CAL_WHEEL_CIRCUMFERENCE =
    1.178; // Outer circumference of tyre, in Meters. i.e. the distance
           // travelled in one revolution

// Board Specific Calibrations
/** @brief Measured voltage on the Arduino 5V rail. */
float CAL_REFERENCE_VOLTAGE = 5; // Voltage seen on the arduino 5V rail
/** @brief Measured voltage on the internal 1.1V analog reference (AREF). */
float CAL_INTERNAL_REFERENCE_VOLTAGE =
    1.1; // Voltage seen on the internal 1.1v AREF
/** @brief Multiplier for 24V total battery voltage calculation. */
float CAL_BATTERY_TOTAL =
    6.15; // Multiplier for 24v calculation. Calculated by 24v Input divided by
          // voltage on Arduino pin A0
/** @brief Multiplier for 12V lower battery bank voltage calculation. */
float CAL_BATTERY_LOWER =
    6.15; // Multiplier for 12v calculation. Calculated by 12V Input divided by
          // voltage on Arduino pin A7
/** @brief Current multiplier for current sensor calibration. */
float CAL_CURRENT =
    37.55; // Current Multiplier - See documentation for calibration method

/** @brief Temperature sensor type: 0 for Thermistors, 1 for Linear sensors. */
int CAL_LINEAR_TEMPERATURE =
    0; // 0 for Thermistors, 1 for a Linear temperature sensor.
// Thermistor Calibration Values
/** @brief Steinhart-Hart constant A for Thermistor 1. */
float CAL_THERM1_A =
    0.001871300068; // Linear multiplier OR Steinhart-Hart constants - See
                    // documentation for calibration method
/** @brief Steinhart-Hart constant B for Thermistor 1. */
float CAL_THERM1_B = 0.00009436080271;
/** @brief Steinhart-Hart constant C for Thermistor 1. */
float CAL_THERM1_C = 0.0000007954800125;
/** @brief Steinhart-Hart constant A for Thermistor 2. */
float CAL_THERM2_A =
    0.001871300068; // Linear multiplier OR Steinhart-Hart constants - See
                    // documentation for calibration method
/** @brief Steinhart-Hart constant B for Thermistor 2. */
float CAL_THERM2_B = 0.00009436080271;
/** @brief Steinhart-Hart constant C for Thermistor 2. */
float CAL_THERM2_C = 0.0000007954800125;

// Throttle calibrations
/** @brief Enable/Disable driving the motor output pin. */
int CAL_THROTTLE_OUTPUT_EN = 1;
/** @brief Throttle input type: 1 for variable, 0 for binary (push button). */
int CAL_THROTTLE_VARIABLE =
    1; // 1 for a variable throttle, 0 for a push button (on/off) throttle.
/** @brief Enable/Disable throttle ramp-up logic. */
int CAL_THROTTLE_RAMP =
    0; // 1 to enable. A simple implementation of a throttle ramp up
/** @brief Low throttle voltage threshold (below this is 0%). */
int CAL_THROTTLE_LOW = 1; // This voltage and below is regarded as 0% throttle
/** @brief High throttle voltage threshold (above this is 100%). */
int CAL_THROTTLE_HIGH =
    4; // This voltage and above is regarded as 100% throttle

// _______________________________________________________________
// Experimental Area!!
// Enabling these might break things :)

/** @brief Experimental high-res RPM calculation logic. */
int CAL_USE_IMPROVED_RPM_CALCULATION =
    0; // Will work best with one magnet on the motor shaft
/** @brief Experimental high-res speed calculation logic. */
int CAL_USE_IMPROVED_SPEED_CALCULATION =
    0; // Will work best with one magnet on the wheel
