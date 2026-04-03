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
#include <Arduino.h>

extern uint8_t FORCE_USE_HARDCODED_CAL;

/** @brief Enable reading/writing of calibration values to EEPROM. */
extern uint8_t CAL_USE_EEPROM;

// Bluetooth Settings
/** @brief The bluetooth name for the car. */
extern char CAL_BT_NAME[32];
/** @brief The bluetooth pairing password. */
extern char CAL_BT_PASSWORD[32];
/** @brief Serial communication baud rate. Must match both BT module and Arduino settings. */
extern long CAL_BT_BAUDRATE; // Baud Rate to run at. Must match Arduino's baud rate.

// Data Read and Transmit Interval:
/** @brief Interval (in ms) between data transmissions. */
extern unsigned long CAL_DATA_TRANSMIT_INTERVAL; // transmit interval in ms

// Car Specific Settings
/** @brief Number of magnets mounted on the wheel for speed sensing. */
extern int CAL_WHEEL_MAGNETS; // Number of magnets on wheel
/** @brief Number of magnets on the motor shaft for RPM sensing. */
extern int CAL_MOTOR_MAGNETS; // Number of magnets on motor shaft for hall effect sensor
/** @brief Outer circumference of the tyre in meters. Used for speed calculation. */
extern float CAL_WHEEL_CIRCUMFERENCE; // Outer circumference of tyre, in Meters. i.e. the distance travelled in one revolution

// Board Specific Calibrations
/** @brief Measured voltage on the Arduino 5V rail. */
extern float CAL_REFERENCE_VOLTAGE; // Voltage seen on the arduino 5V rail
/** @brief Measured voltage on the internal 1.1V analog reference (AREF). */
extern float CAL_INTERNAL_REFERENCE_VOLTAGE; // Voltage seen on the internal 1.1v AREF
/** @brief Multiplier for 24V total battery voltage calculation. */
extern float CAL_BATTERY_TOTAL; // Multiplier for 24v calculation. Calculated by 24v Input divided by voltage on Arduino pin A0
/** @brief Multiplier for 12V lower battery bank voltage calculation. */
extern float CAL_BATTERY_LOWER; // Multiplier for 12v calculation. Calculated by 12V Input divided by voltage on Arduino pin A7
/** @brief Current multiplier for current sensor calibration. */
extern float CAL_CURRENT; // Current Multiplier - See documentation for calibration method

/** @brief Temperature sensor type: 0 for Thermistors, 1 for Linear sensors. */
extern int CAL_LINEAR_TEMPERATURE; // 0 for Thermistors, 1 for a Linear temperature sensor.
// Thermistor Calibration Values
/** @brief Steinhart-Hart constant A for Thermistor 1. */
extern float CAL_THERM1_A; // Linear multiplier OR Steinhart-Hart constants - See documentation for calibration method
/** @brief Steinhart-Hart constant B for Thermistor 1. */
extern float CAL_THERM1_B;
/** @brief Steinhart-Hart constant C for Thermistor 1. */
extern float CAL_THERM1_C;
/** @brief Steinhart-Hart constant A for Thermistor 2. */
extern float CAL_THERM2_A; // Linear multiplier OR Steinhart-Hart constants - See documentation for calibration method
/** @brief Steinhart-Hart constant B for Thermistor 2. */
extern float CAL_THERM2_B;
/** @brief Steinhart-Hart constant C for Thermistor 2. */
extern float CAL_THERM2_C;

// Throttle calibrations
/** @brief Enable/Disable driving the motor output pin. */
extern int CAL_THROTTLE_OUTPUT_EN;
/** @brief Throttle input type: 1 for variable, 0 for binary (push button). */
extern int CAL_THROTTLE_VARIABLE; // 1 for a variable throttle, 0 for a push button (on/off) throttle.
/** @brief Enable/Disable throttle ramp-up logic. */
extern int CAL_THROTTLE_RAMP; // 1 to enable. A simple implementation of a throttle ramp up
/** @brief Low throttle voltage threshold (below this is 0%). */
extern int CAL_THROTTLE_LOW; // This voltage and below is regarded as 0% throttle
/** @brief High throttle voltage threshold (above this is 100%). */
extern int CAL_THROTTLE_HIGH; // This voltage and above is regarded as 100% throttle

// _______________________________________________________________
// Experimental Area!!
// Enabling these might break things :)

/** @brief Experimental high-res RPM calculation logic. */
extern int CAL_USE_IMPROVED_RPM_CALCULATION; // Will work best with one magnet on the motor shaft
/** @brief Experimental high-res speed calculation logic. */
extern int CAL_USE_IMPROVED_SPEED_CALCULATION; // Will work best with one magnet on the wheel
