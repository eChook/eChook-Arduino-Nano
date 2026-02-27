/**
 * @file Globals.h
 * @brief Global variables and constants for the eChook Telemetry Board.
 */

// Bluetooth Data Identifiers
#include <Arduino.h>

/** @brief Data identifier for speed (m/s). */
extern const char SPEED_ID;
/** @brief Data identifier for motor RPM. */
extern const char MOTOR_ID;
/** @brief Data identifier for battery current (Amps). */
extern const char CURRENT_ID;
/** @brief Data identifier for total battery voltage. */
extern const char VOLTAGE_ID;
/** @brief Data identifier for lower battery bank voltage. */
extern const char VOLTAGE_LOWER_ID;
/** @brief Data identifier for throttle input (%). */
extern const char THROTTLE_INPUT_ID;
/** @brief Data identifier for throttle output (%). */
extern const char THROTTLE_OUTPUT_ID;
/** @brief Data identifier for throttle input voltage (V). */
extern const char THROTTLE_VOLTAGE_ID;
/** @brief Data identifier for temperature sensor 1 (°C). */
extern const char TEMP1_ID;
/** @brief Data identifier for temperature sensor 2 (°C). */
extern const char TEMP2_ID;
/** @brief Data identifier for internal processor temperature (°C). */
extern const char TEMP3_ID;
/** @brief Data identifier for launch mode state. */
extern const char LAUNCH_MODE_ID;
/** @brief Data identifier for UI cycle button state. */
extern const char CYCLE_VIEW_ID;
/** @brief Data identifier for calculated gear ratio. */
extern const char GEAR_RATIO_ID;
/** @brief Data identifier for brake pedal state. */
extern const char BRAKE_PRESSED_ID;
/** @brief Data identifier for measured ADC reference voltage. */
extern const char REF_VOLTAGE_ID;

/** @brief The reference voltage (in Volts) used for ADC calculations. */
extern float referenceVoltage;

// Read in values:
/** @brief Most recently measured total battery voltage (V). */
extern float batteryVoltageTotal;
/** @brief Most recently measured lower battery bank voltage (V). */
extern float batteryVoltageLower;
/** @brief Calculated throttle output percentage (%). */
extern float throttleOutput;
/** @brief Most recently read throttle input percentage (%). */
extern float throttleIn;
/** @brief Most recently read throttle voltage from the ADC (V). */
extern float throttleV;
/** @brief Most recently measured battery current (Amps). */
extern float current;
/** @brief Most recently calculated motor RPM. */
extern float motorRPM;
/** @brief Most recently calculated wheel RPM. Stored globally for gear ratio calculations. */
extern float wheelRPM;
/** @brief Most recently calculated wheel speed (m/s). */
extern float wheelSpeed;
/** @brief Most recently calculated gear ratio (Motor RPM / Wheel RPM). */
extern float gearRatio;
/** @brief Most recently read temperature from sensor 1 (°C). */
extern float tempOne;
/** @brief Most recently read temperature from sensor 2 (°C). */
extern float tempTwo;
/** @brief Most recently read internal processor temperature (°C). */
extern float tempThree;
/** @brief Current brake state (0 = released, 1 = pressed). */
extern uint8_t brake;

/** @brief Flag set to 1 when the board is in configuration mode. */
extern uint8_t inConfig;

// Interrupt Variables.
/** @brief Counter for motor pulses detected by interrupt. */
extern volatile unsigned long motorPoll;
/** @brief Counter for wheel pulses detected by interrupt. */
extern volatile unsigned long wheelPoll;

// Tip: Any variables that are being used in an Interrupt Service Routine need
// to be declared as volatile. This ensures that each time the variable is
// accessed it is the master copy in RAM rather than a cached version within the CPU. This way the main loop and the ISR variables are always in sync

/*
 * Sensor Smoothing Implementation Notes:
 * For some signals it is desirable to average them over a longer period than is
 * possible using hardware. We use a moving average: new readings are added to
 * an array, replacing the oldest. For a 250ms update rate, a 4-item array
 * averages over 1 second.
 */

// Current Smoothing Variables:
/** @brief Number of samples to include in the current moving average. */
extern const uint8_t currentSmoothingSetting;
/** @brief Buffer for storing current samples for moving average. */
extern float currentSmoothingArray[];
/** @brief Index for the next current sample to be stored in the smoothing array. */
extern uint8_t currentSmoothingCount;

// ISR Wheel and Motor Speed Variables
/** @brief Timestamp of the last motor pulse interrupt (microseconds). */
extern volatile unsigned long lastMotorPollTime;
/** @brief Time interval between the two most recent motor pulses (microseconds). */
extern volatile unsigned long lastMotorInterval;
/** @brief Timestamp of the last wheel pulse interrupt (microseconds). */
extern volatile unsigned long lastWheelPollTime;
/** @brief Time interval between the two most recent wheel pulses (microseconds). */
extern volatile unsigned long lastWheelInterval;
/** @brief Flag set by wheel interrupt indicating a new pulse has been processed. */
extern volatile bool newSpeedSignal;
/** @brief Flag set by motor interrupt indicating a new pulse has been processed. */
extern volatile bool newMotorSignal;

// Smoothing for RPM and Speed
/** @brief Array size for RPM and speed moving averages. */
extern const int smoothingSize;
/** @brief Buffer for motor RPM moving average. */
extern float motorRPMSmoothing[];
/** @brief Index for the next motor RPM sample in the smoothing buffer. */
extern int motorSmoothingIndex;
/** @brief Buffer for wheel speed moving average. */
extern float wheelSpeedSmoothing[];
/** @brief Index for the next wheel speed sample in the smoothing buffer. */
extern int wheelSmoothingIndex;
