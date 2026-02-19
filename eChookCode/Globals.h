/**
 * @file Globals.h
 * @brief Global variables and constants for the eChook Telemetry Board.
 */

// Bluetooth Data Identifiers
/** @brief Data identifier for speed (m/s). */
const char SPEED_ID = 's';
/** @brief Data identifier for motor RPM. */
const char MOTOR_ID = 'm';
/** @brief Data identifier for battery current (Amps). */
const char CURRENT_ID = 'i';
/** @brief Data identifier for total battery voltage. */
const char VOLTAGE_ID = 'v';
/** @brief Data identifier for lower battery bank voltage. */
const char VOLTAGE_LOWER_ID = 'w';
/** @brief Data identifier for throttle input (%). */
const char THROTTLE_INPUT_ID = 't';
/** @brief Data identifier for throttle output (%). */
const char THROTTLE_OUTPUT_ID = 'd';
/** @brief Data identifier for throttle input voltage (V). */
const char THROTTLE_VOLTAGE_ID = 'T';
/** @brief Data identifier for temperature sensor 1 (°C). */
const char TEMP1_ID = 'a';
/** @brief Data identifier for temperature sensor 2 (°C). */
const char TEMP2_ID = 'b';
/** @brief Data identifier for internal processor temperature (°C). */
const char TEMP3_ID = 'c';
/** @brief Data identifier for launch mode state. */
const char LAUNCH_MODE_ID = 'L';
/** @brief Data identifier for UI cycle button state. */
const char CYCLE_VIEW_ID = 'C';
/** @brief Data identifier for calculated gear ratio. */
const char GEAR_RATIO_ID = 'r';
/** @brief Data identifier for brake pedal state. */
const char BRAKE_PRESSED_ID = 'B';
/** @brief Data identifier for measured ADC reference voltage. */
const char REF_VOLTAGE_ID = 'V';

/** @brief The reference voltage (in Volts) used for ADC calculations. */
float referenceVoltage = 0;

// Read in values:
/** @brief Most recently measured total battery voltage (V). */
float batteryVoltageTotal = 0;
/** @brief Most recently measured lower battery bank voltage (V). */
float batteryVoltageLower = 0;
/** @brief Calculated throttle output percentage (%). */
float throttleOutput = 0;
/** @brief Most recently read throttle input percentage (%). */
float throttleIn = 0;
/** @brief Most recently read throttle voltage from the ADC (V). */
float throttleV = 0;
/** @brief Most recently measured battery current (Amps). */
float current = 0;
/** @brief Most recently calculated motor RPM. */
float motorRPM = 0;
/** @brief Most recently calculated wheel RPM. Stored globally for gear ratio
 * calculations. */
float wheelRPM = 0;
/** @brief Most recently calculated wheel speed (m/s). */
float wheelSpeed = 0;
/** @brief Most recently calculated gear ratio (Motor RPM / Wheel RPM). */
float gearRatio = 0;
/** @brief Most recently read temperature from sensor 1 (°C). */
float tempOne = 0;
/** @brief Most recently read temperature from sensor 2 (°C). */
float tempTwo = 0;
/** @brief Most recently read internal processor temperature (°C). */
float tempThree = 0;
/** @brief Current brake state (0 = released, 1 = pressed). */
uint8_t brake = 0;

/** @brief Flag set to 1 when the board is in configuration mode. */
uint8_t inConfig = 0;

// Interrupt Variables.
/** @brief Counter for motor pulses detected by interrupt. */
volatile unsigned long motorPoll = 0;
/** @brief Counter for wheel pulses detected by interrupt. */
volatile unsigned long wheelPoll = 0;

// Tip: Any variables that are being used in an Interrupt Service Routine need
// to be declared as volatile. This ensures that each time the variable is
// accessed it is the master copy in RAM rather than a cached version within the
// CPU. This way the main loop and the ISR variables are always in sync

/*
 * Sensor Smoothing Implementation Notes:
 * For some signals it is desirable to average them over a longer period than is
 * possible using hardware. We use a moving average: new readings are added to
 * an array, replacing the oldest. For a 250ms update rate, a 4-item array
 * averages over 1 second.
 */

// Current Smoothing Variables:
/** @brief Number of samples to include in the current moving average. */
const uint8_t currentSmoothingSetting = 4;
/** @brief Buffer for storing current samples for moving average. */
float currentSmoothingArray[currentSmoothingSetting];
/** @brief Index for the next current sample to be stored in the smoothing
 * array. */
uint8_t currentSmoothingCount = 0;

// ISR Wheel and Motor Speed Variables
/** @brief Timestamp of the last motor pulse interrupt (microseconds). */
volatile unsigned long lastMotorPollTime = 0;
/** @brief Time interval between the two most recent motor pulses
 * (microseconds). */
volatile unsigned long lastMotorInterval = 0;
/** @brief Timestamp of the last wheel pulse interrupt (microseconds). */
volatile unsigned long lastWheelPollTime = 0;
/** @brief Time interval between the two most recent wheel pulses
 * (microseconds). */
volatile unsigned long lastWheelInterval = 0;
/** @brief Flag set by wheel interrupt indicating a new pulse has been
 * processed. */
volatile bool newSpeedSignal = 0;
/** @brief Flag set by motor interrupt indicating a new pulse has been
 * processed. */
volatile bool newMotorSignal = 0;

// Smoothing for RPM and Speed
/** @brief Array size for RPM and speed moving averages. */
const int smoothingSize = 4;
/** @brief Buffer for motor RPM moving average. */
float motorRPMSmoothing[smoothingSize];
/** @brief Index for the next motor RPM sample in the smoothing buffer. */
int motorSmoothingIndex = 0;
/** @brief Buffer for wheel speed moving average. */
float wheelSpeedSmoothing[smoothingSize];
/** @brief Index for the next wheel speed sample in the smoothing buffer. */
int wheelSmoothingIndex = 0;
