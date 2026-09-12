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
/** @brief Motor pulse debounce floor, in microseconds. A floor on the gap between
 *  pulses, so it sets a ceiling of 500 pulses/sec - 30k RPM with one magnet, and
 *  proportionally less with more. */
constexpr unsigned long motorDebounceUs = 2000;
/** @brief Wheel pulse debounce floor, in microseconds - a ceiling of 100 pulses/sec. */
constexpr unsigned long wheelDebounceUs = 10000;
/** @brief Quiet period after which the motor is read as stopped, in microseconds. The
 *  motor spins far faster than the wheel, so one second of quiet is plenty. */
constexpr unsigned long motorTimeoutUs = 1000000;
/** @brief Quiet period after which the wheel is read as stopped, in microseconds. Longer
 *  than the motor's, since a slow-rolling wheel with one magnet can legitimately go
 *  seconds between pulses. */
constexpr unsigned long wheelTimeoutUs = 3000000;

/** @brief Timestamp of the last motor pulse interrupt (microseconds). */
extern volatile unsigned long lastMotorPollTime;
/** @brief Timestamp of the last wheel pulse interrupt (microseconds). */
extern volatile unsigned long lastWheelPollTime;
/** @brief Total of all accepted motor intervals since the last read, in microseconds.
 *  Accumulated in the ISR so every pulse contributes, not just the most recent. */
extern volatile unsigned long motorAccumUs;
/** @brief Number of accepted motor pulses making up motorAccumUs. */
extern volatile uint16_t motorPulseCount;
/** @brief Total of all accepted wheel intervals since the last read, in microseconds. */
extern volatile unsigned long wheelAccumUs;
/** @brief Number of accepted wheel pulses making up wheelAccumUs. */
extern volatile uint16_t wheelPulseCount;

// Smoothing for RPM and Speed
/** @brief Number of read windows a moving average can hold.
 *  Defined here rather than in Globals.cpp so that it is a compile-time constant at
 *  every use site: the ring index wraps with % smoothingSize, and an extern const would
 *  leave that as a real runtime division on a core with no divide instruction. */
constexpr uint8_t smoothingSize = 4;
/** @brief Longest stretch of time a moving average may span, in microseconds.
 *  With one magnet each pulse is a whole revolution, so at low speed a pulse-counted
 *  window stretches a long way - four revolutions from rest span over two seconds, and
 *  the reported speed becomes an average of history rather than of now. Bounding the
 *  span keeps the average responsive when the car is slow and still accelerating, which
 *  is exactly where a standing start lives.
 *
 *  This is a responsiveness bound, deliberately independent of the two other numbers it
 *  interacts with: CAL_DATA_TRANSMIT_INTERVAL sets how often a window is closed, and
 *  smoothingSize only caps how many windows can be held. At the default 100ms interval
 *  four windows span 400ms, so the bound rarely binds at speed; a longer interval makes
 *  it bind sooner, which is the intent - the average still covers at most this much real
 *  time however the other two are set. See updatePulseAverage(). */
constexpr unsigned long smoothingWindowUs = 500000;

/** @brief Moving average for one pulse-counted channel - the wheel or the motor.
 *
 *  Each slot pairs the pulses counted in one read window with the microseconds those
 *  pulses spanned. Averaging sums both and divides, giving revolutions over elapsed time
 *  - the true mean. Averaging per-window rates instead would average reciprocals and
 *  bias the result high whenever the intervals vary.
 *
 *  The wheel and the motor are the same measurement problem and share one implementation;
 *  see updatePulseAverage() in eChook_Functions.ino. */
struct PulseAverage {
  /** @brief Microseconds spanned by each stored read window. */
  unsigned long windowUs[smoothingSize];
  /** @brief Pulses counted in each stored read window. */
  uint16_t windowPulses[smoothingSize];
  /** @brief Slot the next window will be written to. */
  uint8_t index;
  /** @brief Windows actually stored, up to smoothingSize. */
  uint8_t count;
  /** @brief Latest averaged rate in revolutions per second, held between pulses. */
  float revsPerSec;
};

/** @brief Moving average state for the motor shaft sensor. */
extern PulseAverage motorPulseAvg;
/** @brief Moving average state for the wheel sensor. */
extern PulseAverage wheelPulseAvg;
