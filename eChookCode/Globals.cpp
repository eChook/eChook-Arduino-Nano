/**
 * @file Globals.cpp
 * @brief Instantiation of global variables and constants for the eChook Telemetry Board.
 *
 * This file contains the actual definitions (memory allocations) for the global
 * variables declared in Globals.h. This ensures they are only defined once across
 * the entire compiled program, preventing multiple definition linker errors.
 */

#include "Globals.h"

const char SPEED_ID = 's';
const char MOTOR_ID = 'm';
const char CURRENT_ID = 'i';
const char VOLTAGE_ID = 'v';
const char VOLTAGE_LOWER_ID = 'w';
const char THROTTLE_INPUT_ID = 't';
const char THROTTLE_OUTPUT_ID = 'd';
const char THROTTLE_VOLTAGE_ID = 'T';
const char TEMP1_ID = 'a';
const char TEMP2_ID = 'b';
const char TEMP3_ID = 'c';
const char LAUNCH_MODE_ID = 'L';
const char CYCLE_VIEW_ID = 'C';
const char GEAR_RATIO_ID = 'r';
const char BRAKE_PRESSED_ID = 'B';
const char REF_VOLTAGE_ID = 'V';

float referenceVoltage = 0;

float batteryVoltageTotal = 0;
float batteryVoltageLower = 0;
float throttleOutput = 0;
float throttleIn = 0;
float throttleV = 0;
float current = 0;
float motorRPM = 0;
float wheelSpeed = 0;
float gearRatio = 0;
float tempOne = 0;
float tempTwo = 0;
float tempThree = 0;
uint8_t brake = 0;

uint8_t inConfig = 0;

volatile unsigned long motorPoll = 0;
volatile unsigned long wheelPoll = 0;

const uint8_t currentSmoothingSetting = 4;
float currentSmoothingArray[currentSmoothingSetting];
uint8_t currentSmoothingCount = 0;

volatile unsigned long lastMotorPollTime = 0;
volatile unsigned long lastWheelPollTime = 0;
volatile unsigned long motorAccumUs = 0;
volatile uint16_t motorPulseCount = 0;
volatile unsigned long wheelAccumUs = 0;
volatile uint16_t wheelPulseCount = 0;

PulseAverage motorPulseAvg;
PulseAverage wheelPulseAvg;
