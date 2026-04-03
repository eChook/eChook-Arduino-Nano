/**
 * @file Calibration.cpp
 * @brief Instantiation of calibration variables for the eChook Telemetry Board.
 *
 * This file contains the actual definitions (memory allocations) and default values
 * for the calibration variables declared as extern in Calibration.h. This ensures
 * they are only defined once across the entire compiled program, preventing
 * multiple definition linker errors.
 */

#include "Calibration.h"

uint8_t FORCE_USE_HARDCODED_CAL = 0;
uint8_t CAL_USE_EEPROM = 1;

char CAL_BT_NAME[32] = "eChook";
char CAL_BT_PASSWORD[32] = "1234";
long CAL_BT_BAUDRATE = 115200;

unsigned long CAL_DATA_TRANSMIT_INTERVAL = 100;

int CAL_WHEEL_MAGNETS = 2;
int CAL_MOTOR_MAGNETS = 1;
float CAL_WHEEL_CIRCUMFERENCE = 1.178;

float CAL_REFERENCE_VOLTAGE = 5;
float CAL_INTERNAL_REFERENCE_VOLTAGE = 1.1;
float CAL_BATTERY_TOTAL = 6.15;
float CAL_BATTERY_LOWER = 6.15;
float CAL_CURRENT = 37.55;

int CAL_LINEAR_TEMPERATURE = 0;
float CAL_THERM1_A = 0.001871300068;
float CAL_THERM1_B = 0.00009436080271;
float CAL_THERM1_C = 0.0000007954800125;
float CAL_THERM2_A = 0.001871300068;
float CAL_THERM2_B = 0.00009436080271;
float CAL_THERM2_C = 0.0000007954800125;

int CAL_THROTTLE_OUTPUT_EN = 1;
int CAL_THROTTLE_VARIABLE = 1;
int CAL_THROTTLE_RAMP = 0;
int CAL_THROTTLE_LOW = 1;
int CAL_THROTTLE_HIGH = 4;

int CAL_USE_IMPROVED_RPM_CALCULATION = 0;
int CAL_USE_IMPROVED_SPEED_CALCULATION = 0;
