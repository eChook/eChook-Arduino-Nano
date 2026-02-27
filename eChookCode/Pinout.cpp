/**
 * @file Pinout.cpp
 * @brief Instantiation of reassignable hardware pins for the eChook Telemetry Board.
 *
 * This file contains the actual definitions (memory allocations) for the
 * pin variables declared as extern in Pinout.h. This prevents multiple definition
 * linker errors when Pinout.h is included in multiple source files.
 */

#include "Pinout.h"
#include <Arduino.h>

int TEMP1_IN_PIN = A5;
int TEMP2_IN_PIN = A4;
