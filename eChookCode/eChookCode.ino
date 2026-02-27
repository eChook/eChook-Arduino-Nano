/**
 * @file eChookCode.ino
 * @brief Main entry point for the eChook Telemetry Board.
 * @author Rowan Griffin, Ian Cooper
 * @license MIT
 * @see echook.boards.net
 * @see docs.eChook.uk
 *
 * Target: Arduino Nano (ATMega 328) OR Arduino Nano Every.
 */

/**
 * @brief The version of the code running on the board.
 * Used to check compatibility with the online configuration editor.
 */
const float CODE_VERSION = 2.05;

// Includes
#include "Calibration.h"
#include "Globals.h"
#include "Pinout.h"
#include <Bounce2.h>
#include <math.h>

// Hardware Abstraction Layer contains definitions for architecture specific code.
#include "Hardware.h"

#if defined(__AVR_ATmega4809__)
/** @brief Hardware serial reference for Arduino Nano Every. */
HardwareSerial &SerialA = Serial1;
#else
/** @brief Hardware serial reference for Arduino Nano 328p. */
HardwareSerial &SerialA = Serial;
#endif

/* Option Flags for PCB V2 SMD (Preassembled kit)
 * ---------------------------------------
 * Cutting the left hand link and bridging the centre pad to the right pad on
 * JP1 *AND* JP2 reroutes the temperature sensors to Arduino pins A1 and A6,
 * freeing up pins A4 and A5 to be used as I2C SDA(A4) and SCL(A5) on the
 * expansion header. Default state is commented out.
 */
// #define JUMPER_I2C

/* On PCB V2.x if JP3 is bridged (Default) it enables automatic HC-05
 * programming using pin D4. JP3 can be cut to allow use of D4 on the expansion
 * header - in which case comment out JUMPER_BT_EN below. Default state for PCB
 * V2.x is uncommented Default state for PCB V1.x is commented out.
 */
#define JUMPER_BT_EN
// ---------------------------------------

/**
 * @brief If debug mode is on, no data will be sent via bluetooth.
 * This is to make any debug messages easier to see.
 * @note The online configuration editor WONT work if debug mode is on.
 */
int DEBUG_MODE = 0;

/** @brief Debounce object for the launch button. */
Bounce launchButtonDebounce = Bounce();
/** @brief Debounce object for the cycle button. */
Bounce cycleButtonDebounce = Bounce();
/** @brief Debounce object for the brake button. */
Bounce brakeButtonDebounce = Bounce();

/**
 * @brief Standard Arduino setup function.
 * Runs all initialisation routines.
 */
void setup() {
  eChookSetup(); // Runs all the setup routines for the eChook code

  // Any setup code you might need to add can be added here:
}

/**
 * @brief Standard Arduino loop function.
 * Regularly updates sensors and checks buttons.
 */
void loop() {
  eChookRoutinesUpdate(); // This function runs all the code for the eChook to read and send data on time. Find the code in the eChook_Functions.ino file

  buttonChecks(); // Checks buttons each loop, debounces and sends any changes in state

  // Any new code you want to add to loop you can add below, HOWEVER avoid using blocking code (anything that takes a long time to complete or uses 'delay()' as it will cause timing errors with the eChook code.
}
