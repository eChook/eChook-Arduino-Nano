// ===============================================
// eChook Telemetry Board Code
// Target: Arduino Nano (ATMega 328) OR Arduino Nano Every.
//
// Authors: ROWAN GRIFFIN, IAN COOPER
// Lisence: MIT
// Support: echook.boards.net
// Documentation: docs.eChook.uk
// ===============================================

const float CODE_VERSION = 2.05; // The version of the code running on the board. This is used to check compatibility with the online configuration editor.

// Includes
#include <math.h>
#include <Bounce2.h>
#include "Calibration.h"
#include "Pinout.h"
#include "Globals.h"

// Detects if board is an Arduino Nano Every, sets flags to change code accordingly.
#if defined(__AVR_ATmega4809__)
#define NANO_EVERY
// References Serial1 to SerialA for the Arduino Nano Every
HardwareSerial &SerialA = Serial1;
#else
// References Serial to SerialA for the Arduino Nano 328p
HardwareSerial &SerialA = Serial;
#endif

/* Option Flags for PCB V2 SMD (Preassembled kit)
 * ---------------------------------------
 * Cutting the left hand link and bridging the centre pad to the right pad on JP1 *AND* JP2 reroutes the temperature sensors
 * to Arduino pins A1 and A6, freeing up pins A4 and A5 to be used as I2C SDA(A4) and SCL(A5) on the expansion header.
 * Default state is commented out.
 */
// #define JUMPER_I2C

/* On PCB V2.x if JP3 is bridged (Default) it enables automatic HC-05 programming using pin D4. 
 * JP3 can be cut to allow use of D4 on the expansion header - in which case comment out JUMPER_BT_EN below.
 * Default state for PCB V2.x is uncommented 
 * Default state for PCB V1.x is commented out.
 */
#define JUMPER_BT_EN
// ---------------------------------------

int DEBUG_MODE = 0; // If debug mode is on, no data will be sent via bluetooth. This is to make any debug messages easier to see. The online configuratin editor WONT work if debug mode is on.

// Initialise button debounces
Bounce launchButtonDebounce = Bounce();
Bounce cycleButtonDebounce = Bounce();
Bounce brakeButtonDebounce = Bounce();

void setup()
{
  eChookSetup(); // Runs all the setup routines for the eChook code

  // Any setup code you might need to add can be added here:
}

void loop()
{
  eChookRoutinesUpdate(); // This function runs all the code for the eChook
                          // to read and send data on time. Find the code in
                          // the eChook_Functions.ino file

  buttonChecks(); // Checks buttons each loop, debounces and sends any changes in state

  // Any new code you want to add to loop you can add below, HOWEVER avoid using
  // blocking code (anything that takes a long time to complete or uses 'delay()'
  // as it will cause timing errors with the eChook code.
}
