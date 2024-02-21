// ===============================================
// eChook Telemetry Board Code
// Target: ATMega 328, Arduino Nano, Arduino Nano Every.
//
// Authors: ROWAN GRIFFIN, IAN COOPER
// Lisence: MIT
// Support: echook.boards.net
// Documentation: docs.eChook.uk
// ===============================================

const float CODE_VERSION = 2.01;


//Includes
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

int DEBUG_MODE = 0; // If debug mode is on, no data will be sent via bluetooth. This is to make any debug messages easier to see.

// Initialise button debounces
Bounce launchButtonDebounce = Bounce();
Bounce cycleButtonDebounce  = Bounce();
Bounce brakeButtonDebounce  = Bounce();


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
