/*
* This file contains all the values used to calibrate your eChook board.
* By seperating these it makes it possible to update to newer code without 
* having to copy your calibrations across - just make sure you keep this file!
*
* The values provided by default are for the eChook's team dev board, but each 
* board is different. These defaults will give readings in the right ballpark, 
* but for accuracy, see the documentation to calibrate your own board.
*
* Every varialbe in this file begins with CAL_ so that it is identifiable in 
* the main code as being defined in this file.
*
*/

//Bluetooth Settings
const String CAL_BT_NAME     = "eChook"; // Whatever you want to name your car's bluetooth
const String CAL_BT_PASSWORD = "1234"; // Changing password from default "1234" tends not to work yet - apologies!

// Car Specific Settings

const int       CAL_WHEEL_MAGNETS        = 6; //Number of magnets on wheel
const int       CAL_MOTOR_MAGNETS        = 3; // Number of magnets on motor shaft for hall effect sensor
const float     CAL_WHEEL_CIRCUMFERENCE  = 1.178; //Outer circumference of tyre, in Meters. i.e. the distance travelled in one revolution

//Board Specific Calibrations
const float CAL_REFERENCE_VOLTAGE   = 5;     // Voltage seen on the arduino 5V rail
const float CAL_BATTERY_TOTAL       = 6.15;  // Multiplier for 24v calculation. Calculated by 24v Input devided by voltage on Arduino pin A0
const float CAL_BATTERY_LOWER       = 3.071; // Multiplier for 12v calculation. Calculated by 12V Input devided by voltage on Arduino pin A7
const float CAL_CURRENT             = 37.55; // Current Multiplier - See documentation for calibration method

//Board and Sensor Specific Calibrations
const float CAL_THERM_A = 0.001871300068; //Steinhart-Hart constants - See documentation for calibration method
const float CAL_THERM_B = 0.00009436080271;
const float CAL_THERM_C = 0.0000007954800125;
