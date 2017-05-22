/*
* This file contains all the values used to calibrate your eChook board.
* By seperating these it makes it possible to update to newer code without having to copy your
* calibrations across - just make sure you keep this file!
*
* The values provided by default are for the eChooks team dev board, but each board is different.
* these will give readings in the right ballpark, but for accuracy, see the documentation to calibrate
* your own board.
*
* Every varialbe in this file begins with CAL_ so that it is identifiable in the main code
*
*/

//Calibration Variables

const String CAL_BT_NAME = "eChook";
const String CAL_BT_PASSWORD = "1234";


const float     CAL_REFERENCE_VOLTAGE    = 5;
const uint8_t   CAL_WHEEL_MAGENTS        = 5;
const uint8_t   CAL_MOTOR_MAGNETS        = 6; // Number of magnets on motor shaft for hall effect sensor
const float    CAL_WHEEL_CIRCUMFERENCE  = 1.178; //Outer circumference of tyre, in Meters. i.e. the distance travelled in one revolution

const float CAL_BATTERY_TOTAL   = 6.15; 
const float CAL_BATTERY_LOWER   = 3.071;
const float CAL_CURRENT         = 37.55;