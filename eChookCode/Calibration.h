/*
 * This file contains all the values used to calibrate your eChook board.
 * By seperating these it makes it possible to update to newer code without
 * having to copy your calibrations across - just make sure you keep this file!
 *
 * The values provided by default are for the eChook's team dev board, but each
 * board is slightly different. These defaults will give readings that are reasonable,
 * but for extra accuracy, see the documentation to calibrate your own board.
 *
 * Every varialbe in this file begins with CAL_ so that it is identifiable in
 * the main code as being defined in this file.
 *
 */

//If you intend to use the online configuration/calibration tool, set this to 0, to use these hard coded settings, set it to 1
uint8_t FORCE_USE_HARDCODED_CAL = 0;

uint8_t CAL_USE_EEPROM = 1;

//Bluetooth Settings
String CAL_BT_NAME     = "eChook"; // Whatever you want to name your car's bluetooth
String CAL_BT_PASSWORD = "1234";
long CAL_BT_BAUDRATE         = 115200;            // Baud Rate to run at. Must match Arduino's baud rate.

// Data Read and Transmit Interval:
unsigned long CAL_DATA_TRANSMIT_INTERVAL     = 100;         // transmit interval in ms

// Car Specific Settings
int CAL_WHEEL_MAGNETS        = 2;       //Number of magnets on wheel
int CAL_MOTOR_MAGNETS        = 1;       // Number of magnets on motor shaft for hall effect sensor
float CAL_WHEEL_CIRCUMFERENCE  = 1.178;     //Outer circumference of tyre, in Meters. i.e. the distance travelled in one revolution

//Board Specific Calibrations
float CAL_REFERENCE_VOLTAGE   = 5;     // Voltage seen on the arduino 5V rail
float CAL_INTERNAL_REFERENCE_VOLTAGE   = 1.1;     // Voltage seen on the internal 1.1v AREF
float CAL_BATTERY_TOTAL       = 6.15;  // Multiplier for 24v calculation. Calculated by 24v Input divided by voltage on Arduino pin A0
float CAL_BATTERY_LOWER       = 6.15; // Multiplier for 12v calculation. Calculated by 12V Input divided by voltage on Arduino pin A7
float CAL_CURRENT             = 37.55; // Current Multiplier - See documentation for calibration method

int CAL_LINEAR_TEMPERATURE = 0; //0 for Thermistors, 1 for a Linear temperature sensor.
//Thermistor Calibration Values
float CAL_THERM1_A = 0.001871300068; //Linear multiplier OR Steinhart-Hart constants - See documentation for calibration method
float CAL_THERM1_B = 0.00009436080271;
float CAL_THERM1_C = 0.0000007954800125;
float CAL_THERM2_A = 0.001871300068; //Linear multiplier OR Steinhart-Hart constants - See documentation for calibration method
float CAL_THERM2_B = 0.00009436080271;
float CAL_THERM2_C = 0.0000007954800125;

//Throttle calibrations
int CAL_THROTTLE_OUTPUT_EN = 1;
int CAL_THROTTLE_VARIABLE = 1; // 1 for a variable throttle, 0 for a push button (on/off) throttle.
int CAL_THROTTLE_RAMP = 0; // 1 to enable. A simple implementation of a throttle ramp up
int CAL_THROTTLE_LOW = 1; // This voltage and below is regarded as 0% throttle
int CAL_THROTTLE_HIGH = 4; // This voltage and above is regarded as 100% throttle

// _______________________________________________________________
// Experimental Area!!
// Enabling these might break things :)

// These may break if the vehicle isn't moving
int CAL_USE_IMPROVED_RPM_CALCULATION = 0; // Will work best with one magnet on the motor shaft
int CAL_USE_IMPROVED_SPEED_CALCULATION = 0; // Will work best with one magnet on the wheel
