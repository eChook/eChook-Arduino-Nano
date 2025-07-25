// Bluetooth Data Identifiers
// If these are altered the data will no longer be interpreted correctly by the phone.
const char SPEED_ID            = 's'; // Speed in m/s
const char MOTOR_ID            = 'm'; // Motor RPM
const char CURRENT_ID          = 'i'; // Current in Amps
const char VOLTAGE_ID          = 'v'; // Total Battery Voltage in V
const char VOLTAGE_LOWER_ID    = 'w'; // Lower Vattery Voltage in V
const char THROTTLE_INPUT_ID   = 't'; // Throttle Input percentage
const char THROTTLE_OUTPUT_ID  = 'd'; // Throttle Output Percentage
const char THROTTLE_VOLTAGE_ID = 'T'; // Throttle input voltage
const char TEMP1_ID            = 'a'; // Temp 1 in *C
const char TEMP2_ID            = 'b'; // Temp 2 in *C
const char TEMP3_ID            = 'c'; // Internal Temp in *C
const char LAUNCH_MODE_ID      = 'L'; // Launch Mode (Start) button
const char CYCLE_VIEW_ID       = 'C'; // Cycle View (Scrn) Button
const char GEAR_RATIO_ID       = 'r'; // Calculated Gear Ratio
const char BRAKE_PRESSED_ID    = 'B'; // Brake on/off
const char REF_VOLTAGE_ID      = 'V'; // ADC Reference Voltage

//Measured Reference Voltage:
float referenceVoltage = 0;


// Read in values:
float batteryVoltageTotal   = 0; // Total battery voltage in Volts
float batteryVoltageLower   = 0; // Lower battery voltage in Volts
float throttleOutput        = 0; // Throttle output in Percentage
float throttleIn            = 0; // Throttle input in Percentage
float throttleV             = 0; // Throttle ADC input voltage
float current               = 0; // Current in Amps
float motorRPM              = 0; // Motor RPM
float wheelRPM              = 0; // This is stored globally for use in the Gear Ratio calculation
float wheelSpeed            = 0; // Wheel speed in m/s
float gearRatio             = 0; // Calculated Gear Ratio
float tempOne               = 0; // Temperature 1 in *C
float tempTwo               = 0; // Temperature 2 in *C
float tempThree             = 0; // Internal Temperature in *C
uint8_t brake               = 0; // Brake pressed or not, 0 = not pressed, 1 = pressed

// Loop counter used for longer interval functions


// Flag to indicate web configuration page is being used:
uint8_t inConfig = 0;

// Interrupt Variables.
volatile unsigned long motorPoll      = 0;
volatile unsigned long wheelPoll      = 0;
// Tip: Any variables that are being used in an Interrupt Service Routine need to be declared as volatile. This ensures
// that each time the variable is accessed it is the master copy in RAM rather than a cached version within the CPU.
// This way the main loop and the ISR variables are always in sync


/**
 * For some signals it is desireable to average them over a longer period than is possible using the hardware
 * components on the board. To do this we average the last few readings taken by the Arduino.
 * When a new reading is taken it is added to an array. Each new reading takes the place of the oldest reading
 * in the array, so the array always contains the last X number of readings, where X is the size of the array.
 * In our case, for a reading being updated every 250ms, an array of length 4 would average the readings over
 * the last second.
 * To implement this in code, two golbal variables are needed per filter:
 *  > The Array - to store the last X number of readings
 *	> A Count - This dictates which position in the array a new reading goes to and loops between 0 and array length - 1
 * To make it simple to alter the length of time to average over it is good practice to define the max count
 * value and aray length as a global const too.
 */

//Current Smoothing Variables:

const uint8_t currentSmoothingSetting = 4; //current is sampled every 250ms, therefore 4 makes 1s of smoothing
float currentSmoothingArray[currentSmoothingSetting];
uint8_t currentSmoothingCount = 0;


// ISR Wheel and Motor Speed Variables
volatile unsigned long lastMotorPollTime = 0;
volatile unsigned long lastMotorInterval = 0;
volatile unsigned long lastWheelPollTime = 0;
volatile unsigned long lastWheelInterval = 0;
volatile bool newSpeedSignal = 0;
volatile bool newMotorSignal = 0;
