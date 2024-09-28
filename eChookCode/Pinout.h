/** ================================== */
/** Pin Definitions                    */
/** ================================== */
/**  ANALOG INPUT PINS */
const int VBATT_IN_PIN        = A0;    // Analog input pin for battery voltage
const int AMPS_IN_PIN         = A2;    // Analog input pin for current draw
const int THROTTLE_IN_PIN     = A3;    // Analog input pin for the throttle
int TEMP1_IN_PIN              = A5;    // Analog input pin for temp sensor 1 - not constant as can be reassigned by JUMPER_I2C Flag
int TEMP2_IN_PIN              = A4;    // Analog input pin for temp sensor 2 - not constant as can be reassigned by JUMPER_I2C Flag
const int VBATT1_IN_PIN       = A7;    // Analog input pin for the lower battery voltage (battery between ground and 12V)

/** DIGITAL INPUT PINS */
const int BRAKE_IN_PIN        = 7;      // Digital input pin for launch mode button
const int LAUNCH_BTN_IN_PIN   = 8;      // Digital input pin for launch mode button
const int CYCLE_BTN_IN_PIN    = 12;     // Digital input pin for cycle view button

/** DIGITAL INTERRUPT PINS */
const int MOTOR_RPM_PIN       = 2;      // Digital interrupt for motor pulses
const int WHEEL_RPM_PIN       = 3;      // Digital interrupt for wheel pulses

/** DIGITAL AND PWM OUTPUT PINS */
const int MOTOR_OUT_PIN       = 5;     // PWM output to the motor
const int BT_EN_PIN           = 4;     // Only used in PCB V2 if JP3 is bridged and code flag is enabled. Puts the HC-05 in programming mode.
