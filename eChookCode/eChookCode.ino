/***** ======================================================================================================================================== *****/
/*****           ====================================================================================================================           *****/
/*****                     ================================================================================================                     *****/
/*****                                                                                                                                          *****/
/*****                                                       eChook Telemetry Board Code                                                        *****/
/*****                                                              ARDUINO NANO                                                                *****/
/*****                                                                                                                                          *****/
/*****                                                               IAN COOPER                                                                 *****/
/*****                                                             ROWAN GRIFFIN                                                                *****/
/*****                                                                BEN NAGY                                                                  *****/
/*****                                                              MATT RUDLING                                                                *****/
/*****                                                                                                                                          *****/
/*****                     ================================================================================================                     *****/
/*****           ====================================================================================================================           *****/
/***** ======================================================================================================================================== *****/



/** ================================== */
/** Includes            		           */
/** ================================== */

#include <math.h>
#include <Bounce2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Calibration.h"

/** ================================== */
/** COMPILER DEFINITIONS               */
/** ================================== */
#define arr_len( x ) (sizeof( x ) / sizeof ( *x ) )

/** ================================== */
/** BUILD OPTIONS                      */
/** ================================== */
const int DEBUG_MODE = 0; //if debug mode is on, no data will be sent via bluetooth. This is to make any debug messages easier to see.

/** ================================== */
/** LCD CONFIGURATION                  */
/** ================================== */
const bool ENABLE_LCD_DISPLAY = true; // allows enabling or disabling of the I2C
const bool LCD_HAS_FOUR_LINES = true; // does the LCD have 4 lines, if this is set to false I will assume it only has two
LiquidCrystal_I2C lcd(0x27, 16, 4); // 16,4 LCD. Use a I2C finder to find the address; although they are often are 0x27
const String LCD_FIRST_LINE = "DGS Racing"; // this is the first line that will always be displayed, change this to whatever 


/** ================================== */
/** CONSTANTS                          */
/** ================================== */
/** ___________________________________________________________________________________________________ ANALOG INPUT PINS */
const int   VBATT_IN_PIN  = A0;  // Analog input pin for battery voltage
const int   AMPS_IN_PIN = A2;  // Analog input pin for current draw
const int   THROTTLE_IN_PIN = A3;  // Analog input pin for the throttle
const int   TEMP1_IN_PIN = A5;  // Analog input pin for temp sensor 1
const int   TEMP2_IN_PIN = A4;  // Analog input pin for temp sensor 2
const int   VBATT1_IN_PIN = A7;  // Analog input pin for the lower battery voltage (battery between ground and 12V)

/** ___________________________________________________________________________________________________ DIGITAL INPUT PINS */
const int   BRAKE_IN_PIN        = 7;    // Digital input pin for launch mode button
const int   LAUNCH_BTN_IN_PIN   = 8;    // Digital input pin for launch mode button
const int   CYCLE_BTN_IN_PIN    = 12;   // Digital input pin for cycle view button

/** ___________________________________________________________________________________________________ DIGITAL INTERRUPT PINS */
const int   MOTOR_RPM_PIN       = 2;    // Digital interrupt for motor pulses
const int   WHEEL_RPM_PIN       = 3;    // Digital interrupt for wheel pulses
const int   FAN_RPM_PIN         = 13;   // Digital interrupt for motor pulses

/** ___________________________________________________________________________________________________ DIGITAL AND PWM OUTPUT PINS */
const int   MOTOR_OUT_PIN       = 5;   // PWM output to the motor
const int   LED_1_OUT_PIN       = 6;   // PWM Output for LED 1
const int   LED_2_OUT_PIN       = 9;   // PWM Output for LED 2
const int   FAN_OUT_PIN         = 11;  // PWM output to the fan(s)


/** ________________________________________________________________________________________ BLUETOOTH CONSTANTS */
/* BLUETOOTH SETUP PARAMETERS */
const String  BT_NAME           = CAL_BT_NAME;      // Name of the bluetooth module to appear on phone
const String  BT_PASSWORD       = CAL_BT_PASSWORD;  // Pairing Password
const long    BT_BAUDRATE       = 115200;           // Baud Rate to run at. Must match Arduino's baud rate.

//Bluetooth module uses hardware serial from Arduino, so Arduino Tx -> HC05 Rx, Ard Rx -> HC Tx. EN and Status are disconnected.


/** ________________________________________________________________________________________ CONSTANTS */
/* DATA TRANSMIT INTERVALS */
const unsigned long     SHORT_DATA_TRANSMIT_INTERVAL     = 250;     // transmit interval in ms

/* CURRENT */
const int               AMPSENSOR_CAL_DELAY              = 3000;    // calibration delay for current sensor (ms)




/** ___________________________________________________________________________________________________ DATA IDENTIFIERS */
/**
    If these are altered the data will no longer be read correctly by the phone.
*/

const char SPEED_ID            = 's';
const char MOTOR_ID            = 'm';
const char CURRENT_ID          = 'i';
const char VOLTAGE_ID          = 'v';
const char VOLTAGE_LOWER_ID    = 'w';
const char THROTTLE_INPUT_ID   = 't';
const char THROTTLE_ACTUAL_ID  = 'd';
const char TEMP1_ID            = 'a';
const char TEMP2_ID            = 'b';
const char TEMP3_ID            = 'c';
const char LAUNCH_MODE_ID      = 'L';
const char CYCLE_VIEW_ID       = 'C';
const char GEAR_RATIO_ID       = 'r';
const char BRAKE_PRESSED_ID    = 'B';


/** ================================== */
/** VARIABLES                          */
/** ================================== */
/** ___________________________________________________________________________________________________ TIMING VARIABLES */
unsigned long   lastShortDataSendTime       = 0;
unsigned long   lastLongDataSendTime        = 0;
unsigned long   lastWheelSpeedPollTime      = 0;    // poll interval for wheel speed
unsigned long   lastMotorSpeedPollTime      = 0;    // poll interval for wheel speed
int             currentSensorOffset         = 0;    //offset value for the current sensor
int             currentAvgLoopCount         = 0;    //Counter for current loop average
int       	  	loopCounter              		= 0;


/** ___________________________________________________________________________________________________ INTERRUPT VERIABLES */
/** Any variables that are being used in an Interrupt Service Routine need to be declared as volatile. This ensures
    that each time the variable is accessed it is the master copy in RAM rather than a cached version within the CPU.
    This way the main loop and the ISR variables are always in sync
*/
volatile unsigned long motorPoll      = 0;
volatile unsigned long wheelPoll      = 0;
volatile unsigned long fanPoll        = 0;




/** ___________________________________________________________________________________________________ BUTTON VERIABLES */
Bounce launchButtonDebounce = Bounce();
Bounce cycleButtonDebounce  = Bounce();
Bounce brakeButtonDebounce  = Bounce();
int cycleButtonPrevious   = LOW; // Track state so that a button press can be acted upon once
int launchButtonPrevious  = LOW; // Track state so that a button press can be acted upon once
int brakeButtonPrevious   = LOW; // Track state so that a button press can be acted upon once


/** ___________________________________________________________________________________________________ Sensor Readings */

float batteryVoltageTotal   = 0;
float batteryVoltageLower   = 0;
float throttle        		  = 0;
float current       		    = 0;
float motorRPM        		  = 0;
float wheelRPM 				      = 0;
float wheelSpeed      		  = 0;
float gearRatio 			      = 0;
float tempOne       		    = 0;
float tempTwo       		    = 0;
float tempThree       		  = 0;
int   brake         		    = 0;


/** ___________________________________________________________________________________________________ Smoothing Variables */

/**
   For some signals it is desireable to average them over a longer period than is possible using the hardware
   components on the board. To do this we average the last few readings taken by the Arduino.
   When a new reading is taken it is added to an array. Each new reading takes the place of the oldest reading
   in the array, so the array always contains the last X number of readings, where X is the size of the array.
   In our case, for a reading being updated every 250ms, an array of length 4 would average the readings over
   the last second.
   To implement this in code, two golbal variables are needed per filter:
  	> The Array - to store the last X number of readings
 	> A Count - This dictates which position in the array a new reading goes to and loops between 0 and array length - 1
   To make it simple to alter the length of time to average over it is good practice to define the max count
   value and aray length as a global const too.
*/

//Current Smoothing Variables:

const int currentSmoothingSetting = 4; //current is sampled every 250ms, therefore 4 makes 1s of smoothing
int currentSmoothingArray[currentSmoothingSetting];
int currentSmoothingCount = 0;

//Speed Smoothing Variables:

const int speedSmoothingSetting = 3; //speed is sampled every 1s, therefore 3 makes 3 seconds of smoothing
int speedSmoothingArray[speedSmoothingSetting];
int speedSmoothingCount = 0;





/** ================================== */
/** SETUP                              */
/** ================================== */
void setup()
{

  //Set up pin modes for all inputs and outputs
  pinMode(MOTOR_OUT_PIN,        OUTPUT);
  digitalWrite(MOTOR_OUT_PIN,   LOW);  // Ensure motor is not driven on startup
  pinMode(FAN_OUT_PIN,          OUTPUT);
  digitalWrite(FAN_OUT_PIN,     LOW);  // Ensure fan is not driven on startup
  pinMode(LED_1_OUT_PIN,        OUTPUT);
  pinMode(LED_2_OUT_PIN,        OUTPUT);

  pinMode(VBATT_IN_PIN,     	  INPUT);
  pinMode(VBATT1_IN_PIN,      	INPUT);
  pinMode(THROTTLE_IN_PIN,    	INPUT);
  pinMode(AMPS_IN_PIN,        	INPUT);
  pinMode(TEMP1_IN_PIN,       	INPUT);
  pinMode(TEMP2_IN_PIN,       	INPUT);


  pinMode(LAUNCH_BTN_IN_PIN,    INPUT_PULLUP);
  pinMode(CYCLE_BTN_IN_PIN,     INPUT_PULLUP);
  pinMode(BRAKE_IN_PIN,         INPUT_PULLUP);  //input type will depend on implementation of brake light

  lcd.begin();
  lcd.backlight();

  /**
     Set up Interrupts:
     When the specified digital change is seen on a the interrupt pin it will pause the main loop and
     run the code in the Interrupt Service Routine (ISR) before resuming the main code.
     The interrupt number is not the pin number on the arduino Nano. For explanation see here:
     https://www.arduino.cc/en/Reference/AttachInterrupt
  */

  attachInterrupt(0, motorSpeedISR, RISING);
  attachInterrupt(1, wheelSpeedISR, RISING);
  //FAN_RPM_PIN is connected to a pin that can't be used as an interrupt with arduino libraries
  //so requires some different code.

  //Initialise debounce objects
  cycleButtonDebounce.attach(CYCLE_BTN_IN_PIN);
  cycleButtonDebounce.interval(50);

  launchButtonDebounce.attach(LAUNCH_BTN_IN_PIN);
  launchButtonDebounce.interval(50);

  brakeButtonDebounce.attach(BRAKE_IN_PIN);
  brakeButtonDebounce.interval(50);


  /**
     Zero the current transducer. All hall effect current sensors will need zeroing - To do this we
     take the reading from the sensor during startup where current is virtually zero. (Obviously the
     arduino is drawing current at this point, but compared to the resolution of the curent sensor
     this is negligible).
     The time over which the zero reading is taken is defined in milliseconds by the
     AMPSENSOR_CAL_DELAY constant and the zero value is taken by averaging readings over this time.
     All future current readings take the differential between this zero and the new reading as the
     current value.
  */

  long temp = 0;
  int nReadings = AMPSENSOR_CAL_DELAY / 100; //Ten Readings Per Second
  for (int i = 0; i < nReadings; i++)
  {
    temp += analogRead(AMPS_IN_PIN);
    delay(AMPSENSOR_CAL_DELAY / nReadings);
  }
  currentSensorOffset = (float) temp / nReadings;



  /**
     Initialise Serial Communication
     If communication over bluetooth is not working or the results are garbled it is likely the
     baud rate set here (number in brakcets after Serial.begin) and the baud rate of the bluetooth
     module aren't set the same.

     A good tutorial for altering the HC-05 Bluetooth Module parameters is here:
     http://www.instructables.com/id/Modify-The-HC-05-Bluetooth-Module-Defaults-Using-A/

     The HC-05 modules commonly come preset with baud rates of 9600 or 32000

     Alternatively the following bit of code will attempt to automatically configure a
     HC-05 module if it is plugged in in AT (setup) mode then the arduino is reset. (Power Arduino,
     unplug HC-05 module, press button on HC-05 module, plug back in holding button [light should blink slowly],
     release button, then reset Arduino)
  */

  if (checkBtAtMode()) // checks if the arduino is in AT mode
  {
    configureBluetooth(); // If AT mode is set, configure according to the BT_xxx constants defined above
  }

  Serial.begin(BT_BAUDRATE);    // Bluetooth and USB communications

  lastShortDataSendTime - millis(); //Give the timing a start value.

} //End of Setup

void loop()
{
  // We want to check different variables at different rates. For most variables 0.25 seconds will be good for logging and analysis.
  // Certain variables either can't have or do not need this resolution.
  // Wheel and Motor speed are accumulated over time, so the longer time left between samples, the higher the resolution of the value.
  // As such, these are only updated ever 1 second. Temperature is a reading that will not change fast, and consumes more processing
  // time to calculate than most, so this is also checked every 1s.

  //Asynchronous Operations - those that aren't governed by the 4Hz update/transmit. Primarily buttons.
  buttonChecks(); //Checks buttons each loop, debounces and sends any changes in state



  throttle = readThrottle(); // if this is being used as the input to a motor controller it is recommended to check it at a higher frequency than 4Hz

  if (millis() - lastShortDataSendTime > SHORT_DATA_TRANSMIT_INTERVAL) //i.e. if 250ms have passed since this code last ran
  {
    lastShortDataSendTime = millis(); //this is reset at the start so that the calculation time does not add to the loop time
    loopCounter = loopCounter + 1; // This value will loop 1-4, the 1s update variables will update on certain loops to spread the processing time.

    //It is recommended to leave the ADC a short recovery period between readings (~1ms). To ensure this we can transmit the data between readings
    batteryVoltageTotal = readVoltageTotal();
    sendData(VOLTAGE_ID, batteryVoltageTotal);

    batteryVoltageLower = readVoltageLower();
    sendData(VOLTAGE_LOWER_ID, batteryVoltageLower);

    current = readCurrent();
    sendData(CURRENT_ID, current);


    sendData(THROTTLE_INPUT_ID, throttle);



    if (loopCounter == 1)
    {
      tempOne = readTempOne();
      sendData(TEMP1_ID, tempOne);
      digitalWrite(13, HIGH); //these are just flashing the LEDs as visual confimarion of the loop
      digitalWrite(9, HIGH);
    }

    if (loopCounter == 2)
    {
      tempTwo = readTempTwo();
      sendData(TEMP2_ID, tempTwo);
      digitalWrite(6, HIGH);
    }

    if (loopCounter == 3)
    { //nothing actaully needed to do at .75 seconds
      digitalWrite(13, LOW);
      digitalWrite(9, LOW);
    }

    if (loopCounter == 4)
    {
      loopCounter = 0; //4 * 0.25 makes one second, so counter resets

      wheelSpeed = readWheelSpeed();
      sendData(SPEED_ID, wheelSpeed);

      motorRPM = readMotorRPM();
      sendData(MOTOR_ID, motorRPM);

      gearRatio = calculateGearRatio();
      sendData(GEAR_RATIO_ID, gearRatio);

      digitalWrite(6, LOW);
    }

    if (ENABLE_LCD_DISPLAY) {
      printToLCD();  
    }
  }


}

void buttonChecks()
{
  cycleButtonDebounce.update();
  launchButtonDebounce.update();
  brakeButtonDebounce.update();

  int cycleButtonState = !cycleButtonDebounce.read(); //Buttons are LOW when pressed, ! inverts this, so state is HIGH when pressed
  if (cycleButtonState != cycleButtonPrevious) //Button has changed state - either pressed or depressed
  {
    if (cycleButtonState == HIGH) //Button Pressed
    {
      sendData(CYCLE_VIEW_ID, 1); // Actual packet content is irrelevant - sending a packet with the ID represents a button press
    }

    //Don't care when button is released

    cycleButtonPrevious = cycleButtonState; //Update previous state
  }

  int launchButtonState = !launchButtonDebounce.read(); //Buttons are LOW when pressed, ! inverts this, so state is HIGH when pressed
  if (launchButtonState != launchButtonPrevious) //Button has changed state - either pressed or depressed
  {
    if (launchButtonState == HIGH) //Button Pressed
    {
      sendData(LAUNCH_MODE_ID, 1); // Actual packet content is irrelevant - sending a packet with the ID represents a button press
    }

    //Don't care when button is released

    launchButtonPrevious = launchButtonState; //Update previous state
  }

  int brakeButtonState = !brakeButtonDebounce.read(); //Buttons are LOW when pressed, ! inverts this, so state is HIGH when pressed
  if (brakeButtonState != brakeButtonPrevious) //Button has changed state - either pressed or depressed
  {
    if (brakeButtonState == HIGH) //Button Pressed
    {
      sendData(BRAKE_PRESSED_ID, 100);
    }
    else if (brakeButtonState == LOW) //Button Released
    {
      sendData(BRAKE_PRESSED_ID, 0);
    }

    brakeButtonPrevious = brakeButtonState; //Update previous state
  }

}

void printToLCD() {
  String tempVoltage = String(round(batteryVoltageTotal*10)/10); 
  String tempAmperage = String(round(current*10)/10);
  String tempTemp1 = String(round(tempOne*10)/10);
  String tempTemp2 = String(round(tempTwo*10)/10);
  String tempWheelSpeed = String((round(wheelSpeed*10)/10)*3.6); // *3.6 to kmph
  String tempWheelRPM = String(round(wheelRPM*10)/10);
  
  lcd.print(LCD_FIRST_LINE);
  lcd.setCursor(0,1);
  lcd.print(tempVoltage + "V    " + tempAmperage + "A");
  lcd.setCursor(0,2);
  lcd.print(tempTemp1 + "C    " + tempTemp2 + "C");
  lcd.setCursor(0,3);
  lcd.print(tempWheelSpeed + "KMPH " + tempWheelRPM + "?");
}

/**
   Sensor Reading Functions

   Each function returns the value of the sensor it is reading
*/


float readVoltageTotal()
{
  float tempVoltage = analogRead(VBATT_IN_PIN); //this will give a 10 bit value of the voltage with 1024 representing the ADC reference voltage of 5V

  tempVoltage = (tempVoltage / 1024) * CAL_REFERENCE_VOLTAGE; //This gives the actual voltage seen at the arduino pin, assuming reference voltage of 5v

  tempVoltage = tempVoltage * CAL_BATTERY_TOTAL; //Gives battery voltage where 6 is the division ratio of the potential divider. NEEDS TUNING!!

  return (tempVoltage);
}

float readVoltageLower()
{
  float tempVoltage = analogRead(VBATT1_IN_PIN); //this will give a 10 bit value of the voltage with 1024 representing the ADC reference voltage of 5V

  tempVoltage = (tempVoltage / 1024) * CAL_REFERENCE_VOLTAGE; //This gives the actual voltage seen at the arduino pin, assuming reference voltage of 5v

  tempVoltage = tempVoltage * CAL_BATTERY_LOWER; //Gives battery voltage where 3 is the division ratio of the potential divider. NEEDS TUNING!!

  return (tempVoltage);
}

float readCurrent()
{
  float tempCurrent = analogRead(AMPS_IN_PIN);

  tempCurrent = (tempCurrent / 1024) * CAL_REFERENCE_VOLTAGE; //gives voltage output of current sensor.

  tempCurrent = tempCurrent * CAL_CURRENT; //calibration value for LEM current sensor on eChook board.

  currentSmoothingArray[currentSmoothingCount] = tempCurrent; //updates array with latest value

  // The next 5 lines manage the smoothing count for the averaging:

  currentSmoothingCount ++; //increment smoothing count

  if (currentSmoothingCount >= currentSmoothingSetting)
  {
    currentSmoothingCount = 0; // if current smoothing count is higher than max, reset to 0
  }

  // Now back to the current calculations:

  tempCurrent = 0; //reset temp current to receive sum of array values

  for (int i = 0; i < currentSmoothingSetting; i++)
  {
    tempCurrent += currentSmoothingArray[i]; //sum all values in the current smoothing array
  }

  tempCurrent = tempCurrent / currentSmoothingSetting; //divide summed value by number of samples to get mean

  return (tempCurrent); //return the final smoothed value
}

//If a push button throttle is used, comment out the upper readThrottle function, and unComment the lower one.

int readThrottle() //This function is for a variable Throttle input
{
  float tempThrottle = analogRead(THROTTLE_IN_PIN);



  tempThrottle = (tempThrottle / 1023) * CAL_REFERENCE_VOLTAGE; // Gives the actual voltage seen on the arduino Pin, assuming reference voltage of 5V

  if (tempThrottle < 1) //less than 1V
  {
    tempThrottle = 0;
  }
  else if (tempThrottle > 4)  //greater than 4 V
  {
    tempThrottle = CAL_REFERENCE_VOLTAGE;
  }

  tempThrottle = (tempThrottle / CAL_REFERENCE_VOLTAGE) * 100; // Gives throttle as a percentage


  analogWrite(MOTOR_OUT_PIN, map((int)tempThrottle, 0, 100, 0, 255)); //This drives the motor output. Unless you are using the board to drive your motor, comment it out.

  return ((int)tempThrottle); // the (int) converts the value into an integer value before the return function uses it
}

/*
  int readThrottle() //This function is for a push button, on/off Throttle input
  {
  float tempThrottle = analogRead(THROTTLE_IN_PIN);

  if(tempThrottle > 200)
  {
  	tempThrottle = 100; //full throttle
  }else
  {
  	tempThrottle = 0; //No throttle
  }

  return ((int)tempThrottle); // the (int) converts the value into an integer value before the return function uses it
  }
*/




/**

   If an active sensor such as a TMP37 is used this has a calibrated voltage output, linear to the temperature change.
   A cheaper option is to use Thermistors. The resistance across a thrmistor changes with temperature, but the change is not linear
   so some maths is required to translate the voltage reading into a temperature value.
   For more information see here: http://playground.arduino.cc/ComponentLib/Thermistor2
   This method uses the Steinhart-Hart equation to calculate the actual temperature, however this requires three coefficients,
   A, B and C, that are specific to a thermistor. The ones below are for the thermistors provided with the board, however if you
   use a different thermistor the coefficients should be given in the datasheet, and if not, can be calculated using this calculator:
   http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm


 **/


float readTempOne()
{
  float temp = thermistorADCToCelcius(analogRead(TEMP1_IN_PIN)); //use the thermistor function to turn the ADC reading into a temperature

  return (temp); //return Temperature.
}

float readTempTwo()
{
  float temp = thermistorADCToCelcius(analogRead(TEMP2_IN_PIN));

  return (temp);
}


float readWheelSpeed() //wheelRPM is updated whenever this is called
{
  // First action is to take readings and reset the wheel count so that there are no change to the variables during the calculations or between reading and resetting:

  int tempWheelPoll = wheelPoll;
  wheelPoll = 0;


  unsigned long tempLastWheelPollTime = lastWheelSpeedPollTime;
  unsigned long tempWheelPollTime = millis();
  lastWheelSpeedPollTime = tempWheelPollTime;

  //Wheel RMP has been tacked into this function at a later date, so could do with re-writing really...

  //Wheel RMP Calculations:
  wheelRPM = (float) tempWheelPoll / (float) CAL_WHEEL_MAGNETS; //gives number of rotations

  wheelRPM = wheelRPM / ((float)(tempWheelPollTime - tempLastWheelPollTime) / (float) 60000.0); // /60,000 converts millis to minutes

  // All integers in the folowing equation are cast to float so that the value is not converted to an integer at any point reducing accuracy through rounding.

  // Next task is to calculate the distance travelled. This is dome by takin the wheel poll, which is the number of magnets that have passed the sensor since the last check
  // and dividing it by the number of wheel magnets to give the number of revolutions, then multiply by the circumference to give distance travelled in meters:
  float wheelDistanceTravelled =  (float) tempWheelPoll / (float) CAL_WHEEL_MAGNETS * (float) CAL_WHEEL_CIRCUMFERENCE;

  // Now determine how much time in seconds it took to travel this distance, and divide the distanve by time to get speed in meters per second.

  float wheelSpeedMetersPerSecond = wheelDistanceTravelled / ((float)(tempWheelPollTime - tempLastWheelPollTime) / (float)1000.0); // the /1000 converts the milliseconds to seconds

  //Next section of code handles the smooting:

  speedSmoothingArray[speedSmoothingCount] = wheelSpeedMetersPerSecond; //adds current speed into oldest position in array

  speedSmoothingCount ++ ; //incrememnts array position for next reading

  if (speedSmoothingCount >= speedSmoothingSetting)
  {
    speedSmoothingCount = 0; //reset if count exceeds array length
  }

  wheelSpeedMetersPerSecond = 0; //reset variable ready to sum array

  for (int i = 0; i < speedSmoothingSetting; i++)
  {
    wheelSpeedMetersPerSecond = speedSmoothingArray[i];
  }

  wheelSpeedMetersPerSecond = wheelSpeedMetersPerSecond / speedSmoothingSetting; //divide summed figure by array count to get mean value

  return (wheelSpeedMetersPerSecond); //return smoothed value
}

float readMotorRPM()
{
  // First action is to take copies of the motor poll count and time so that the variables don't change during the calculations.

  int tempMotorPoll = motorPoll;
  motorPoll = 0;


  unsigned long tempLastMotorPollTime = lastMotorSpeedPollTime;
  unsigned long tempMotorPollTime = millis();
  lastMotorSpeedPollTime = tempMotorPollTime;

  // Now calculate the number of revolutions of the motor shaft

  float motorRevolutions = tempMotorPoll / CAL_MOTOR_MAGNETS;

  float motorRevolutionsPerMin = motorRevolutions * 60.0;

  float timeDiffms = tempMotorPollTime - tempLastMotorPollTime;
  float timeDiffs = timeDiffms / 1000.0;
  // Now use the time time passed to convert this to revolutions per minute
  // RMP = (revolutions / latestPollTIme - lastPollTime) / 1000 to convert to Seconds) * 60 to convert to minutes

  float motorShaftRPM = motorRevolutionsPerMin / timeDiffs;

  return (motorShaftRPM);
}

float calculateGearRatio()
{
  float tempGearRatio = motorRPM / wheelRPM;

  return (tempGearRatio);
}



/**
   Calculation Functions

   Where calculations are needed multiple times, they are broken out into their own functions here
*/

/**
   Temperature Calculations:

   If an active sensor such as a TMP37 is used this has a calibrated voltage output, linear to the temperature change.
   A cheaper option is to use Thermistors. The resistance across a thrmistor changes with temperature, but the change is not linear
   so some maths is required to translate the voltage reading into a temperature value.
   For more information see here: http://playground.arduino.cc/ComponentLib/Thermistor2
   This method uses the Steinhart-Hart equation to calculate the actual temperature, however this requires three coefficients,
   A, B and C, that are specific to a thermistor. The ones below are for the thermistors provided with the board, however if you
   use a different thermistor the coefficients should be given in the datasheet, and if not, can be calculated using this calculator:
   http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
 **/

float thermistorADCToCelcius(int rawADC)
{
  // Constants for this calculation:

  // Steinhart-Hart Coefficients, see comment above
  // These coefficients are for the MF52AT NTC 10k thermistor, however due to thermistor tolerances each thermistor should be calibrated individually.
  const float A = CAL_THERM_A;
  const float B = CAL_THERM_B;
  const float C = CAL_THERM_C;

  // Value of resistor forming potential divider with Thermistor in ohms.
  const int FIXED_RESISTOR_VALUE = 10000; //10k

  // Calculations:
  // The formula is: Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3} where A, B and C are the coefficients above and R is the resistance across the thermistor.

  // First step is to calculate the resistance of the thermistor using the potential divider equation V_out = (R1 + R2)/(R1 * R2)*V_in
  // As R2 is the only unknown variable we can re-write this as: R2 = R1((V_in/V_out)-1).  R2 = (R1 V2)/(V1-V2)
  // As the ADC values are our readings of the voltage, we can substitute V_in with 1024 and V_out with the reading taken from the ADC, which is passed into this function as rawADC
  // This makes the calculation:

  float thermistorResistance = ((float)FIXED_RESISTOR_VALUE * (float)rawADC) / (float)((float)1023 - (float)rawADC);

  // Next, you'll notice that the log natural (ln) of this resistance needs to be calculated 4 times in the Steinhart-Hart equation. This is a complex and long calculation for the arduino.
  // As such it is efficient to do it once and save the result for use later:

  double lnResistance = log(thermistorResistance);

  // Now plug it all into the equation:

  double temperature = 1 / (A + (B * lnResistance) + (C * lnResistance * lnResistance * lnResistance));



  // We now have the temperature in Kelvin. To convert it into Celcius we need to subtract 273.15

  temperature = temperature - 273.15;

  if (DEBUG_MODE)
  {
    Serial.print("\n\rThemistore Resistance = ");
    Serial.println(thermistorResistance);
    Serial.print("Temperature = ");
    Serial.println(temperature);
  }



  // Now return the Celcius Value:

  return (temperature);

}

/** ================================== */
/** BLUETOOTH DATA PACKETING FUNCTIONS */
/** ================================== */
/** The two functions in this section handle packeting the data and sending it over USART to the bluetooth module. The two functions are
    identically named so are called the in the same way, however the first is run if the value passed to it is a float and the second is
    run if the value passed into it is an integer (an override function). For all intents and purposes you can ignore this and simply call
    'sendData(identifier, value);' using one of the defined identifiers and either a float or integer value to send informaion over BT.

   identifier:  see definitions at start of code
   value:       the value to send (typically some caluclated value from a sensor)
*/

void sendData(char identifier, float value)
{
  if (!DEBUG_MODE) // Only runs if debug mode is LOW (0)
  {
    byte dataByte1;
    byte dataByte2;

    if (value == 0)
    {
      // It is impossible to send null bytes over Serial connection
      // so instead we define zero as 0xFF or 11111111 i.e. 255
      dataByte1 = 0xFF;
      dataByte2 = 0xFF;

    }
    else if (value <= 127)
    {
      // Values under 128 are sent as a float
      // i.e. value = dataByte1 + dataByte2 / 100
      int integer;
      int decimal;
      float tempDecimal;

      integer = (int) value;
      tempDecimal = (value - (float) integer) * 100;
      decimal = (int) tempDecimal;

      dataByte1 = (byte) integer;
      dataByte2 = (byte) decimal;

      if (decimal == 0)
      {
        dataByte2 = 0xFF;
      }

      if (integer == 0)
      {
        dataByte1 = 0xFF;
      }

    }
    else
    {
      // Values above 127 are sent as integer
      // i.e. value = dataByte1 * 100 + dataByte2
      int tens;
      int hundreds;

      hundreds = (int)(value / 100);
      tens = value - hundreds * 100;

      dataByte1 = (byte)hundreds;
      //dataByte1 = dataByte1 || 0x10000000; //flag for integer send value
      dataByte1 += 128;
      dataByte2 = (byte) tens;

      if (tens == 0)
      {
        dataByte2 = 0xFF;
      }

      if (hundreds == 0)
      {
        dataByte1 = 0xFF;
      }
    }

    // Send the data in the format { [id] [1] [2] }
    Serial.write(123);
    Serial.write(identifier);
    Serial.write(dataByte1);
    Serial.write(dataByte2);
    Serial.write(125);

  }

}

/** override for integer values*/
void sendData(char identifier, int value)
{
  if (!DEBUG_MODE)
  {
    byte dataByte1;
    byte dataByte2;

    if (value == 0)
    {
      dataByte1 = 0xFF;
      dataByte2 = 0xFF;

    }
    else if (value <= 127)
    {

      dataByte1 = (byte)value;
      dataByte2 = 0xFF; //we know there's no decimal component as an int was passed in
    }
    else
    {
      int tens;
      int hundreds;

      hundreds = (int)(value / 100);
      tens = value - hundreds * 100;

      dataByte1 = (byte)hundreds;
      dataByte1 += 128;   //sets MSB High to indicate Integer value
      dataByte2 = (byte) tens;

      if (tens == 0)
      {
        dataByte2 = 0xFF;
      }

      if (hundreds == 0)
      {
        dataByte1 = 0xFF;
      }
    }

    Serial.write(123);
    Serial.write(identifier);
    Serial.write(dataByte1);
    Serial.write(dataByte2);
    Serial.write(125);
  }

}


/** ================================== */
/** HC-05 CONFIGURATION FUNCTIONS      */
/** ================================== */


int checkBtAtMode() //Checks if the HC-05 Bluetooth module is in AT mode. Returns 1 if it is, 0 otherwise
{

  int atMode = 0;

  Serial.begin(38400); //AT mode baud rate

  while (!Serial) {} //Wait for serial to initialise

  delay(200);

  Serial.print("AT\r\n"); // for some reason when AT is sent the first time, an error is always returned.

  delay(200); //wait for response

  //    Serial.flush(); // Flush the response to the first AT query
  flushSerial();

  while (Serial.available()) // to check if the flush worked
  {
    Serial.println((char)Serial.read());
  }

  //Serial.println("End of flush check");
  Serial.println("AT");

  delay(200);

  char tempOne = (char)Serial.read();
  delay(20);
  char tempTwo = (char)Serial.read();

  if (tempOne == 'O' && tempTwo == 'K') //Was the response "OK"?
  {
    //    Serial.println("AT Mode Confirmed");
    digitalWrite(13, HIGH);
    atMode = 1;
  }
  else
  {
    //    Serial.println("AT Mode NOT Confirmed");
    //    Serial.print("Char 1 = ");
    //    Serial.print(tempOne);
    //    Serial.print("Char 2 = ");
    //    Serial.println(tempTwo);

  }

  Serial.begin(BT_BAUDRATE); //reset baud rate
  while (!Serial) {} //wait while serial is inialising

  return (atMode);
}


void configureBluetooth()
{
  //Assumes AT mode has been confirmed.

  Serial.begin(38400);

  uint8_t btNameSet = 0; //These will be set to 1 when each is successfully updated
  uint8_t btBaudSet = 0;
  uint8_t btPasswordSet = 0;

  //Set Bluetooth Name

  flushSerial();// Flush the buffer. Not entirely sure what is in there to flush at this point, but it is needed!
  Serial.print("AT+NAME=");
  Serial.print(BT_NAME);
  Serial.print("\r\n");

  //Now Check Response

  waitForSerial(500);

  char tempOne = (char)Serial.read();

  waitForSerial(500);

  char tempTwo = (char)Serial.read();


  if (tempOne == 'O' && tempTwo == 'K') //Was the response "OK"?
  {
    Serial.println("Name Set");
    btNameSet = 1;
  }
  else
  {
    Serial.println("Name Not Set");
    Serial.print("Char 1 = ");
    Serial.println(tempOne);
    Serial.print("Char 2 = ");
    Serial.print(tempTwo);
    Serial.print("\r\n");
  }

  //Set Baud Rate_____________________________________________
  delay(100);
  //Serial.flush();
  flushSerial();
  Serial.print("AT+UART="); //command to change BAUD rate
  Serial.print(BT_BAUDRATE); // overflowed when baud rate was an int
  //Serial.print(115200); //Convert the integer constant baud rate into a string BT_BAUDRATE
  Serial.println(",0,0"); //Parity and Stop bits

  //Now Check Response.

  waitForSerial(500);

  tempOne = (char)Serial.read();

  waitForSerial(500);

  tempTwo = (char)Serial.read();


  if (tempOne == 'O' && tempTwo == 'K') //Was the response "OK"?
  {
    Serial.println("Baud Rate Set");
    btBaudSet = 1;
  }
  else
  {
    Serial.println("Baud Rate Not Set");
    Serial.print("Char 1 = ");
    Serial.println(tempOne);
    Serial.print("Char 2 = ");
    Serial.println(tempTwo);
  }


  //Set Password_____________________________________________ Not working - leaving for now

  //    flushSerial();
  //    Serial.print("AT+PSWD="); //command to change Password
  //    //  Serial.println(BT_PASSWORD);
  //    Serial.print("1234");
  //    Serial.print("\r\n");
  //
  //    waitForSerial(1000);
  //
  //    tempOne = (char)Serial.read();
  //
  //    waitForSerial(500);
  //
  //    tempTwo = (char)Serial.read();
  //
  //
  //    if (tempOne == 'O' && tempTwo == 'K') //Was the response "OK"?
  //    {
  //      Serial.println("Password Set");
  //      btPasswordSet = 1;
  //    }
  //    else
  //    {
  //      Serial.println("Password Not Set");
  //    }


  // Check all operations completed successfully
  if (btBaudSet && btNameSet)// && btPasswordSet)
  {
    flushSerial();
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
    Serial.println("AT+RESET\r\n"); // has to be in the middle to provide a suitable delay before and after
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);

  }
  else
  {
    while (1) //endless loop - config failed so shouldn't continue!
    {
      digitalWrite(13, HIGH);
      delay(50);
      digitalWrite(13, LOW);
      delay(50);
    }
  }

  Serial.begin(BT_BAUDRATE);
}


void flushSerial()
{
  while (Serial.available())
  {
    char temp = Serial.read();
  }
}

void waitForSerial(int timeOut)
{
  long tempTime = millis() + timeOut;

  while (!Serial.available() && millis() < tempTime)
  {} //Do nothing - i.e. wait until serial becomes availble or the timeout is reached.
}

/** ================================== */
/** INTERRUPT SERVICE ROUTINES         */
/** ================================== */

/** This function is triggered every time the magnet on the motor shaft passes the Hall effect
    sensor. Care must be taken to ensure the sensor is position the correct way and so is the
    magnet as it will only respond in a specific orientation and magnet pole.

    It is best practice to keep ISRs as small as possible.
*/
void motorSpeedISR()
{
  motorPoll++;
}

void wheelSpeedISR()
{
  wheelPoll++;
}

void fanSpeedISR()
{
  fanPoll++;
}




