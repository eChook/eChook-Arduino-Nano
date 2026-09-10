/**
 * @file eChook_Functions.ino
 * @brief Core measurement and logic functions for the eChook board.
 *
 * This file contains functions for reading sensors, calculating derived values,
 * and managing data transmission logic.
 */

/**
 * @brief Performs all setup routines for the eChook board.
 *
 * Initializes pins, debounces, serial communication, Bluetooth, EEPROM, and
 * reference voltage.
 */
void eChookSetup() {
  pinSetup(); // Sets Input/Output for all pins. Function found in
              // eChook_Functions.ino.

  // Initialise debounce objects for the three buttons
  cycleButtonDebounce.attach(CYCLE_BTN_IN_PIN);
  cycleButtonDebounce.interval(50); // 50ms

  launchButtonDebounce.attach(LAUNCH_BTN_IN_PIN);
  launchButtonDebounce.interval(50);

  brakeButtonDebounce.attach(BRAKE_IN_PIN);
  brakeButtonDebounce.interval(50);

  /**
   * Initialise Serial Communication
   * If communication over bluetooth is not working or the results are garbled
   * it is likely the baud rate set here (number in brackets after
   * SerialA.begin) and the baud rate of the bluetooth module aren't set the
   * same.
   *
   * A good tutorial for altering the HC-05 Bluetooth Module parameters is here:
   * http://www.instructables.com/id/Modify-The-HC-05-Bluetooth-Module-Defaults-Using-A/
   *
   * The HC-05 modules commonly come preset with baud rates of 9600 or 32000
   *
   * Alternatively configureBluetooth function will attempt to automatically
   * configure a HC-05 module if it is plugged in. A PCB V2 Board will do this
   * fully automatically, a V1.x board requires manually setting AT mode - Power
   * Arduino, unplug HC-05 module, press button on HC-05 module, plug back in
   * holding button [light should blink slowly], release button, then reset
   * Arduino)
   */

  hardwareSerialSetup();

  configureBluetooth(); // Checks if If AT mode is set and configures HC-05
                        // according to the BT_xxx constants defined above

  SerialA.begin(CAL_BT_BAUDRATE); // Nano Clone - Bluetooth and USB
                                  // communications, Nano Every - BT Only.

  // Read in calibration from EEPROM memory if required
  EEPROMSetup();

  referenceVoltage = updateReferenceVoltage();

  hardwarePrintSetupComplete();
}

/**
 * @brief Configures pin modes and sets up interrupts for the board.
 */
void pinSetup() {

#ifdef JUMPER_I2C
  TEMP1_IN_PIN = A1;
  TEMP2_IN_PIN = A6;
#endif

  // Set up pin modes for all inputs and outputs
  pinMode(MOTOR_OUT_PIN, OUTPUT);
  digitalWrite(MOTOR_OUT_PIN, LOW); // Ensure motor is not driven on startup

  pinMode(VBATT_IN_PIN, INPUT);
  pinMode(VBATT1_IN_PIN, INPUT);
  pinMode(THROTTLE_IN_PIN, INPUT);
  pinMode(AMPS_IN_PIN, INPUT);
  pinMode(TEMP1_IN_PIN, INPUT);
  pinMode(TEMP2_IN_PIN, INPUT);

  pinMode(LAUNCH_BTN_IN_PIN, INPUT_PULLUP);
  pinMode(CYCLE_BTN_IN_PIN, INPUT_PULLUP);
  pinMode(
      BRAKE_IN_PIN,
      INPUT_PULLUP); // input type will depend on implementation of brake light

#ifdef JUMPER_BT_EN
  pinMode(BT_EN_PIN, OUTPUT);
  digitalWrite(BT_EN_PIN, HIGH);
#endif

  /**
   * @brief Set up Interrupts.
   *
   * When the specified digital change is seen on the interrupt pin, it will
   * pause the main loop and run the Interrupt Service Routine (ISR).
   *
   * @see motorSpeedISR
   * @see wheelSpeedISR
   * @see https://www.arduino.cc/en/Reference/AttachInterrupt
   */

  hardwareAttachInterrupts();
}

/**
 * @brief Updates sensor readings and sends data at defined intervals.
 * Handles periodic updates for throttle, voltage, current, temperature, speed,
 * and gear ratio.
 */
void eChookRoutinesUpdate() {

  // We want to check different variables at different rates. For most variables
  // 0.25 seconds will be good for logging and analysis. Certain variables
  // either can't have or do not need this resolution. Wheel and Motor speed are
  // accumulated over time, so the longer time left between samples, the higher
  // the resolution of the value. As such, these are only updated ever 1 second.
  // Temperature is a reading that will not change fast, and consumes more
  // processing time to calculate than most, so this is also checked every 1s.

  SerialCheck();

  static unsigned long nextThrottleReadMs = millis();
  if (millis() > nextThrottleReadMs) { // millis() gives milliseconds since power on. If this is greater than the nextThrottleReadMs we've calculated it will run.
    nextThrottleReadMs += 100;         // 100 ms, 10hz
    throttleOutput = readThrottle();   // if this is being used as the input to a motor controller it is recommended to check it at a higher frequency than 4Hz
  }

  static unsigned long lastShortDataSendTime = millis();             // this is reset at the start so that the calculation time does not add to the loop time
  if (millis() - lastShortDataSendTime > CAL_DATA_TRANSMIT_INTERVAL) // i.e. if 250ms have passed since this code last ran
  {
    lastShortDataSendTime = millis();
    static unsigned int loopCounter = 0;
    loopCounter = loopCounter + 1; // This value will loop 1-4, the 1s update variables will update on certain loops to spread the processing time.
    // It is recommended to leave the ADC a short recovery period between
    // readings (~1ms). To achieve this we can transmit the data between
    // readings
    // The reference voltage has to be measured before anything is scaled by
    // it, otherwise every reading below is corrected using a rail measurement
    // a full cycle out of date. Sending it straight afterwards doubles as the
    // ADC recovery delay described above.
    referenceVoltage = updateReferenceVoltage();
    sendData(REF_VOLTAGE_ID, referenceVoltage);

    batteryVoltageTotal = readVoltageTotal();
    sendData(VOLTAGE_ID, batteryVoltageTotal);

    batteryVoltageLower = readVoltageLower();
    sendData(VOLTAGE_LOWER_ID, batteryVoltageLower);

    current = readCurrent();
    sendData(CURRENT_ID, current);

    sendData(THROTTLE_VOLTAGE_ID, throttleV);
    sendData(THROTTLE_INPUT_ID, throttleIn);
    sendData(THROTTLE_OUTPUT_ID, throttleOutput);

    motorRPM = readMotorRPM();
    sendData(MOTOR_ID, motorRPM);

    wheelSpeed = readWheelSpeed();
    sendData(SPEED_ID, wheelSpeed);

    if (loopCounter == 1) { // Functions to run every 1st loop
      tempOne = readTempOne();
      sendData(TEMP1_ID, tempOne);
      digitalWrite(13, HIGH); // these are just flashing the LEDs as visual
                              // confimarion of the loop
    }

    if (loopCounter == 2) { // Functions to run every 2nd loop
      tempTwo = readTempTwo();
      sendData(TEMP2_ID, tempTwo);
    }

    if (loopCounter == 3) { // Functions to run every 3rd loop
      tempThree = readTempInternal();
      sendData(TEMP3_ID, tempThree);
      digitalWrite(13, LOW);
    }

    if (loopCounter == 4) { // Functions to run every 4th loop
      loopCounter = 0;      // 4 * 0.25 makes one second, so counter resets
      gearRatio = calculateGearRatio();
      sendData(GEAR_RATIO_ID, gearRatio);
    }
  }
}

/**
 * @brief Checks the state of each button and sends updates over Bluetooth if pressed.
 */
void buttonChecks() {
  cycleButtonDebounce.update();
  launchButtonDebounce.update();
  brakeButtonDebounce.update();
  static unsigned int cycleButtonPrevious =
      LOW;                                                     // Track state so that a button press can be detected
  unsigned int cycleButtonState = !cycleButtonDebounce.read(); // Buttons are LOW when pressed, ! inverts this, so state is HIGH when pressed
  if (cycleButtonState != cycleButtonPrevious)                 // Button has changed state - either pressed or depressed
  {
    if (cycleButtonState == HIGH) // Button Pressed
    {
      sendData(CYCLE_VIEW_ID, 1);
    } else {
      sendData(CYCLE_VIEW_ID, 0);
    }
    cycleButtonPrevious = cycleButtonState; // Update previous state
  }

  static unsigned int launchButtonPrevious =
      LOW;                                                       // Track state so that a button press can be detected
  unsigned int launchButtonState = !launchButtonDebounce.read(); // Buttons are LOW when pressed, ! inverts this, so state is HIGH when pressed
  if (launchButtonState != launchButtonPrevious)                 // Button has changed state -
                                                                 // either pressed or depressed
  {
    if (launchButtonState == HIGH) // Button Pressed
    {
      sendData(LAUNCH_MODE_ID, 1);
    } else {
      sendData(LAUNCH_MODE_ID, 0);
    }
    launchButtonPrevious = launchButtonState; // Update previous state
  }

  static unsigned int brakeButtonPrevious =
      LOW; // Track state so that a button press can be detected
  unsigned int brakeButtonState =
      !brakeButtonDebounce.read();             // Buttons are LOW when pressed, ! inverts
                                               // this, so state is HIGH when pressed
  if (brakeButtonState != brakeButtonPrevious) // Button has changed state -
                                               // either pressed or depressed
  {
    if (brakeButtonState == HIGH) // Button Pressed
    {
      sendData(BRAKE_PRESSED_ID, 100);
    } else if (brakeButtonState == LOW) // Button Released
    {
      sendData(BRAKE_PRESSED_ID, 0);
    }
    brakeButtonPrevious = brakeButtonState; // Update previous state
  }
}

/**
 * @brief Updates and returns the reference voltage for ADC calculations.
 * @return The calculated reference voltage.
 */
float updateReferenceVoltage() {
  // Handled by hardware abstraction layer
  return hardwareUpdateReferenceVoltage();
}

/**
 * @brief Converts a raw ADC reading into the voltage seen at the Arduino pin.
 *
 * An ADC result of N means the input sat somewhere in a band one LSB wide
 * starting at N, so the best estimate of the input is half an LSB above N.
 * Dividing by 1024 rather than 1023 matches the convention used to derive the
 * rail voltage in hardwareUpdateReferenceVoltage(). That consistency matters:
 * the rail voltage is itself an ADC ratio, so using the same convention on
 * both sides lets ADC gain error cancel rather than compound.
 *
 * @param rawADC The raw reading, on the 0-1023 scale.
 * @return The voltage at the Arduino pin.
 */
float adcToPinVoltage(float rawADC) {
  return ((rawADC + 0.5) / 1024.0) * referenceVoltage;
}

/**
 * @brief Reads and calculates the total battery voltage.
 * @return The total battery voltage in volts.
 */
float readVoltageTotal() {
  // Oversampled reading of the potential divider, as the voltage seen at the pin
  float pinVoltage = adcToPinVoltage(hardwareAnalogReadOversampled(VBATT_IN_PIN));
  // Scale by the division ratio of the potential divider. NEEDS TUNING!!
  return (pinVoltage * CAL_BATTERY_TOTAL);
}

/**
 * @brief Reads and calculates the lower battery voltage.
 * @return The lower battery voltage in volts.
 */
float readVoltageLower() {
  // Oversampled reading of the potential divider, as the voltage seen at the pin
  float pinVoltage = adcToPinVoltage(hardwareAnalogReadOversampled(VBATT1_IN_PIN));
  // Scale by the division ratio of the potential divider. NEEDS TUNING!!
  return (pinVoltage * CAL_BATTERY_LOWER);
}

/**
 * @brief Reads, smooths, and calculates the Current value.
 * @return The Current in amps.
 */
float readCurrent() {
  // Oversampled reading, converted to the output voltage of the current sensor
  float tempCurrent = adcToPinVoltage(hardwareAnalogReadOversampled(AMPS_IN_PIN));
  tempCurrent = tempCurrent * CAL_CURRENT; // calibration value for LEM current sensor on eChook board.

  currentSmoothingArray[currentSmoothingCount] = tempCurrent; // updates array with latest value
  // The next 5 lines manage the smoothing count for the averaging:
  currentSmoothingCount++; // increment smoothing count
  if (currentSmoothingCount >= currentSmoothingSetting) {
    currentSmoothingCount = 0; // if current smoothing count is higher than max, reset to 0
  }
  // Now back to the current calculations:
  tempCurrent = 0; // reset temp current to receive sum of array values
  for (int i = 0; i < currentSmoothingSetting; i++) {
    tempCurrent += currentSmoothingArray[i]; // sum all values in the current smoothing array
  }
  tempCurrent = tempCurrent / currentSmoothingSetting; // divide summed value by number of samples to get mean
  return (tempCurrent);                                // return the final smoothed value
}

/**
 * @brief Reads and processes the throttle input.
 * @return The throttle output as a percentage.
 */
float readThrottle() {
  static int currThrtlOut = 0;
  float rawThrottle = hardwareAnalogReadOversampled(THROTTLE_IN_PIN);
  float tempThrottle = 0;

  if (CAL_THROTTLE_VARIABLE) // Analogue throttleOutput, not push button
  {
    tempThrottle = adcToPinVoltage(rawThrottle); // Gives the actual voltage seen on the arduino Pin
    throttleV = tempThrottle;                    // Update Global variable for throttleOutput in voltage

    // The following code adds dead bands to the start and end of the
    // throttleOutput travel
    if (tempThrottle < CAL_THROTTLE_LOW) // less than 1V
    {
      tempThrottle = CAL_THROTTLE_LOW;
    } else if (tempThrottle > CAL_THROTTLE_HIGH) // greater than 4 V
    {
      tempThrottle = CAL_THROTTLE_HIGH;
    }

    tempThrottle = ((tempThrottle - CAL_THROTTLE_LOW) / (float)(CAL_THROTTLE_HIGH - CAL_THROTTLE_LOW)) * (255);
  } else {
    throttleV = adcToPinVoltage(rawThrottle); // Update Global variable for throttleOutput in voltage
    if (rawThrottle > 200)                    // Approx 1v
    {
      tempThrottle = 255; // full throttleOutput
    } else {
      tempThrottle = 0; // No throttleOutput
    }
  }

  throttleIn = (float)tempThrottle / 2.55; // Convert to a float percentage for the output

  if (CAL_THROTTLE_RAMP) {
    // This code generates a simple ramp up in throttleOutput. The >100 is there
    // as it will likely take about 40% throttleOutput to get the car moving, so
    // this will give a quicker start.
    if (tempThrottle >= currThrtlOut && tempThrottle > 100) { // This could be if(thrtlIn > thrtlOut && speed < threshold) to make it low speed only. Speed and threshold are undefined in this example!
      if (currThrtlOut < 100) {
        currThrtlOut = 101;
      }
      currThrtlOut = currThrtlOut + 4;   // Value dictates ramp speed. Calculated by
                                         // (155/x)/10. 4 gives (155/4)/10=3.875 seconds, 2
                                         // gives 7.75 seconds, 1 gives 15.5 seconds
      if (currThrtlOut > tempThrottle) { // Fixes the throttleOutput jitter if the increment
                                         // puts output over request.
        currThrtlOut = tempThrottle;
      }
    } else {
      currThrtlOut = tempThrottle;
    }

  } else {
    currThrtlOut = tempThrottle;
  }

  if (CAL_THROTTLE_OUTPUT_EN) {
    analogWrite(MOTOR_OUT_PIN, currThrtlOut); // This drives the motor output. Unless you are using the board to drive your motor you can comment it out.
  } else {
    analogWrite(MOTOR_OUT_PIN, 0);
  }

  return (float)currThrtlOut / 2.55; // Convert to a float percentage for the output
}

/**
 * @brief Reads and calculates the temperature from thermistor 1.
 * @return The temperature in Celsius.
 */
float readTempOne() {
  float temp = thermistorADCToCelcius(hardwareAnalogReadOversampled(TEMP1_IN_PIN), 1); // use the thermistor function to turn the ADC reading into a temperature
  return (temp);                                                    // return Temperature.
}

/**
 * @brief Reads and calculates the temperature from thermistor 2.
 * @return The temperature in Celsius.
 */
float readTempTwo() {
  float temp = thermistorADCToCelcius(hardwareAnalogReadOversampled(TEMP2_IN_PIN), 2);
  return (temp);
}

// Reading the interanl arduino tempreature - notes
// on the accuracy and calibration here:
// https://playground.arduino.cc/Main/InternalTemperatureSensor/
/**
 * @brief Reads and calculates the internal temperature of the Arduino.
 * @return The internal temperature in Celsius.
 */
float readTempInternal(void) {
  // Handled by hardware abstraction layer
  return hardwareReadTempInternal();
}

/**
 * @brief Calculates and returns the wheel speed.
 * @return The wheel speed in meters per second.
 */
float readWheelSpeed() {
  if (CAL_WHEEL_MAGNETS == 0)
    return 0;

  // Timeout logic: if no pulse for 3 seconds, speed is 0
  unsigned long timeSinceLast;
  noInterrupts();
  timeSinceLast = micros() - lastWheelPollTime; // Handle wrap-around automatically
  interrupts();

  if (timeSinceLast > 3000000) {
    wheelSpeed = 0;
    // Reset smoothing buffer
    for (int i = 0; i < smoothingSize; i++)
      wheelSpeedSmoothing[i] = 0;
    return 0;
  }

  noInterrupts();
  bool speedSignal = newSpeedSignal;
  unsigned long interval = lastWheelInterval;
  newSpeedSignal = false;
  interrupts();

  if (speedSignal) {
    if (interval > 0) {
      // Calculate instantaneous Speed in m/s
      // Interval is in micros.
      // RPS = 1,000,000 / (interval * magnets)
      float wheelRPS = 1000000.0 / ((float)(interval * CAL_WHEEL_MAGNETS));
      float instantaneousSpeed = wheelRPS * CAL_WHEEL_CIRCUMFERENCE;

      // Add to smoothing buffer
      wheelSpeedSmoothing[wheelSmoothingIndex] = instantaneousSpeed;
      wheelSmoothingIndex = (wheelSmoothingIndex + 1) % smoothingSize;

      // Calculate average
      float sum = 0;
      for (int i = 0; i < smoothingSize; i++)
        sum += wheelSpeedSmoothing[i];
      wheelSpeed = sum / smoothingSize;
    }
  }

  return wheelSpeed;
}

/**
 * @brief Calculates and returns the motor RPM.
 * @return The motor RPM.
 */
float readMotorRPM() {
  if (CAL_MOTOR_MAGNETS == 0)
    return 0;

  // Timeout logic: if no pulse for 1 second, RPM is 0
  unsigned long timeSinceLast;
  noInterrupts();
  timeSinceLast = micros() - lastMotorPollTime;
  interrupts();

  if (timeSinceLast > 1000000) {
    motorRPM = 0;
    // Reset smoothing buffer
    for (int i = 0; i < smoothingSize; i++)
      motorRPMSmoothing[i] = 0;
    return 0;
  }

  noInterrupts();
  bool motorSignal = newMotorSignal;
  unsigned long interval = lastMotorInterval;
  newMotorSignal = false;
  interrupts();

  if (motorSignal) {
    if (interval > 0) {
      // Calculate instantaneous RPM
      // RPM = (1,000,000 / (interval * magnets)) * 60
      float instantaneousRPM = (60000000.0 / ((float)(interval * CAL_MOTOR_MAGNETS)));

      // Add to smoothing buffer
      motorRPMSmoothing[motorSmoothingIndex] = instantaneousRPM;
      motorSmoothingIndex = (motorSmoothingIndex + 1) % smoothingSize;

      // Calculate average
      float sum = 0;
      for (int i = 0; i < smoothingSize; i++)
        sum += motorRPMSmoothing[i];
      motorRPM = sum / smoothingSize;
    }
  }

  return motorRPM;
}

/**
 * @brief Calculates the gear ratio from motor and wheel RPM.
 * @return The calculated gear ratio.
 */
float calculateGearRatio() {
  float tempGearRatio = 0;
  if (wheelRPM) {
    tempGearRatio = motorRPM / wheelRPM;
  }
  return (tempGearRatio);
}

/**
 * @brief Converts a thermistor ADC reading to Celsius using the Steinhart-Hart equation.
 *
 * This method uses the Steinhart-Hart equation to calculate the actual
 * temperature, which requires three coefficients (A, B and C) specific to the thermistor.
 *
 * @param rawADC The raw ADC value from the thermistor pin.
 * @param thermNumber The thermistor number (1 or 2) to select calibration constants.
 * @return The calculated temperature in Celsius.
 * @see http://playground.arduino.cc/ComponentLib/Thermistor2
 */
float thermistorADCToCelcius(float rawADC, uint8_t thermNumber) {

  // If no sensor is plugged in, rawADC reading will be close to 1023, so return 0.
  if (rawADC > 1000)
    return (0);

  // Steinhart-Hart Coefficients, see comment above
  // These coefficients are for the MF52AT NTC 10k thermistor, however due to
  // thermistor tolerances each thermistor should be calibrated individually.
  float A, B, C;
  if (thermNumber == 1) {
    A = CAL_THERM1_A;
    B = CAL_THERM1_B;
    C = CAL_THERM1_C;
  } else {
    A = CAL_THERM2_A;
    B = CAL_THERM2_B;
    C = CAL_THERM2_C;
  }
  // Value of resistor forming potential divider with Thermistor in ohms.
  const int FIXED_RESISTOR_VALUE = 10000; // 10k
  // Calculations:
  // The formula is: Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}
  // where A, B and C are the coefficients above and R is the resistance across
  // the thermistor. First step is to calculate the resistance of the thermistor
  // using the potential divider equation V_out = (R1 + R2)/(R1 * R2)*V_in As R2
  // is the only unknown variable we can re-write this as: R2 = R1((V_in/V_out)-1).
  // R2 = (R1 V2)/(V1-V2) As the ADC values are our readings of the voltage, we can
  // substitute V_in with 1024 and V_out with the reading taken from the ADC, which
  // is passed into this function as rawADC This makes the calculation:
  // The divider is fed from the same rail the ADC references, so this is
  // ratiometric: the rail voltage cancels out and only the ratio matters. Half
  // an LSB is added for the same reason as in adcToPinVoltage().
  float correctedADC = rawADC + 0.5;
  float thermistorResistance = ((float)FIXED_RESISTOR_VALUE * correctedADC) / (1024.0 - correctedADC);
  // Next, you'll notice that the log natural (ln) of this resistance needs to
  // be calculated 4 times in the Steinhart-Hart equation. This is a complex and
  // long calculation for the arduino. As such it is efficient to do it once and
  // save the result for use later:
  double lnResistance = log(thermistorResistance);
  // Now plug it all into the equation:
  double temperature = 1 / (A + (B * lnResistance) +
                            (C * lnResistance * lnResistance * lnResistance));
  // We now have the temperature in Kelvin. To convert it into Celcius we need
  // to subtract 273.15
  temperature = temperature - 273.15;
  // if (DEBUG_MODE)
  // {
  //         SerialA.print("\n\rThemistore Resistance = ");
  //         SerialA.println(thermistorResistance);
  //         SerialA.print("Temperature = ");
  //         SerialA.println(temperature);
  // }
  // Now return the Celcius Value:
  return (temperature);
}

/**
 * @brief Sends data packet over USART to the bluetooth module.
 *
 * These functions handle packeting the data and sending it over USART.
 * There are two versions: one for float values and one for integer values.
 *
 * @param identifier The data identifier (see Globals.h).
 * @param value The value to send.
 */

/**
 * @brief Sends a float value over Bluetooth using a custom packet format.
 * @param identifier The data identifier.
 * @param value The float value to send.
 */
void sendData(char identifier, float value) {
  if (!DEBUG_MODE) // Only runs if debug mode is LOW (0)
  {
    byte dataByte1;
    byte dataByte2;
    if (value <= 127) {
      // Values under 128 are sent as a float
      // i.e. value = dataByte1 + dataByte2 / 100
      int integer;
      int decimal;
      float tempDecimal;
      integer = (int)value;
      tempDecimal = (value - (float)integer) * 100;
      decimal = (int)tempDecimal;
      dataByte1 = (byte)integer;
      dataByte2 = (byte)decimal;
    } else {
      // Values above 127 are sent as integer
      // i.e. value = dataByte1 * 100 + dataByte2
      int tens;
      int hundreds;
      hundreds = (int)(value / 100);
      tens = value - hundreds * 100;
      dataByte1 = (byte)hundreds;
      // dataByte1 = dataByte1 || 0x10000000; //flag for integer send value
      dataByte1 += 128;
      dataByte2 = (byte)tens;
    }
    // Send the data in the format { [id] [1] [2] }
#ifdef NANO_EVERY
    if (inConfig) {
      Serial.write(123);
      Serial.write(identifier);
      Serial.write(dataByte1);
      Serial.write(dataByte2);
      Serial.write(125);
    }
#endif
    SerialA.write(123);
    SerialA.write(identifier);
    SerialA.write(dataByte1);
    SerialA.write(dataByte2);
    SerialA.write(125);
  } else {
    SerialA.print("Data Out: \t");
    SerialA.print(identifier);
    SerialA.print(",\t");
    SerialA.println(value);
  }
}

/**
 * @brief Sends an integer value over Bluetooth using a custom packet format.
 * @param identifier The data identifier.
 * @param value The integer value to send.
 */
void sendData(char identifier, int value) {
  if (!DEBUG_MODE) {
    byte dataByte1;
    byte dataByte2;
    if (value <= 127) {
      dataByte1 = (byte)value;
      dataByte2 =
          0; // we know there's no decimal component as an int was passed in
    } else {
      int tens;
      int hundreds;
      hundreds = (int)(value / 100);
      tens = value - (hundreds * 100);
      dataByte1 = (byte)hundreds;
      dataByte1 += 128; // sets MSB High to indicate Integer value
      dataByte2 = (byte)tens;
    }
#ifdef NANO_EVERY
    if (inConfig) {
      Serial.write(123);
      Serial.write(identifier);
      Serial.write(dataByte1);
      Serial.write(dataByte2);
      Serial.write(125);
    }
#endif
    SerialA.write(123);
    SerialA.write(identifier);
    SerialA.write(dataByte1);
    SerialA.write(dataByte2);
    SerialA.write(125);
  } else {

    char buffer[16];
    sprintf(buffer, "%d", value);
    hardwareSerialPrint("Data Out: \t");
    char idStr[2] = {identifier, '\0'};
    hardwareSerialPrint(idStr);
    hardwareSerialPrint(",\t");
    hardwareSerialPrintln(buffer);
  }
}

/**
 * @brief Configures the HC-05 Bluetooth module with name, baud rate, and
 * password.
 */
void configureBluetooth() {

  Serial.println(F("Attempting to Configure Bluetooth (HC-05 Module)"));

  flushSerial();

  SerialA.begin(38400); // AT mode baud rate
  while (!SerialA) {
  } // Wait for serial to initialise

  uint8_t atMode = 0;

#ifdef JUMPER_BT_EN // PCBV2 - Automatically set BT AT Mode by setting EN pin
                    // HIGH
  digitalWrite(BT_EN_PIN, HIGH);
  delay(100);
#endif

  if (atModeCheck()) {

    atMode = 1;
    hardwareSerialPrintln(F("HC-05 AT MODE Entered"));
  } else { // If AT Mode not entered, send error messages, and exit config
           // gracefully
    hardwareSerialPrintln(F("HC-05 not in AT Mode"));
    hardwareSerialPrintln(F("To program HC-05 Module, perform a cold boot."));

#ifdef JUMPER_BT_EN
    digitalWrite(BT_EN_PIN, LOW);
#endif

    SerialA.println(
        F("AT+RESET\r\n")); // Unlikely event - Just in case it actually had
                            // entered, attempt to exit.
    return;
  }

  uint8_t btNameSet =
      0; // These will be set to 1 when each is successfully updated
  uint8_t btBaudSet = 0;
  uint8_t btPassSet = 0;

  // Get and print HC-05 Firmware Version
  char response[64] = {0};
  flushSerial();
  SerialA.print(F("AT+VERSION?\r\n"));
  SerialA.flush();    // Waits for transmission to end
  waitForSerial(500); // Waits for start of response with 500ms timeout
  delay(50);          // Now waits to ensure full response is recieved

  uint8_t index = 0;
  while (SerialA.available() && index < 63) {
    char c = SerialA.read();
    if (c != '\r' && c != '\n') {
      response[index++] = c;
    }
  }
  response[index] = '\0';

  hardwareSerialPrint(F("HC-05 Firmware Version: "));
  hardwareSerialPrintln(response);

  if (strcmp(response, "+VERSION:hc05V2.3_le OK") == 0) {
    hardwareSerialPrintln(F("\n\r******************"));
    hardwareSerialPrintln(F("WARNING - This is not an HC-05 compatible Bluetooth "
                            "module as it uses BLE instead of Bluetooth 2.0"));
    hardwareSerialPrintln(F("The eChook Nano CANNOT work with this module. Setup will "
                            "not continue."));
    hardwareSerialPrintln(
        F("Please see https://docs.echook.uk/troubleshooting/bluetooth"));
    hardwareSerialPrintln(F("******************\n\r"));
    digitalWrite(13, HIGH);
    while (1) {
    }
  }

  // Set Bluetooth Name
  flushSerial(); // Flush the buffer. Not entirely sure what is in there to
                 // flush at this point, but it is needed!

  SerialA.print(F("AT+NAME=\""));
  SerialA.print(CAL_BT_NAME);
  SerialA.print(F("\"\r\n"));
  SerialA.flush(); // Wait for transmission to end
  // Now Check Response
  waitForSerial(100);
  delay(50);
  index = 0;
  while (SerialA.available() && index < 63) {
    char c = SerialA.read();
    if (c != '\r' && c != '\n') {
      response[index++] = c;
    }
  }
  response[index] = '\0';
  if (strcmp(response, "OK") == 0) {
    hardwareSerialPrintln("HC-05 Name Set");
    btNameSet = 1;
  } else {
    hardwareSerialPrintln(F("ERROR - HC-05 Name NOT Set"));
  }

  // Set Baud Rate_____________________________________________
  // delay(100);
  flushSerial();
  SerialA.print(F("AT+UART=")); // command to change BAUD rate
  SerialA.print(CAL_BT_BAUDRATE);
  SerialA.println(F(",0,0")); // Parity and Stop bits
  SerialA.flush();            // Wait for transmission to end
  // Now Check Response.
  waitForSerial(100);
  delay(50);
  index = 0;
  while (SerialA.available() && index < 63) {
    char c = SerialA.read();
    if (c != '\r' && c != '\n') {
      response[index++] = c;
    }
  }
  response[index] = '\0';
  if (strcmp(response, "OK") == 0) {
    hardwareSerialPrintln(F("HC-05 Baudrate Set"));
    btBaudSet = 1;
  } else {
    hardwareSerialPrintln(F("ERROR - HC-05 Baud Rate NOT Set"));
  }

  // Set Bluetooth Password
  flushSerial(); // Flush the serial input buffer

  SerialA.print(F("AT+PSWD=\""));
  SerialA.print(CAL_BT_PASSWORD);
  SerialA.print(F("\"\r\n"));

  SerialA.flush(); // Wait for transmission to end
  // Now Check Response
  waitForSerial(100);
  delay(50);
  index = 0;
  while (SerialA.available() && index < 63) {
    char c = SerialA.read();
    if (c != '\r' && c != '\n') {
      response[index++] = c;
    }
  }
  response[index] = '\0';
  if (strcmp(response, "OK") == 0) {
    hardwareSerialPrintln(F("HC-05 Password Set"));
    btPassSet = 1;
  } else {
    hardwareSerialPrintln(F("ERROR - HC-05 Password NOT Set with error: "));
    hardwareSerialPrintln(response);
  }

  // Check all operations completed successfully
  if (btBaudSet && btNameSet && btPassSet) {
    flushSerial();
#ifdef JUMPER_BT_EN
    digitalWrite(BT_EN_PIN, LOW);
#endif
    hardwareSerialPrintln(F("HC-05 Configuration Successful, Resetting..."));
    delay(100);
    SerialA.println(F("AT+RESET\r\n")); // has to be in the middle to provide a
                                        // suitable delay before and after
    SerialA.flush();                    // Wait for transmission to end.
  }

  // Test if AT mode has successfuly exited, if not, reset until it does!
  digitalWrite(13, HIGH);

  while (atMode) {
    delay(100);
    flushSerial();
    SerialA.println(F("AT+RESET\r\n")); // has to be in the middle to provide a
                                        // suitable delay before and after
    SerialA.flush();
    delay(100);
    atMode = atModeCheck();
  }
  // Send the reset command.
  delay(100);
  SerialA.println(F("AT+RESET\r\n")); // has to be in the middle to provide a
                                      // suitable delay before and after
  SerialA.flush();

  SerialA.begin(CAL_BT_BAUDRATE); // reset baud rate
  while (!SerialA) {
  } // wait while serial is inialising
  return;
}

/**
 * @brief Checks if the HC-05 Bluetooth module is in AT mode.
 * @return 1 if in AT mode, 0 otherwise.
 */
int atModeCheck() {
  flushSerial();
  SerialA.print(F("AT\r\n"));
  SerialA.flush();    // Waits for transmission to end
  waitForSerial(100); // Waits for start of response with 500ms timeout
  delay(50);          // Now waits to ensure full response is recieved
  char response[64] = {0};
  uint8_t index = 0;
  while (SerialA.available() && index < 63) {
    char c = SerialA.read();
    if (c != '\r' && c != '\n') {
      response[index++] = c;
    }
  }
  response[index] = '\0';
  if (strcmp(response, "OK") == 0) {
    return 1;
  } else {
    return 0;
  }
}

/**
 * @brief Flushes the SerialA read buffer.
 */
void flushSerial() { // SerialA.flush() flushes the write buffer, this function
                     // manually flushes the read buffer.
  while (SerialA.available()) {
    SerialA.read();
  }
}

/**
 * @brief Waits for SerialA to become available or until a timeout is reached.
 * @param timeOut Timeout in milliseconds.
 */
void waitForSerial(int timeOut) {
  unsigned long tempTime = millis() + timeOut;
  while (!SerialA.available() && millis() < tempTime) {
  } // Do nothing - i.e. wait until serial becomes availble or the timeout is
    // reached.
}