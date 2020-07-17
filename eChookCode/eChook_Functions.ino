void eChookSetup(){
        pinSetup(); //Sets Input/Output for all pins. Function found in eChook_Functions.ino.

        //Initialise debounce objects for the three buttons
        cycleButtonDebounce.attach(CYCLE_BTN_IN_PIN);
        cycleButtonDebounce.interval(buttonDebounceTime);

        launchButtonDebounce.attach(LAUNCH_BTN_IN_PIN);
        launchButtonDebounce.interval(buttonDebounceTime);

        brakeButtonDebounce.attach(BRAKE_IN_PIN);
        brakeButtonDebounce.interval(buttonDebounceTime);

        /**
         * Initialise Serial Communication
         * If communication over bluetooth is not working or the results are garbled it is likely the
         * baud rate set here (number in brackets after Serial.begin) and the baud rate of the bluetooth
         * module aren't set the same.
         *
         * A good tutorial for altering the HC-05 Bluetooth Module parameters is here:
         * http://www.instructables.com/id/Modify-The-HC-05-Bluetooth-Module-Defaults-Using-A/
         *
         * The HC-05 modules commonly come preset with baud rates of 9600 or 32000
         *
         * Alternatively the following bit of code will attempt to automatically configure a
         * HC-05 module if it is plugged in in AT (setup) mode then the arduino is reset. (Power Arduino,
         * unplug HC-05 module, press button on HC-05 module, plug back in holding button [light should blink slowly],
         * release button, then reset Arduino)
         */

        if (checkBtAtMode()) // checks if the arduino is in AT mode
        {
                configureBluetooth(); // If AT mode is set, configure according to the BT_xxx constants defined above
        }

        Serial.begin(CAL_BT_BAUDRATE); // Bluetooth and USB communications

        lastShortDataSendTime = millis(); //Give the timing a start value.
}

void pinSetup(){
        //Set up pin modes for all inputs and outputs
        pinMode(MOTOR_OUT_PIN,        OUTPUT);
        digitalWrite(MOTOR_OUT_PIN,   LOW);// Ensure motor is not driven on startup

        pinMode(VBATT_IN_PIN,         INPUT);
        pinMode(VBATT1_IN_PIN,        INPUT);
        pinMode(THROTTLE_IN_PIN,      INPUT);
        pinMode(AMPS_IN_PIN,          INPUT);
        pinMode(TEMP1_IN_PIN,         INPUT);
        pinMode(TEMP2_IN_PIN,         INPUT);


        pinMode(LAUNCH_BTN_IN_PIN,    INPUT_PULLUP);
        pinMode(CYCLE_BTN_IN_PIN,     INPUT_PULLUP);
        pinMode(BRAKE_IN_PIN,         INPUT_PULLUP);//input type will depend on implementation of brake light

        /**
         * Set up Interrupts:
         * When the specified digital change is seen on a the interrupt pin it will pause the main loop and
         * run the code in the Interrupt Service Routine (ISR) before resuming the main code.
         * The interrupt number is not the pin number on the arduino Nano. For explanation see here:
         * https://www.arduino.cc/en/Reference/AttachInterrupt
         */

        attachInterrupt(0, motorSpeedISR, RISING);
        attachInterrupt(1, wheelSpeedISR, RISING);

}

void eChookRoutinesUpdate(){

        // We want to check different variables at different rates. For most variables 0.25 seconds will be good for logging and analysis.
        // Certain variables either can't have or do not need this resolution.
        // Wheel and Motor speed are accumulated over time, so the longer time left between samples, the higher the resolution of the value.
        // As such, these are only updated ever 1 second. Temperature is a reading that will not change fast, and consumes more processing
        // time to calculate than most, so this is also checked every 1s.

        if(millis() > nextThrottleReadMs) { // millis() gives milliseconds since power on. If this is greater than the nextThrottleReadMs we've calculated it will run.
                nextThrottleReadMs = millis() + 100; //100 ms, 10hz
                throttle = readThrottle(); // if this is being used as the input to a motor controller it is recommended to check it at a higher frequency than 4Hz
        }

        if (millis() - lastShortDataSendTime > CAL_DATA_TRANSMIT_INTERVAL) //i.e. if 250ms have passed since this code last ran
        {
                lastShortDataSendTime = millis(); //this is reset at the start so that the calculation time does not add to the loop time
                loopCounter = loopCounter + 1; // This value will loop 1-4, the 1s update variables will update on certain loops to spread the processing time.
                //It is recommended to leave the ADC a short recovery period between readings (~1ms). To achieve this we can transmit the data between readings
                batteryVoltageTotal = readVoltageTotal();
                sendData(VOLTAGE_ID, batteryVoltageTotal);

                batteryVoltageLower = readVoltageLower();
                sendData(VOLTAGE_LOWER_ID, batteryVoltageLower);

                current = readCurrent();
                sendData(CURRENT_ID, current);

                sendData(THROTTLE_INPUT_ID, throttle);

                if (loopCounter == 1)
                { //Functions to run every 1st loop
                        tempOne = readTempOne();
                        sendData(TEMP1_ID, tempOne);
                        digitalWrite(13, HIGH); //these are just flashing the LEDs as visual confimarion of the loop
                        digitalWrite(9, HIGH);
                }

                if (loopCounter == 2)
                { //Functions to run every 2nd loop
                        tempTwo = readTempTwo();
                        sendData(TEMP2_ID, tempTwo);
                        digitalWrite(6, HIGH);
                }

                if (loopCounter == 3)
                { //Functions to run every 3rd loop
                  //nothing actaully needed to do at .75 seconds
                        digitalWrite(13, LOW);
                        digitalWrite(9, LOW);
                }

                if (loopCounter == 4)
                { //Functions to run every 4th loop
                        loopCounter = 0; //4 * 0.25 makes one second, so counter resets

                        wheelSpeed = readWheelSpeed();
                        sendData(SPEED_ID, wheelSpeed);

                        motorRPM = readMotorRPM();
                        sendData(MOTOR_ID, motorRPM);

                        gearRatio = calculateGearRatio();
                        sendData(GEAR_RATIO_ID, gearRatio);

                        digitalWrite(6, LOW);
                }

        }

}

void buttonChecks()
{ // Checks state of each button, if a press is detected sends the data over bluetooth
        cycleButtonDebounce.update();
        launchButtonDebounce.update();
        brakeButtonDebounce.update();
        int cycleButtonState = !cycleButtonDebounce.read(); //Buttons are LOW when pressed, ! inverts this, so state is HIGH when pressed
        if(cycleButtonState != cycleButtonPrevious) //Button has changed state - either pressed or depressed
        {
                if(cycleButtonState == HIGH) //Button Pressed
                {
                        sendData(CYCLE_VIEW_ID, 1); // Actual packet content is irrelevant - sending a packet with the ID represents a button press
                }
                //We don't care when button is released
                cycleButtonPrevious = cycleButtonState; //Update previous state
        }

        int launchButtonState = !launchButtonDebounce.read(); //Buttons are LOW when pressed, ! inverts this, so state is HIGH when pressed
        if(launchButtonState != launchButtonPrevious) //Button has changed state - either pressed or depressed
        {
                if(launchButtonState == HIGH) //Button Pressed
                {
                        sendData(LAUNCH_MODE_ID, 1); // Actual packet content is irrelevant - sending a packet with the ID represents a button press
                }
                //We don't care when button is released
                launchButtonPrevious = launchButtonState; //Update previous state
        }

        int brakeButtonState = !brakeButtonDebounce.read(); //Buttons are LOW when pressed, ! inverts this, so state is HIGH when pressed
        if(brakeButtonState != brakeButtonPrevious) //Button has changed state - either pressed or depressed
        {
                if(brakeButtonState == HIGH) //Button Pressed
                {
                        sendData(BRAKE_PRESSED_ID, 100);
                }
                else if(brakeButtonState == LOW) //Button Released
                {
                        sendData(BRAKE_PRESSED_ID, 0);
                }
                brakeButtonPrevious = brakeButtonState; //Update previous state
        }
}

// Sensor Reading Functions

float readVoltageTotal()
{ //Reads in the ADC value for the 24v input, converts it to a voltage, returns the voltage value.
        float tempVoltage = analogRead(VBATT_IN_PIN); //this will give a 10 bit value of the voltage with 1024 representing the ADC reference voltage of 5V
        tempVoltage = (tempVoltage / 1024) * CAL_REFERENCE_VOLTAGE; //This gives the actual voltage seen at the arduino pin, assuming reference voltage of 5v
        tempVoltage = tempVoltage * CAL_BATTERY_TOTAL; //Gives battery voltage where 6 is the division ratio of the potential divider. NEEDS TUNING!!
        return (tempVoltage);
}

float readVoltageLower()
{ //Reads in the ADC value for the 12v input, converts it to a voltage, returns the voltage value.
        float tempVoltage = analogRead(VBATT1_IN_PIN); //this will give a 10 bit value of the voltage with 1024 representing the ADC reference voltage of 5V
        tempVoltage = (tempVoltage / 1024) * CAL_REFERENCE_VOLTAGE; //This gives the actual voltage seen at the arduino pin, assuming reference voltage of 5v
        tempVoltage = tempVoltage * CAL_BATTERY_LOWER; //Gives battery voltage where 3 is the division ratio of the potential divider. NEEDS TUNING!!
        return (tempVoltage);
}

float readCurrent()
{ // Reads in ACC input from the differential amplifier, converts it to the current value and smooths it
        float tempCurrent = analogRead(AMPS_IN_PIN);
        tempCurrent = (tempCurrent / 1024) * CAL_REFERENCE_VOLTAGE; //gives voltage output of current sensor.
        tempCurrent = tempCurrent * CAL_CURRENT; //calibration value for LEM current sensor on eChook board.
        currentSmoothingArray[currentSmoothingCount] = tempCurrent; //updates array with latest value
        // The next 5 lines manage the smoothing count for the averaging:
        currentSmoothingCount++; //increment smoothing count
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

int readThrottle()
{
        int currThrtlOut = throttle;
        float tempThrottle = analogRead(THROTTLE_IN_PIN);
        if(CAL_THROTTLE_VARIABLE) //Analogue throttle, not push button
        {
                tempThrottle = (tempThrottle / 1023) * CAL_REFERENCE_VOLTAGE; // Gives the actual voltage seen on the arduino Pin, assuming reference voltage of 5V
                Serial.print(tempThrottle);
                Serial.print(", ");
                //The following code adds dead bands to the start and end of the throttle travel
                if (tempThrottle < CAL_THROTTLE_LOW) //less than 1V
                {
                        tempThrottle = CAL_THROTTLE_LOW;
                }
                else if (tempThrottle > CAL_THROTTLE_HIGH) //greater than 4 V
                {
                        tempThrottle = CAL_THROTTLE_HIGH;
                }

                tempThrottle = ((tempThrottle-CAL_THROTTLE_LOW)/(float)(CAL_THROTTLE_HIGH-CAL_THROTTLE_LOW))*(255);
        }else{
                if(tempThrottle > 200) //Approx 1v
                {
                        tempThrottle = 255; //full throttle
                }else
                {
                        tempThrottle = 0; //No throttle
                }
        }

        if(CAL_THROTTLE_RAMP) {
                // This code generates a simple ramp up in throttle. The >100 is there as it will likely take about 30% throttle to get the car moving, so this will give a quicker start.
                if(tempThrottle >= currThrtlOut && tempThrottle > 100) { // This could be if(thrtlIn > thrtlOut && speed < threshold) to make it low speed only. Speed and threshold are undefined in this example!
                        if(currThrtlOut<100) {
                                currThrtlOut = 100;
                        }
                        currThrtlOut += 4; // Value dictates ramp speed. Calculated by (155/x)/10. 4 gives (155/4)/10=3.875 seconds, 2 gives 7.75 seconds, 1 gives 15.5 seconds
                        if(currThrtlOut > tempThrottle) { //Fixes the throttle jitter if the increment puts output over request.
                                currThrtlOut = tempThrottle;
                        }
                } else {
                        currThrtlOut = tempThrottle;
                }
        } else{
                currThrtlOut = tempThrottle;
        }
        analogWrite(MOTOR_OUT_PIN, currThrtlOut); //This drives the motor output. Unless you are using the board to drive your motor you can comment it out.
        return (currThrtlOut);
}

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

float readWheelSpeed()
{
        float wheelSpeedMetersPerSecond = 0;

        if(USE_IMPROVED_SPEED_CALCULATION) { // Two ways to calculate wheel RPM
                // Calculate time taken for last wheel rotation
                float wheelRPS = 0;
                if(lastWheelInterval < 15000) { //If it takes longer than 5s for a rotation, consider it stopped.
                        long fullRotationMs = lastWheelInterval * CAL_WHEEL_MAGNETS; // Ideally one motor magnet
                        wheelRPS = (float)1000 / fullRotationMs; // 1 second, divided by time of one rotation at current speed
                }
                wheelSpeedMetersPerSecond = wheelRPS *  (float) CAL_WHEEL_CIRCUMFERENCE;
        }else {
                //Counts the number of magnet passesdetected since the last wheel speed check,
                // converts to a speed in meters per second and returns that value.
                // Also updates the global wheel RPM value each call.

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
                wheelSpeedMetersPerSecond = wheelDistanceTravelled / ((float)(tempWheelPollTime - tempLastWheelPollTime) / (float)1000.0); // the /1000 converts the milliseconds to seconds
        }


        //Next section of code handles the smooting:
        speedSmoothingArray[speedSmoothingCount] = wheelSpeedMetersPerSecond; //adds current speed into oldest position in array
        speedSmoothingCount++; //incrememnts array position for next reading
        if (speedSmoothingCount >= speedSmoothingSetting)
        {
                speedSmoothingCount = 0; //reset if count exceeds array length
        }
        wheelSpeedMetersPerSecond = 0; //reset variable ready to sum array
        for (int i = 0; i < speedSmoothingSetting; i++)
        {
                wheelSpeedMetersPerSecond += speedSmoothingArray[i];
        }
        wheelSpeedMetersPerSecond = wheelSpeedMetersPerSecond / speedSmoothingSetting; //divide summed figure by array count to get mean value
        return (wheelSpeedMetersPerSecond); //return smoothed value

}

float readMotorRPM()
{
        if(USE_IMPROVED_RPM_CALCULATION) {
                float tempRpm = 0;
                // Calculate time taken for last wheel rotation
                if(lastMotorInterval < 5000) { //If it takes longer than 5s for a rotation, consider it stopped.
                        long fullRotationMs = lastMotorInterval * CAL_MOTOR_MAGNETS; // Ideally one motor magnet
                        tempRpm = (float)60000 / fullRotationMs; // 1 minute, divided by time of one rotation at current speed
                }
                return(tempRpm);
        }else {
                //Counts the number of magnet passesdetected since the last Motor rpm check, converts to revolutions per minute and returns that value
                // First action is to take copies of the motor poll count and time so that the variables don't change during the calculations.
                int tempMotorPoll = motorPoll;
                motorPoll = 0;
                unsigned long tempLastMotorPollTime = lastMotorSpeedPollTime;
                unsigned long tempMotorPollTime = millis();
                lastMotorSpeedPollTime = tempMotorPollTime;
                // Now calculate the number of revolutions of the motor shaft
                float motorRevolutions = (float)tempMotorPoll / CAL_MOTOR_MAGNETS;
                float motorRevolutionsPerMin = motorRevolutions * 60.0;
                float timeDiffms = tempMotorPollTime - tempLastMotorPollTime;
                float timeDiffs = timeDiffms / 1000.0;
                // Now use the time time passed to convert this to revolutions per minute
                // RMP = (revolutions / latestPollTIme - lastPollTime) / 1000 to convert to Seconds) * 60 to convert to minutes
                float motorShaftRPM = motorRevolutionsPerMin / timeDiffs;
                return (motorShaftRPM);
        }
}

float calculateGearRatio()
{
        float tempGearRatio = motorRPM/wheelRPM;
        return(tempGearRatio);
}

// Thermistor calculation
// If an active sensor such as a TMP37 is used this has a calibrated voltage output, linear to the temperature change.
// A cheaper option is to use Thermistors. The resistance across a thrmistor changes with temperature, but the change is not linear
// so some maths is required to translate the voltage reading into a temperature value.
// For more information see here: http://playground.arduino.cc/ComponentLib/Thermistor2
// This method uses the Steinhart-Hart equation to calculate the actual temperature, however this requires three coefficients,
// A, B and C, that are specific to a thermistor. The ones below are for the thermistors provided with the board, however if you
// use a different thermistor the coefficients should be given in the datasheet, and if not, can be calculated using this calculator:
// http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm

float thermistorADCToCelcius(int rawADC)
{
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
        float thermistorResistance = ((float)FIXED_RESISTOR_VALUE * (float)rawADC)/ (float)((float)1023 - (float)rawADC);
        // Next, you'll notice that the log natural (ln) of this resistance needs to be calculated 4 times in the Steinhart-Hart equation. This is a complex and long calculation for the arduino.
        // As such it is efficient to do it once and save the result for use later:
        double lnResistance = log(thermistorResistance);
        // Now plug it all into the equation:
        double temperature = 1 / (A + (B * lnResistance) + (C * lnResistance * lnResistance * lnResistance));
        // We now have the temperature in Kelvin. To convert it into Celcius we need to subtract 273.15
        temperature = temperature - 273.15;
        // if (DEBUG_MODE)
        // {
        //         Serial.print("\n\rThemistore Resistance = ");
        //         Serial.println(thermistorResistance);
        //         Serial.print("Temperature = ");
        //         Serial.println(temperature);
        // }
        // Now return the Celcius Value:
        return (temperature);
}

// BLUETOOTH DATA PACKETING FUNCTIONS
// The two functions in this section handle packeting the data and sending it over USART to the bluetooth module. The two functions are
//  identically named so are called the in the same way, however the first is run if the value passed to it is a float and the second is
//  run if the value passed into it is an integer (an override function). For all intents and purposes you can ignore this and simply call
//  'sendData(identifier, value);' using one of the defined identifiers and either a float or integer value to send informaion over BT.
//
// identifier:  see definitions in the Globals.h file
// value:       the value to send (typically some caluclated value from a sensor)

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
        }else{
          Serial.print(identifier);
          Serial.print(", ");
          Serial.println(value);
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
                        dataByte1 += 128; //sets MSB High to indicate Integer value
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
        } else{
          Serial.print(identifier);
          Serial.print(", ");
          Serial.println(value);
        }
}


// HC-05 CONFIGURATION FUNCTIONS
int checkBtAtMode() //Checks if the HC-05 Bluetooth module is in AT mode. Returns 1 if it is, 0 otherwise
{
        int atMode = 0;
        Serial.begin(38400); //AT mode baud rate
        while(!Serial) {} //Wait for serial to initialise
        delay(200);
        Serial.print("AT\r\n"); // for some reason when AT is sent the first time, an error is always returned.
        delay(200); //wait for response
        flushSerial();
        while (Serial.available()) // to check if the flush worked
        {
                Serial.println((char)Serial.read());
        }
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

        Serial.begin(CAL_BT_BAUDRATE); //reset baud rate
        while (!Serial) {} //wait while serial is inialising
        return (atMode);
}

void configureBluetooth()
{
        //Assumes AT mode has been confirmed.
        Serial.begin(38400);
        uint8_t btNameSet = 0; //These will be set to 1 when each is successfully updated
        uint8_t btBaudSet = 0;
        //Set Bluetooth Name
        flushSerial();// Flush the buffer. Not entirely sure what is in there to flush at this point, but it is needed!
        Serial.print("AT+NAME=");
        Serial.print(CAL_BT_NAME);
        Serial.print("\r\n");
        //Now Check Response
        waitForSerial(500);
        char tempOne = (char)Serial.read();
        waitForSerial(500);
        char tempTwo = (char)Serial.read();
        if (tempOne == 'O' && tempTwo == 'K') //Was the response "OK"?
        {
                // Serial.println("Name Set");
                btNameSet = 1;
        }
        else
        {
                // Serial.println("Name Not Set");
                // Serial.print("Char 1 = ");
                // Serial.println(tempOne);
                // Serial.print("Char 2 = ");
                // Serial.print(tempTwo);
                // Serial.print("\r\n");
        }
        //Set Baud Rate_____________________________________________
        delay(100);
        flushSerial();
        Serial.print("AT+UART="); //command to change BAUD rate
        Serial.print(CAL_BT_BAUDRATE);
        Serial.println(",0,0"); //Parity and Stop bits
        //Now Check Response.
        waitForSerial(500);
        tempOne = (char)Serial.read();
        waitForSerial(500);
        tempTwo = (char)Serial.read();
        if (tempOne == 'O' && tempTwo == 'K') //Was the response "OK"?
        {
                // Serial.println("Baud Rate Set");
                btBaudSet = 1;
        }
        else
        {
                // Serial.println("Baud Rate Not Set");
                // Serial.print("Char 1 = ");
                // Serial.println(tempOne);
                // Serial.print("Char 2 = ");
                // Serial.println(tempTwo);
        }
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
                // Serial.println("AT+RESET\r\n"); // has to be in the middle to provide a suitable delay before and after
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
        Serial.begin(CAL_BT_BAUDRATE);
}


void flushSerial()
{ //Serial.flush() flushes the write buffer, this function manually flushes the read buffer.
        while (Serial.available())
        {
                // char temp = Serial.read();
                Serial.read();
        }
}

void waitForSerial(int timeOut)
{
        unsigned long tempTime = millis() + timeOut;
        while (!Serial.available() && millis() < tempTime) {} //Do nothing - i.e. wait until serial becomes availble or the timeout is reached.
}
