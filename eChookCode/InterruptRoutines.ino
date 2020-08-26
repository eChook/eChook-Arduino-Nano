// INTERRUPT SERVICE ROUTINES
//This function is triggered every time the magnet on the motor shaft passes the Hall effect
//sensor. Care must be taken to ensure the sensor is position the correct way and so is the
//magnet as it will only respond in a specific orientation and magnet pole.
//It is best practice to keep ISRs as small as possible.
void motorSpeedISR()
{
        if(USE_IMPROVED_RPM_CALCULATION) {
                unsigned long intervalTemp = micros() - lastMotorPollTime;
                if(intervalTemp > 2000) { //under 20ms, assume bounce/noise
                        lastMotorInterval = intervalTemp;
                        lastMotorPollTime = micros();
                }
        }else {
                motorPoll++;
        }
}

void wheelSpeedISR()
{
        if(USE_IMPROVED_SPEED_CALCULATION) {
                unsigned long intervalTemp = millis() - lastWheelPollTime;
                if(intervalTemp > 20) { //under 20ms, assume bounce/noise
                        lastWheelInterval = intervalTemp;
                        lastWheelPollTime = millis();
                }
        }else {
                wheelPoll++;
        }
}
