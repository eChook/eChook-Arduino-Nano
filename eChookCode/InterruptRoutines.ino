// INTERRUPT SERVICE ROUTINES
//This function is triggered every time the magnet on the motor shaft passes the Hall effect
//sensor. Care must be taken to ensure the sensor is position the correct way and so is the
//magnet as it will only respond in a specific orientation and magnet pole.
//It is best practice to keep ISRs as small as possible.
void motorSpeedISR()
{
        if(USE_IMPROVED_RPM_CALCULATION) {
                lastMotorInterval = millis() - lastMotorPollTime;
                lastMotorPollTime = millis();
        }else {
                motorPoll++;
        }
}

void wheelSpeedISR()
{
        if(USE_IMPROVED_SPEED_CALCULATION) {
                lastWheelInterval = millis() - lastWheelPollTime;
                lastWheelPollTime = millis();
        }else {
                wheelPoll++;
        }
}
