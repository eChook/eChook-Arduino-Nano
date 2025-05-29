// INTERRUPT SERVICE ROUTINES
// This function is triggered every time the magnet on the motor shaft passes the Hall effect
// sensor. Care must be taken to ensure the sensor is position the correct way and so is the
// magnet as it will only respond in a specific orientation and magnet pole.
// It is best practice to keep ISRs as small as possible.

void motorSpeedISR()
{
  unsigned long currentTime = micros();
  lastMotorInterval = currentTime - lastMotorPollTime;
  lastMotorPollTime = currentTime;
  newMotorSignal = true;
}

void wheelSpeedISR()
{
  unsigned long currentTime = micros();
  lastWheelInterval = currentTime - lastWheelPollTime;
  lastWheelPollTime = currentTime;
  newSpeedSignal = true;
}