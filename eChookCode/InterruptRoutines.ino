/**
 * @file InterruptRoutines.ino
 * @brief Interrupt Service Routines (ISRs) for pulse counting.
 *
 * These functions handle low-level pulse counting for motor and wheel sensors.
 * best practice is kept by keeping these routines as short as possible.
 */

/**
 * @brief ISR for motor speed hall effect sensor.
 * Triggered on rising edge of pulses from the motor shaft.
 * Includes a 2ms software debounce (max 30k RPM).
 */
void motorSpeedISR() {
  unsigned long now = micros();
  unsigned long interval = now - lastMotorPollTime;
  if (interval > 2000) // Debounce 2ms (Max 30k RPM)
  {
    lastMotorInterval = interval;
    lastMotorPollTime = now;
    newMotorSignal = true;
  }
}

/**
 * @brief ISR for wheel speed sensor.
 * Triggered on rising edge of pulses from the wheel hub/sprocket magnets.
 * Includes a 10ms software debounce (max 100 RPS).
 */
void wheelSpeedISR() {
  unsigned long now = micros();
  unsigned long interval = now - lastWheelPollTime;
  if (interval > 10000) // Debounce 10ms (Max 100 RPS / 6000 RPM approx)
  {
    lastWheelInterval = interval;
    lastWheelPollTime = now;
    newSpeedSignal = true;
  }
}
