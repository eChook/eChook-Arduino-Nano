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
#if defined(__AVR_ATmega4809__)
  // Read exact tick count from Timer B 0 (Hardware captured on rising edge via EVSYS)
  // Hybrid Hrdware and millis() approach - The ATmega4809 TCB has two hard limitations:
  // 1. It shares the TCA0 prescaler clock (250kHz, 4us/tick), wrapping every 262.14ms.
  // 2. The TCB overflow and capture interrupts share the same status flag, making pure hardware
  //    overflow counting messy.
  // Solution: We use TCB for perfect cycle-accurate microsecond stamping of the pulse, and
  // software micros() to track the massive 262ms wrap-around epochs.
  uint16_t exactHardwareTicks = TCB0.CCMP;

  // Calculate the time elapsed in micros according to the software clock
  unsigned long softwareInterval = now - lastMotorPollTime;

  // Combine software interval (for magnitude > 262ms) with exact hardware ticks
  // We divide software interval by the 262.14ms wraparound period to find how many times TCB wrapped
  unsigned long wrapArounds = softwareInterval / 262144UL;
  unsigned long exactIntervalLength = (wrapArounds * 262144UL) + (exactHardwareTicks * 4UL);

  // If the software interval was just over a wraparound boundary but the hardware tick is very high,
  // it means the hardware timer hadn't wrapped yet when the event occurred.
  if ((softwareInterval % 262144UL) < 131072UL && exactHardwareTicks > 32768) {
    if (wrapArounds > 0)
      exactIntervalLength -= 262144UL;
  }
  // Conversely, if software was just under the boundary but hardware is very low, it had already wrapped.
  else if ((softwareInterval % 262144UL) > 131072UL && exactHardwareTicks < 32768) {
    exactIntervalLength += 262144UL;
  }

  unsigned long interval = exactIntervalLength;
#else
  unsigned long interval = now - lastMotorPollTime;
#endif

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
#if defined(__AVR_ATmega4809__)
  // Read exact tick count from Timer B 1 (Hardware captured on rising edge via EVSYS)
  // See previous ISR for implementation resaoning
  uint16_t exactHardwareTicks = TCB1.CCMP;

  // Calculate the time elapsed in micros according to the software clock
  unsigned long softwareInterval = now - lastWheelPollTime;

  // Combine software interval (for magnitude > 262ms) with exact hardware ticks
  // We divide software interval by the 262.14ms wraparound period to find how many times TCB wrapped
  unsigned long wrapArounds = softwareInterval / 262144UL;
  unsigned long exactIntervalLength = (wrapArounds * 262144UL) + (exactHardwareTicks * 4UL);

  // Boundary correction: if software is just over a wrap, but hardware is very high, hardware hadn't wrapped.
  if ((softwareInterval % 262144UL) < 131072UL && exactHardwareTicks > 32768) {
    if (wrapArounds > 0)
      exactIntervalLength -= 262144UL;
  }
  // Boundary correction: if software was just under a wrap, but hardware is very low, hardware had already wrapped.
  else if ((softwareInterval % 262144UL) > 131072UL && exactHardwareTicks < 32768) {
    exactIntervalLength += 262144UL;
  }

  unsigned long interval = exactIntervalLength;
#else
  unsigned long interval = now - lastWheelPollTime;
#endif

  if (interval > 10000) // Debounce 10ms (Max 100 RPS / 6000 RPM approx)
  {
    lastWheelInterval = interval;
    lastWheelPollTime = now;
    newSpeedSignal = true;
  }
}
