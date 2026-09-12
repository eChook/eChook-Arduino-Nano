/**
 * @file InterruptRoutines.ino
 * @brief Interrupt Service Routines (ISRs) for pulse counting.
 *
 * These functions handle low-level pulse counting for motor and wheel sensors.
 * best practice is kept by keeping these routines as short as possible.
 */

#if defined(__AVR_ATmega4809__)
// Ticks captured on edges the debounce rejected.
//
// TCB captures and restarts its counter on every active edge, including ones the
// debounce goes on to reject. Without carrying those ticks forward the next accepted
// pulse measures from the bounce rather than from the last accepted edge, so the
// interval always comes back short and the speed always reads high - by up to the
// width of the debounce window as a fraction of the period (around 9% for the motor
// at 3000rpm, 8% for the wheel at 10m/s).
//
// These are only ever touched inside the ISR, so they do not need to be volatile.
static uint16_t motorPendingTicks = 0;
static uint16_t wheelPendingTicks = 0;
#endif

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
  // Any ticks carried over from rejected edges are added back in here. The uint16_t
  // arithmetic wraps, which is exactly the mod-65536 the wrap logic below expects.
  uint16_t exactHardwareTicks = motorPendingTicks + TCB0.CCMP;

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

  bool accepted = interval > motorDebounceUs;

  if (accepted) {
    lastMotorPollTime = now;
    // Accumulate rather than overwrite, so a read that spans several pulses sees all
    // of them instead of only the last one.
    motorAccumUs += interval;
    motorPulseCount++;
  }
#if defined(__AVR_ATmega4809__)
  // On accept the next measurement starts from this edge. On a rejected bounce the
  // hardware counter has already restarted, so keep the ticks and the next accepted
  // pulse still measures from the last accepted edge.
  motorPendingTicks = accepted ? 0 : exactHardwareTicks;
#endif
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
  // Ticks carried over from rejected edges added back in - see motorSpeedISR()
  uint16_t exactHardwareTicks = wheelPendingTicks + TCB1.CCMP;

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

  bool accepted = interval > wheelDebounceUs;

  if (accepted) {
    lastWheelPollTime = now;
    // Accumulate rather than overwrite - see motorSpeedISR()
    wheelAccumUs += interval;
    wheelPulseCount++;
  }
#if defined(__AVR_ATmega4809__)
  // Zero on accept, carry forward on reject - see motorSpeedISR()
  wheelPendingTicks = accepted ? 0 : exactHardwareTicks;
#endif
}
