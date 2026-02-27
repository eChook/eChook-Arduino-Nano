/**
 * @file Pinout.h
 * @brief Hardware pin mapping for the eChook Telemetry Board.
 */

#include <Arduino.h>

// ANALOG INPUT PINS
/** @brief Analog input pin for measuring total battery voltage. */
const int VBATT_IN_PIN = A0;
/** @brief Analog input pin for measuring battery current draw. */
const int AMPS_IN_PIN = A2;
/** @brief Analog input pin for reading the throttle pedal/button. */
const int THROTTLE_IN_PIN = A3;
/** @brief Analog input pin for temperature sensor 1. Reassignable if I2C jumper is used. */
extern int TEMP1_IN_PIN;
/** @brief Analog input pin for temperature sensor 2. Reassignable if I2C jumper is used. */
extern int TEMP2_IN_PIN;
/** @brief Analog input pin for measuring lower battery bank voltage. */
const int VBATT1_IN_PIN = A7;

// DIGITAL INPUT PINS
/** @brief Digital input pin for the brake sensor/button. */
const int BRAKE_IN_PIN = 7;
/** @brief Digital input pin for the launch mode button. */
const int LAUNCH_BTN_IN_PIN = 8;
/** @brief Digital input pin for the user interface cycle button. */
const int CYCLE_BTN_IN_PIN = 12;

// DIGITAL INTERRUPT PINS
/** @brief interrupt-capable pin for motor speed pulses. */
const int MOTOR_RPM_PIN = 2;
/** @brief interrupt-capable pin for wheel speed pulses. */
const int WHEEL_RPM_PIN = 3;

// DIGITAL AND PWM OUTPUT PINS
/** @brief PWM output pin for driving the motor (if enabled). */
const int MOTOR_OUT_PIN = 5;
/** @brief Pin used for HC-05 Bluetooth module configuration/enabling. */
const int BT_EN_PIN = 4;
