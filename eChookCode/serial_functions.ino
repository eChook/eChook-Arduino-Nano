/**
 * @file serial_functions.ino
 * @brief Handles complex serial communication for configuration and
 * calibration.
 *
 * This file contains functions for processing serial commands from the
 * configuration website, including getting and setting calibration data.
 */

/**
 * @brief Checks for incoming serial commands and executes the appropriate
 * action. Handles the configuration menu logic over USB Serial.
 */
void SerialCheck() {

  if (Serial.available()) {
    char temp = Serial.read();
    uint8_t inMenu = 1;
    long menuEnterTime = millis();
    while (inMenu) {
      if (temp == 'g') {
        inConfig = 1; // This triggers the nano every to start sending data out
                      // over USB Serial.
        // Get / Request Calibration Data (Get/Set)
        // Action - send calibration data.
        while (!Serial.available()) {
          inMenu = menuTimeout(menuEnterTime);
        }
        temp = Serial.read();
        if (temp == 'f') { // get float cal
          sendFloatCal();
        } else if (temp == 'b') { // get binary cal
          sendBinaryCal();
        } else if (temp == 'n') { // get BT name
          sendBTName();
        } else if (temp == 'v') { // get code version
          sendVersion();
        } else if (temp == 'd') { // toggle Debug Mode
          DEBUG_MODE = !DEBUG_MODE;
        }
      } else if (temp == 's') {
        // Set Calibration data

        while (!Serial.available()) {
          inMenu = menuTimeout(menuEnterTime);
        }
        temp = Serial.read();
        if (temp == 'n') {
          receiveBTName();
        } else if (temp == 'f') {
          receiveFloatCal();
        } else if (temp == 'b') {
          receiveBinaryCal();
        }
      } else if (temp == 'C') { // Clear EEPROM Data
        clearVerificationByte();
        resetArduino();
      } else {
        // Invalid data
        inMenu = 0;
      }
    }
  }
}

/**
 * @brief Checks if a menu operation has timed out.
 * @param time The start time of the operation.
 * @return 1 if timed out, 0 otherwise.
 */
uint8_t menuTimeout(long time) {
  return millis() - time > 1000; // 1sec timeout
}

/**
 * @brief Sends the float calibration array over Serial.
 * Packet format: [f<data>]
 */
void sendFloatCal() {
  const uint8_t sendArrayLength = 80 + 3;
  char sendArr[sendArrayLength] = {};
  sendArr[0] = '[';                   // Packet Start Indicator
  sendArr[1] = 'f';                   // Float Array Identifier
  sendArr[sendArrayLength - 1] = ']'; // Packet End Indicator

  for (uint8_t i = 0; i < sendArrayLength - 3; i++) {
    sendArr[i + 2] = getFloatByte(i);
  }

  Serial.write(sendArr, sendArrayLength);
  Serial.flush();
}

/**
 * @brief Sends the binary calibration bytes over Serial.
 * Packet format: [b<data>]
 */
void sendBinaryCal() {
  const uint8_t sendArrayLength = 7 + 3;
  char sendArr[sendArrayLength] = {};
  sendArr[0] = '[';                   // Packet Start Indicator
  sendArr[1] = 'b';                   // Float Array Identifier
  sendArr[sendArrayLength - 1] = ']'; // Packet End Indicator

  for (uint8_t i = 0; i < sendArrayLength - 3; i++) {
    sendArr[i + 2] = getBinaryCalByte(CAL_A + i);
  }

  Serial.write(sendArr, sendArrayLength);
  Serial.flush();
}

/**
 * @brief Sends the Bluetooth name over Serial.
 * Packet format: [n<data>]
 */
void sendBTName() {
  const uint8_t sendArrayLength = 30 + 3;
  char sendArr[sendArrayLength] = {};
  sendArr[0] = '[';                   // Packet Start Indicator
  sendArr[1] = 'n';                   // Float Array Identifier
  sendArr[sendArrayLength - 1] = ']'; // Packet End Indicator

  for (uint8_t i = 0; i < sendArrayLength - 3; i++) {
    sendArr[i + 2] = getNameByte(i);
  }

  Serial.write(sendArr, sendArrayLength);
  Serial.flush();
}

/**
 * @brief Sends the current firmware version over Serial.
 * Packet format: [v<version>]
 */
void sendVersion() {
  const uint8_t sendArrayLength = 5 + 3;
  char sendArr[sendArrayLength] = {};
  char version[5] = {'0'};

  dtostrf(CODE_VERSION, 5, 2, version);
  // Add padding 0 to start of array if needed
  if (CODE_VERSION < 10) {
    version[0] = '0';
  }

  for (uint8_t i = 0; i < sendArrayLength - 3; i++) {
    sendArr[i + 2] = version[i];
  }

  sendArr[0] = '[';                   // Packet Start Indicator
  sendArr[1] = 'v';                   // Version Array Identifier
  sendArr[sendArrayLength - 1] = ']'; // Packet End Indicator
  Serial.write(sendArr, sendArrayLength);
  Serial.flush();
}

/**
 * @brief Receives a new Bluetooth name from Serial and saves it to EEPROM.
 */
void receiveBTName() {
  unsigned long entryTime = millis();
  uint8_t receivedCount = 0;
  char inBuff[30] = {};
  uint8_t timeout = 0;

  while (receivedCount < 30) {
    timeout = menuTimeout(entryTime);

    if (Serial.available()) {
      inBuff[receivedCount] = Serial.read();
      receivedCount++;
    }
  }

  if (!timeout) {
    String temp = "";
    for (uint8_t i = 0; i < 30; i++) {
      if (inBuff[i] != 0xff) {
        temp += inBuff[i];
      }
    }
    temp.trim();
    CAL_BT_NAME = temp;

    writeBTName();           // Writes the new name to EEPROM
    loadEepromCalibration(); // Reloads dynamic calibration from EEPROM
    sendBTName();            // Sends out eeprom contents
  }
}

/**
 * @brief Receives new float calibration data from Serial and saves it to
 * EEPROM.
 */
void receiveFloatCal() {
  unsigned long entryTime = millis();
  uint8_t receivedCount = 0;
  char inBuff[80] = {};
  uint8_t timeout = 0;

  while (receivedCount < 80) {
    timeout = menuTimeout(entryTime);

    if (Serial.available()) {
      inBuff[receivedCount] = Serial.read();
      receivedCount++;
    }
  }

  if (!timeout) {
    EEPROM.put(FLOAT_ARRAY_START, inBuff);

    loadEepromCalibration(); // Reloads dynamic calibration from EEPROM
    sendFloatCal();          // Sends out eeprom contents
  }
}

/**
 * @brief Receives new binary calibration data from Serial and saves it to
 * EEPROM.
 */
void receiveBinaryCal() {
  unsigned long entryTime = millis();
  uint8_t receivedCount = 0;
  char inBuff[4] = {};
  uint8_t timeout = 0;

  while (receivedCount < 4) {
    timeout = menuTimeout(entryTime);

    if (Serial.available()) {
      inBuff[receivedCount] = Serial.read();
      receivedCount++;
    }
  }

  if (!timeout) {
    EEPROM.put(CAL_A, inBuff);

    loadEepromCalibration(); // Reloads dynamic calibration from EEPROM
    sendBinaryCal();         // Sends out eeprom contentss
  }
}

/**
 * @brief Resets the Arduino by jumping to the bootloader address.
 * @note This is a hard reset using assembly.
 */
void resetArduino() { asm volatile("jmp 0x7800"); }
