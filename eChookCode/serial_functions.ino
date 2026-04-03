/**
 * @file serial_functions.ino
 * @brief Handles complex serial communication for configuration and
 * calibration.
 *
 * This file contains functions for processing serial commands from the
 * configuration website, including getting and setting calibration data.
 */

// V2.5+ Protocol Constants
#define PKT_VERSION_REQ 0x01
#define PKT_VERSION_RESP 0x81
#define PKT_FLOAT_REQ 0x02
#define PKT_FLOAT_RESP 0x82
#define PKT_BIN_REQ 0x03
#define PKT_BIN_RESP 0x83
#define PKT_NAME_REQ 0x04
#define PKT_NAME_RESP 0x84

#define PKT_FLOAT_SET 0x42
#define PKT_BIN_SET 0x43
#define PKT_NAME_SET 0x44
#define PKT_CLEAR_EEPROM 0x45

#define PKT_ACK 0xFC
#define PKT_NACK 0xFD

/**
 * @brief Checks for incoming serial commands and executes the appropriate action. Handles the configuration menu logic over USB Serial.
 */
void SerialCheck() {

  if (Serial.available()) {
    char temp = Serial.read();

    // V2.5+ Protocol Sync Byte
    if ((uint8_t)temp == 0xAA) {
      receiveV25Packet();
      return;
    }

    uint8_t inMenu = 1;
    long menuEnterTime = millis();
    while (inMenu) {
      if (temp == 'g') {
        inConfig = 1; // This triggers the nano every to start sending data out over USB Serial.
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
    uint8_t outIdx = 0;
    for (uint8_t i = 0; i < 30 && outIdx < 31; i++) {
      if (inBuff[i] != (char)0xff && inBuff[i] != '\0') {
        CAL_BT_NAME[outIdx++] = inBuff[i];
      }
    }
    CAL_BT_NAME[outIdx] = '\0';

    // Trim trailing whitespace manually
    while (outIdx > 0 && (CAL_BT_NAME[outIdx - 1] == ' ' || CAL_BT_NAME[outIdx - 1] == '\r' || CAL_BT_NAME[outIdx - 1] == '\n')) {
      outIdx--;
      CAL_BT_NAME[outIdx] = '\0';
    }

    // Trim leading whitespace manually
    uint8_t startIdx = 0;
    while (CAL_BT_NAME[startIdx] == ' ' || CAL_BT_NAME[startIdx] == '\r' || CAL_BT_NAME[startIdx] == '\n') {
      startIdx++;
    }
    if (startIdx > 0) {
      uint8_t i = 0;
      while (CAL_BT_NAME[startIdx + i] != '\0') {
        CAL_BT_NAME[i] = CAL_BT_NAME[startIdx + i];
        i++;
      }
      CAL_BT_NAME[i] = '\0';
    }

    writeBTName();           // Writes the new name to EEPROM
    loadEepromCalibration(); // Reloads dynamic calibration from EEPROM
    sendBTName();            // Sends out eeprom contents
  }
}

/**
/**
 * @brief Receives new float calibration data from Serial and saves it to EEPROM.
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
/**
 * @brief Receives new binary calibration data from Serial and saves it to EEPROM.
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

// --- V2.5+ Protocol Implementation ---

void sendV25Packet(uint8_t type, const uint8_t *data, uint8_t len) {
  uint8_t checksum = type ^ len;
  Serial.write(0xAA);
  Serial.write(type);
  Serial.write(len);
  for (uint8_t i = 0; i < len; i++) {
    Serial.write(data[i]);
    checksum ^= data[i];
  }
  Serial.write(checksum);
  Serial.write(0x55); // End byte
  Serial.flush();
}

void sendAck(uint8_t type) {
  uint8_t data[1] = {type};
  sendV25Packet(PKT_ACK, data, 1);
}

void sendNack(uint8_t type) {
  uint8_t data[1] = {type};
  sendV25Packet(PKT_NACK, data, 1);
}

void sendV25Version() {
  uint8_t data[6];
  char version[5] = {'0'};
  dtostrf(CODE_VERSION, 5, 2, version);
  if (CODE_VERSION < 10) {
    version[0] = '0';
  }
  for (int i = 0; i < 5; i++) {
    data[i] = version[i];
  }

#if defined(__AVR_ATmega4809__)
  data[5] = 1; // Nano Every
#else
  data[5] = 0; // Standard Nano
#endif

  sendV25Packet(PKT_VERSION_RESP, data, 6);
}

void sendV25FloatCal() {
  uint8_t data[80];
  for (uint8_t i = 0; i < 80; i++) {
    data[i] = getFloatByte(i);
  }
  sendV25Packet(PKT_FLOAT_RESP, data, 80);
}

void sendV25BinaryCal() {
  uint8_t data[4];
  for (uint8_t i = 0; i < 4; i++) {
    data[i] = getBinaryCalByte(CAL_A + i);
  }
  sendV25Packet(PKT_BIN_RESP, data, 4);
}

void sendV25BTName() {
  uint8_t data[30];
  for (uint8_t i = 0; i < 30; i++) {
    data[i] = getNameByte(i);
  }
  sendV25Packet(PKT_NAME_RESP, data, 30);
}

void receiveV25Packet() {
  unsigned long entryTime = millis();
  uint8_t type = 0;
  uint8_t len = 0;
  uint8_t checksum = 0;
  uint8_t calculatedChecksum = 0;

  // Read Type
  while (!Serial.available()) {
    if (menuTimeout(entryTime))
      return;
  }
  type = Serial.read();
  calculatedChecksum ^= type;

  // Read Length
  while (!Serial.available()) {
    if (menuTimeout(entryTime))
      return;
  }
  len = Serial.read();
  calculatedChecksum ^= len;

  // Read Data
  uint8_t data[85];
  for (uint8_t i = 0; i < len; i++) {
    while (!Serial.available()) {
      if (menuTimeout(entryTime))
        return;
    }
    data[i] = Serial.read();
    calculatedChecksum ^= data[i];
    if (i >= sizeof(data) - 1)
      break; // Safety
  }

  // Read Checksum
  while (!Serial.available()) {
    if (menuTimeout(entryTime))
      return;
  }
  checksum = Serial.read();

  if (checksum != calculatedChecksum) {
    sendNack(type);
    return;
  }

  // Process Packet
  switch (type) {
  case PKT_VERSION_REQ:
    inConfig = 1;
    sendV25Version();
    break;
  case PKT_FLOAT_REQ:
    sendV25FloatCal();
    break;
  case PKT_BIN_REQ:
    sendV25BinaryCal();
    break;
  case PKT_NAME_REQ:
    sendV25BTName();
    break;
  case PKT_FLOAT_SET:
    if (len == 80) {
      EEPROM.put(FLOAT_ARRAY_START, data);
      loadEepromCalibration();
      sendAck(type);
    } else {
      sendNack(type);
    }
    break;
  case PKT_BIN_SET:
    if (len == 4) {
      EEPROM.put(CAL_A, data);
      loadEepromCalibration();
      sendAck(type);
    } else {
      sendNack(type);
    }
    break;
  case PKT_NAME_SET: {
    uint8_t outIdx = 0;
    for (uint8_t i = 0; i < len && outIdx < 31; i++) {
      if (data[i] != 0xff && data[i] != '\0') {
        CAL_BT_NAME[outIdx++] = data[i];
      }
    }
    CAL_BT_NAME[outIdx] = '\0';
    writeBTName();
    loadEepromCalibration();
    sendAck(type);
  } break;
  case PKT_CLEAR_EEPROM:
    clearVerificationByte();
    sendAck(type);
    resetArduino();
    break;
  }
}
