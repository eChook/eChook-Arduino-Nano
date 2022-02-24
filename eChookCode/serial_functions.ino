// This file handles all serial transactions that aren't simple 'send data' transactions.
// Primarily this is for the new (2022) calibration techniques.

void SerialCheck()
{

    if (Serial.available())
    {
        char temp = Serial.read();
        uint8_t inMenu = 1;
        long menuEnterTime = millis();
        while (inMenu)
        {
            if (temp == 'g')
            {
                // Get / Request Calibration Data (Get/Set)
                // Action - send calibration data.
                while (!Serial.available())
                {
                    inMenu = menuTimeout(menuEnterTime);
                }
                temp = Serial.read();
                if (temp == 'f')
                { // get float cal
                    sendFloatCal();
                }
                else if (temp == 'b')
                { // get binary cal
                    sendBinaryCal();
                }
                else if (temp == 'n')
                { // get BT name
                    sendBTName();
                }
                else if (temp == 'd')
                { // get version info
                    DEBUG_MODE = !DEBUG_MODE;
                }
            }
            else if (temp == 's')
            {
                // Set Calibration data

                while (!Serial.available())
                {
                    inMenu = menuTimeout(menuEnterTime);
                }
                temp = Serial.read();
                if (temp == 'n')
                { 
                    receiveBTName();
                } else if(temp == 'f'){
                    receiveFloatCal();
                }
                else if (temp == 'b')
                {
                    receiveBinaryCal();
                }
            }
            else
            {
                // Invalid data
                inMenu = 0;
            }
        }
    }
}

uint8_t menuTimeout(long time)
{
    return millis() - time > 500; // 5sec timeout
}

// ToDo - add a checksum to calibration transmissions

void sendFloatCal()
{
    const uint8_t sendArrayLength = 80 + 3;
    char sendArr[sendArrayLength] = {};
    sendArr[0] = '[';                   // Packet Start Indicator
    sendArr[1] = 'f';                   // Float Array Identifier
    sendArr[sendArrayLength - 1] = ']'; // Packet End Indicator

    for (uint8_t i = 0; i < sendArrayLength - 3; i++)
    {
        sendArr[i + 2] = getFloatByte(i);
    }

    Serial.write(sendArr, sendArrayLength);
}

void sendBinaryCal()
{
    const uint8_t sendArrayLength = 7 + 3;
    char sendArr[sendArrayLength] = {};
    sendArr[0] = '[';                   // Packet Start Indicator
    sendArr[1] = 'b';                   // Float Array Identifier
    sendArr[sendArrayLength - 1] = ']'; // Packet End Indicator

    for (uint8_t i = 0; i < sendArrayLength - 3; i++)
    {
        sendArr[i + 2] = getBinaryCalByte(CAL_A + i);
    }

    Serial.write(sendArr, sendArrayLength);
}

void sendBTName()
{
    const uint8_t sendArrayLength = 30 + 3;
    char sendArr[sendArrayLength] = {};
    sendArr[0] = '[';                   // Packet Start Indicator
    sendArr[1] = 'n';                   // Float Array Identifier
    sendArr[sendArrayLength - 1] = ']'; // Packet End Indicator

    for (uint8_t i = 0; i < sendArrayLength - 3; i++)
    {
        sendArr[i + 2] = getNameByte(i);
    }

    Serial.write(sendArr, sendArrayLength);
}

void receiveBTName()
{
    unsigned long entryTime = millis();
    uint8_t receivedCount = 0;
    char inBuff[30] = {};
    uint8_t timeout = 0;

    while (receivedCount < 30)
    // while (receivedCount < 30 || !timeout)
    {
        timeout = menuTimeout(entryTime);

        if (Serial.available())
        {
            inBuff[receivedCount] = Serial.read();
            receivedCount++;
        }
    }

    if (!timeout)
    {
        String temp = "";
        for (uint8_t i = 0; i < 30; i++)
        {
            if (inBuff[i] != 0xff)
            {
                temp += inBuff[i];
            }
        }
        CAL_BT_NAME = temp;

        writeBTName();// Writes the new name to EEPROM
        loadEepromCalibration(); // Reloads dynamic calibration from EEPROM
        sendBTName(); // Sends out eeprom contentss
    }
}

void receiveFloatCal()
{
    unsigned long entryTime = millis();
    uint8_t receivedCount = 0;
    char inBuff[80] = {};
    uint8_t timeout = 0;

    while (receivedCount < 80)
    // while (receivedCount < 30 || !timeout)
    {
        timeout = menuTimeout(entryTime);

        if (Serial.available())
        {
            inBuff[receivedCount] = Serial.read();
            receivedCount++;
            // Serial.print(receivedCount);
        }
    }

    if (!timeout)
    {
        EEPROM.put(FLOAT_ARRAY_START, inBuff);

        loadEepromCalibration(); // Reloads dynamic calibration from EEPROM
        sendFloatCal();            // Sends out eeprom contentss
    }
}

void receiveBinaryCal()
{
    unsigned long entryTime = millis();
    uint8_t receivedCount = 0;
    char inBuff[4] = {};
    uint8_t timeout = 0;

    while (receivedCount < 4)
    // while (receivedCount < 30 || !timeout)
    {
        timeout = menuTimeout(entryTime);

        if (Serial.available())
        {
            inBuff[receivedCount] = Serial.read();
            receivedCount++;
            // Serial.print(receivedCount);
        }
    }

    if (!timeout)
    {
        EEPROM.put(CAL_A, inBuff);

        loadEepromCalibration(); // Reloads dynamic calibration from EEPROM
        sendBinaryCal();          // Sends out eeprom contentss
    }
}