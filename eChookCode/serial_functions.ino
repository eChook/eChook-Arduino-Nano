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
                while(!Serial.available()){
                    inMenu = menuTimeout(menuEnterTime);
                }
                temp = Serial.read();
                if(temp == 'f'){ // get float cal
                    sendFloatCal();
                }else if(temp == 'b'){ // get binary cal
                    sendBinaryCal();
                }else if (temp == 'v'){ // get version info

                }else if (temp == 'd'){ // get version info
                    DEBUG_MODE = !DEBUG_MODE;
                }
            }
            else if (temp == 's')
            {
                // Set Calibration data
            }else{
                //Invalid data
                inMenu = 0;
            }
        }
    }
}

uint8_t menuTimeout(long time){
    return millis()-time > 5000; //5sec timeout
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