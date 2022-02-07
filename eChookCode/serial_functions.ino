// This file handles all serial transactions that aren't simple 'send data' transactions.
// Primarily this is for the new (2022) calibration techniques.

void SerialCheck()
{

    if (Serial.available())
    {
        char temp = Serial.read();

        if (temp == 'g')
        {
            // Get / Request Calibration Data (Get/Set)
            // Action - send calibration data.
        }
        else if (temp == 's')
        {
            // Set Calibration data
        }
    }
}

