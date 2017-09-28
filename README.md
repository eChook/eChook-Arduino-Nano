# eChook Nano
The eChook is a Greenpower specific data logging project with all the inputs required for a Greenpower race car. While many of the data logging options on the market are 'black box' solutions, the eChook uses simple circuitry and simple, heavily commented code so that teams can understand how it works, and tweak it as they like!
## Arduino Code
This repository contains the Arduino code for the eChook Nano board. It is written for an Arduino Nano 328p. It should also run on an Arduino Nano 168 but will be close to the memory limit.
It is also possible to use an Arduino Micro on the eChook board, however code will need to be modified to use Serial1 for Bluetooth communication instead of Serial.
## Documentation
The main documentation for the eChook is hosted on a [Google Document](https://drive.google.com/open?id=12MMUZwj4w5DUC1TnJMnzvs-AOBMiBQhTdfGyMm6rY0o). 
## Usage
To run this code you will require the [Bounce2](https://github.com/thomasfredericks/Bounce2) library.  
If you are updating to the new code please ensure your `Calibration.h` file is backedup. Enjoy :)
