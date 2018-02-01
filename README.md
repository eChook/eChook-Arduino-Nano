# eChook Nano
The eChook is a Greenpower specific data logging project with all the inputs required for a Greenpower race car. While many of the data logging options on the market are 'black box' solutions, the eChook uses simple circuitry and simple, heavily commented code so that teams can understand how it works, and tweak it as they like!
## Arduino Code
This repository contains the Arduino code for the eChook Nano board. It is written for an Arduino Nano 328p. It should also run on an Arduino Nano 168 but will be close to the memory limit.
It is also possible to use an Arduino Micro on the eChook board, however code will need to be modified to use Serial1 for Bluetooth communication instead of Serial.
## Documentation
The main documentation for the eChook is hosted at [docs.echook.uk](https://docs.echook.uk). 
## Usage
To run this code you will require the [Bounce2](https://github.com/thomasfredericks/Bounce2) library.  
If you are updating to the new code please ensure your `Calibration.h` file is backedup. Enjoy :)

## Modifications
Some people have already modified the eChook to make it work exactly how they want! If you have any modifications of your own you want to share get in touch and we'll add them here.

#### 4 Line LCD Driver information display and Digital Temperature Probes
Derby Grammar have implemented an LCD driver information display, and replaced the thermistors with pre-calibrated temperature probes.
See their code here: https://github.com/DerbyGrammar/eChookArduinoNano
