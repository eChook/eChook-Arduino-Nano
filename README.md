# eChook Nano
The eChook is a Greenpower specific data logging project with all the inputs required for a Greenpower race car. While many of the data logging options on the market are 'black box' solutions, the eChook uses simple circuitry and simple, heavily commented code so that teams can understand how it works, and tweak it as they like!
## Arduino Code
This repository contains the Arduino code for the eChook Nano board.  
As of release 2.0, this code now targets the **Arduino Nano Every** development board. It should still be backwards compatible with an Arduino Nano 328p or 168p, but it is no longer actively tested on these legacy boards.

## Documentation
The main documentation for the eChook is hosted at [docs.echook.uk](https://docs.echook.uk). 
## Usage
Please download the latest release of the code from github. (The 'Release' section is on the right hand side of the main page) Cloning the 'master' branch may result in you downloading code that has not been fully tested. 

To compile this code you will require the [Bounce2](https://github.com/thomasfredericks/Bounce2) library.  


## Changelog
For recent changes, please see the Release History.  

  
Historic release changelog:
- 10/03/22: Tidied Button reading functions to better utilise the Bounce2 Library
- 10/03/22: Added compatibility with the Arduino Nano Every board, which is the official replacement for the venerable Arduino Nano 328p
- 12/02/19: Fixed bug that made speed read at 1/3 of actual (Thanks Apex Racing!)

## Modifications
Some people have already modified the eChook to make it work exactly how they want! If you have any modifications of your own you want to share get in touch and we'll add them here. Pull requests are welcome, but they must not break the original, out the box functionality of the code and will be merged at eChook's discression.

#### Notable Forks:
Derby Grammar have implemented an LCD driver information display, and replaced the thermistors with pre-calibrated temperature probes.
See their code here: https://github.com/DerbyGrammar/eChookArduinoNano
