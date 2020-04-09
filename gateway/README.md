# Simple IoT Button
Developed as part of Hack Quarantine 2020.

## Functionality overview
A simple and straightforward device that detects a button press, provides
a visual feedback to the user, and performs a call to a backend server.

Will have an option to connect through a LoRaWAN network in case there is no
Wi-Fi coverage at a particular house.

## Hardware and software used
A Pycom LoPy4 module as a System-in-Package with an external 868 MHz antenna.
Programmed using MicroPython.

## TODO
1. LoRa connectivity option
2. Mobile phone network connectivity option
3. End-to-end encryption
4. Error catching and recovery
5. Intelligent choice of connectivity option