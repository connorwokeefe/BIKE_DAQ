This is a project to collect motion data from a dirt bike, but it can be used for any other projects that use the following compontents included in the module:
- MPU6050
- ADS1115
- NEO-6M GPS
- Micro SD
- RGB LEDs
- ESP32

The module includes a micro-usb port to program the ESP32 microcontroller.

-The mother board control file is used to upload to an ESP32 module to control daughter boards via bluetooth. This board will need to be connected to a computer. Control will be from inputs to the serial communication window.

-The daughter file is used to upload to each individual DAQ module to read and collect data from its location on the bike. 
