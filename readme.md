Digital thermometer and hygrometer with integrated data logger
==============================================================

Components:
-----------
* Arduino Nano (or compatible)
* DHT22 sensor
* OLED display, SSD1306-compatible, I2C, 128x64

Hardware setup:
---------------
* Arduino D2:           DHT22 "data" pin
* Arduino D3:           DHT22 "VCC" pin
* Arduino A4 (I2C SDA): SDA pin of display
* Arduino A5 (I2C SCL): SCL pin of display

The DHT22 sensor is supplied from an Arduino output pin. The maximum current
drawn by the sensor is around 1.5 mA, so the pin can easily supply the chip.
This way the sensor can be switched off to save power and avoid self-heating
of the sensor. The sensor is enabled 1s before a measurement is taken and
switched off after the measurement.

Required software libraries:
----------------------------
* Install from the integrated Arduino library manager:
  * Adafruit unified sensor library
  * Adafruit DHT unified library
  * u8g2 library

* Download manually:
  * CmdArduino (https://github.com/jan-r/CmdArduino, use develop branch)

Serial interface:
-----------------
The following commands can be given over the serial interface (57600 baud, 8-N-1):


    rs [x]  read sensor
        Print the current temperature and/or relative humidity. If no argument is given,
        both values are printed. If argument "0" is given, only temperature is printed.
        If "1" is given, only humidity is printed.

    dm x    select display mode
        Valid display modes are:
        * 0: show current values
        * 1: show bargraph
        * 2: alternate between values and bargraph
 
