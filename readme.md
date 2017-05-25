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
* Arduino A4 (I2C SDA): SDA pin of display
* Arduino A5 (I2C SCL): SCL pin of display

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
The following commands can be given over the serial interface (57600 baud, 8N1):

rs   - read sensor
       Reads the current temperature and relative humidity

