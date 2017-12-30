Digital thermometer and hygrometer with integrated data logger
==============================================================

Components:
-----------
* Arduino Nano (or compatible)
* DHT22 sensor
* DCF77 module
* OLED display, SSD1306-compatible, I2C, 128x64

Hardware setup:
---------------
* Arduino D2:           DHT22 "data" pin
* Arduino D3:           DHT22 "VCC" pin
* Arduino D4:           DCF77 module "PON" pin (power mode)
* Arduino D5:           DCF77 module "TCO" pin (time signal)
* Arduino A4 (I2C SDA): SDA pin of display
* Arduino A5 (I2C SCL): SCL pin of display

The DHT22 sensor is supplied from an Arduino output pin. The maximum current
drawn by the sensor is around 1.5 mA, so the pin can easily supply the chip.
This way the sensor can be switched off to save power and avoid self-heating
of the sensor. The sensor is enabled 1s before a measurement is taken and
switched off after the measurement.

The DCF77 module was harvested from an old alarm clock. It operates on any
voltage between 3V and 5V. The "PON" pin controlls the module's power mode:
pulling the signal LOW will enable the module. Pulling it HIGH will disable
the module and save power. With PON set to low, the module will provide the
DCF77 time signal on its "TCO" pin. The time signal is transmitted as a bit
stream with one bit per second and a length of 58 or 59 bits. A "0" bit is
transmitted as a pulse with a width of around 100 ms, and a "1" is a pulse with
a width of around 200 ms.

As the DHT22 sensor is quite timing-critical and the Adafruit library turns
off interrupts during a transmission, polling the sensor and interfacing to
the display might disturb the timing measurements of the TCO signal. Therefore,
these tasks are only started right after a bit pulse has been detected, which
gives them up to 780 ms of free CPU time until the next bit starts.


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
 
    now     print current time
        Print the current time and date.

