/*

  Thermometer.ino

  Thermometer/hygrometer and data logger  

*/

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT22     // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);

#define SERIAL_BAUDRATE   19200

#define DEGREE '\xb0'
float fCurrentTemp = 99.9f;
float fCurrentHumidity = 99.9f;


U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ A5, /* data=*/ A4, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display


void setup(void) {
  u8g2.begin();
  Serial.begin(SERIAL_BAUDRATE);
  dht.begin();
  updateDisplay();
}

void updateSensors()
{
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (!isnan(event.temperature))
  {
    fCurrentTemp = event.temperature;
  }
  dht.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity)) {
    fCurrentHumidity = event.relative_humidity;
  }
}

void updateDisplay()
{
  u8g2.firstPage();
  do {
    char buf[10];
    String s = String(fCurrentTemp, 1);
    s += ' ';
    s += DEGREE;
    s += 'C';
    s.toCharArray(buf, sizeof(buf));

    u8g2.setFont(u8g2_font_fub20_tf);
    u8g2.drawStr(0,24,buf);

    s = String(fCurrentHumidity, 0);
    s += '%';
    s += " rH";
    s.toCharArray(buf, sizeof(buf));

    u8g2.drawStr(0,54,buf);
  } while ( u8g2.nextPage() );
}

void loop(void)
{
  int in = Serial.read();
  updateSensors();
  updateDisplay();
  delay(1000);
}

