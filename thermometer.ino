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
#include <Cmd.h>

#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define DHTPOWERPIN       3         // Pin which supplies the DHT sensor with power.
#define DHTTYPE           DHT22     // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);

#define SERIAL_BAUDRATE   57600

#define DEGREE '\xb0'
float fCurrentTemp = 99.9f;
float fCurrentHumidity = 99.9f;


U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ A5, /* data=*/ A4, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
#define DISPLAY_RES_X   128
#define DISPLAY_RES_Y   64

// A type to hold a display coordinate (an X or Y value)
#if (DISPLAY_RES_X <= 255) && (DISPLAY_RES_Y <= 255)
typedef unsigned char COORD;
#else
typedef unsigned int  COORD;
#endif

#define DISPLAY_ALTERNATING_SECONDS   5
//#define WITH_TEST_COORDS

// global display mode
char displayMode = 0;

// ----------------------------------------------------------------------------
// Initial setup
// ----------------------------------------------------------------------------
void setup(void)
{
  // initialize the display and show a welcome message
  u8g2.begin();
  u8g2.setFont(u8g2_font_fub20_tf);
  u8g2.firstPage();
  do
  {
    u8g2.drawStr(0,24,"Hi!");
  } while ( u8g2.nextPage() );

  // power up and initialize DHT22 sensor
  pinMode(DHTPOWERPIN, OUTPUT);
  digitalWrite(DHTPOWERPIN, HIGH);
  delay(1000);
  dht.begin();

  // initialize serial command interface
  Serial.begin(SERIAL_BAUDRATE);
  cmdInit(&Serial);
  cmdAdd("rs", readSensor);
  #ifdef WITH_TEST_COORDS
  cmdAdd("tc", testCoords);
  #endif
  cmdAdd("dm", setDisplayMode);
  Serial.println("Rdy");
}

// ----------------------------------------------------------------------------
// The cyclic main loop
// ----------------------------------------------------------------------------
void loop(void)
{
  unsigned long currentTime = millis();

  // handle the connected sensors
  updateSensors(currentTime);

  // handle the display
  updateDisplay(currentTime);

  // handle serial interface
  cmdPoll();
}

// ----------------------------------------------------------------------------
// Fetch the current sensor values and update the global variables
// fCurrentTemp and fCurrentHumidity.
// ----------------------------------------------------------------------------
void updateSensors(unsigned long currentTime)
{
  enum {POWERING_UP, MEASURING, SLEEPING};
  static unsigned long lastStateChange = 0;
  static char state = MEASURING;
  sensors_event_t event;

  // sensor state machine:
  // - power up the sensor for 1000 ms to stabilize
  // - take a measurement
  // - power down the sensor for 9000 ms
  switch (state)
  {
    case MEASURING:
      dht.temperature().getEvent(&event);
      if (!isnan(event.temperature))
      {
        fCurrentTemp = event.temperature;
      }
      dht.humidity().getEvent(&event);
      if (!isnan(event.relative_humidity)) {
        fCurrentHumidity = event.relative_humidity;
      }
      // switch off the sensor
      digitalWrite(DHTPOWERPIN, LOW);
      state = SLEEPING;
      lastStateChange = currentTime;
      break;

    case SLEEPING:
      if ((currentTime - lastStateChange) >= 9000)
      {
        lastStateChange = currentTime;
        state = POWERING_UP;
      }
      break;

    case POWERING_UP:
    default:
      // enable sensor
      digitalWrite(DHTPOWERPIN, HIGH);
      if ((currentTime - lastStateChange) >= 1000)
      {
        lastStateChange = currentTime;
        state = MEASURING;
      }
      break;
  }
}

// ----------------------------------------------------------------------------
// Command to change the display mode
// ----------------------------------------------------------------------------
void setDisplayMode(int argc, char **args)
{
  if (argc != 2)
  {
    Stream *s = cmdGetStream();
    s->println("*** dm: needs one argument (0: values, 1: bargraph, 2 auto)");
  }
  else
  {
    char val = cmdStr2Num(args[1], 10);
    if ((val >= 0) && (val <= 2))
    {
      displayMode = val;
    }
  }
}

// ----------------------------------------------------------------------------
// Update the display
// ----------------------------------------------------------------------------
void updateDisplay(unsigned long currentTime)
{
  static unsigned long lastUpdate = 0UL;
  static char currentDisplayMode = 0;
  static char alternatingCounter = 0;

  if ((currentTime - lastUpdate) >= 1000)
  {
    // one second has passed, update the display

    // determine the effective display mode depending on the globally selected display mode
    if (displayMode == 2)
    {
      // alternating display
      if (alternatingCounter <= 0)
      {
        // toggle display mode
        currentDisplayMode ^= 1;
        alternatingCounter = DISPLAY_ALTERNATING_SECONDS;
      }
      else
      {
        alternatingCounter--;
      }
    }
    else
    {
      // in mode 0 and 1, just use the globally selected display mode
      currentDisplayMode = displayMode;
      alternatingCounter = 0;
    }

    // finally draw the display contents
    if (currentDisplayMode == 0)
    {
      displayValues();
    }
    else
    {
      displayBargraph();
    }

    // update the one second timer
    lastUpdate = currentTime;
  }
}

// ----------------------------------------------------------------------------
// Show current measured values on the display
// ----------------------------------------------------------------------------
void displayValues()
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

// ----------------------------------------------------------------------------
// Display a bargraph with the temperature history
// ----------------------------------------------------------------------------
void displayBargraph()
{
  u8g2.firstPage();
  do
  {
    drawAxes();
  } while ( u8g2.nextPage() );
}

// ----------------------------------------------------------------------------
// Print the current sensor values on the serial interface
// ----------------------------------------------------------------------------
void readSensor(int argc, char **args)
{
  Stream *s = cmdGetStream();
  bool printTemp = false;
  bool printHumidity = false;

  if (argc == 1)
  {
    printTemp = true;
    printHumidity = true;
  }
  else if (argc == 2)
  {
    char value = cmdStr2Num(args[1], 10);
    if (value == 0)
    {
      printTemp = true;
    }
    else if (value == 1)
    {
      printHumidity = true;
    }
    else
    {
      s->println("*** rs: 0: temp  1: humidity");
      return;
    }
  }
  else
  {
    s->println("*** rs: expecting 0 or 1 argument(s)");
    return;
  }

  if (printTemp)
  {
    s->print(String(fCurrentTemp, 1));
    s->println("°C");
  }
  if (printHumidity)
  {
    s->print(String(fCurrentHumidity, 0));
    s->println("%");
  }
}

// ------------ Bargraph display -------------------------
float fMinY = 16.0f;
float fMaxY = 30.0f;
float fMinX = 0.0f;
float fMaxX = 100.0f;
COORD marginBotPx = 10;
COORD marginTopPx = 1;
COORD marginLeftPx = 15;
COORD marginRightPx = 1;

// ----------------------------------------------------------------------------
/// Transform a floating point value to absolute display coordinates
// ----------------------------------------------------------------------------
bool pointToDisplayCoords(float fPointX, float fPointY, COORD& x, COORD& y)
{
  bool boOnScreen = false;
  if (     (fPointX >= fMinX)
        && (fPointX <= fMaxX)
        && (fPointY >= fMinY)
        && (fPointY <= fMaxY) )
  {
    boOnScreen = true;

    // calculate absolute x coordinate
    COORD pixels = DISPLAY_RES_X - (marginLeftPx + marginRightPx);
    float fRelPoint = (fPointX - fMinX) / (fMaxX - fMinX);
    x = marginLeftPx + (fRelPoint * pixels);

    // calculate absolute y coordinate
    pixels = DISPLAY_RES_Y - (marginTopPx + marginBotPx);
    fRelPoint = ((fPointY - fMinY) / (fMaxY - fMinY));
    y = DISPLAY_RES_Y - (marginBotPx + fRelPoint * pixels);
  }

  return boOnScreen;
}

#ifdef WITH_TEST_COORDS
// ----------------------------------------------------------------------------
// Test command to test the implementation of pointToDisplayCoords
// ----------------------------------------------------------------------------
void testCoords(int argc, char **args)
{
  Stream *s = cmdGetStream();
  COORD x, y;
  if (argc == 3)
  {
    float fX = (float)cmdStr2Num(args[1], 10) / 10.0f;
    float fY = (float)cmdStr2Num(args[2], 10) / 10.0f;
    if (pointToDisplayCoords(fX, fY, x, y))
    {
      s->println(x);
      s->println(y);
    }
    else
    {
      s->println("off screen");
    }
  }
  else
  {
    s->println("*** testCoords: invalid number of arguments (must be 2)");
  }
}
#endif

// ----------------------------------------------------------------------------
// Draw the coordinate system axes and ticks
// ----------------------------------------------------------------------------
void drawAxes()
{
  char buf[8];
  u8g2.setFont(u8g2_font_5x7_tf);

  COORD minx, miny, maxx, maxy;
  pointToDisplayCoords(fMinX, fMinY, minx, miny);
  pointToDisplayCoords(fMaxX, fMaxY, maxx, maxy);
  u8g2.drawLine(minx, miny, maxx, miny);
  u8g2.drawLine(minx, miny, minx, maxy);

  float fTick = floor(fMinY + 1.0f);
  COORD x, y;
  pointToDisplayCoords(fMinX, fTick, x, y);
  u8g2.drawPixel(x-1, y);
  u8g2.drawPixel(x-2, y);
  String s = String(fTick, 0);
  s.toCharArray(buf, sizeof(buf));
  u8g2.drawStr(0, y+3, buf);

  fTick = floor(fMaxY - 1.0f);
  pointToDisplayCoords(fMinX, fTick, x, y);
  u8g2.drawPixel(x-1, y);
  u8g2.drawPixel(x-2, y);
  s = String(fTick, 0);
  s.toCharArray(buf, sizeof(buf));
  u8g2.drawStr(0, y+3, buf);
}

