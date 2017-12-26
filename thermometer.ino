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

// ----------------------------------------------------------------------------
// Hardware setup
// ----------------------------------------------------------------------------
#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define DHTPOWERPIN       3         // Pin which supplies the DHT sensor with power.
#define DCF77POWERPIN     4         // Pin to control power of the DCF77 module (low == on)
#define DCF77SIGNALPIN    5         // Pin which connects to the DCF77 time signal 

#define DISP_DATA         A4        // HW I2C data line to display
#define DISP_CLOCK        A5        // HW I2C clock line to display

#define SERIAL_BAUDRATE   57600


// ----------------------------------------------------------------------------
// DHT22 sensor
// ----------------------------------------------------------------------------
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);

#define DEGREE '\xb0'               // degree symbol
float fCurrentTemp = 99.9f;
float fCurrentHumidity = 99.9f;

// calibration
#define REFERENCE_1     7.6f
#define VALUE_1         7.0f
#define REFERENCE_2     29.1f
#define VALUE_2         30.4f
float calibrationFactor = 1.0f;
float calibrationOffset = 0.0f;


// ----------------------------------------------------------------------------
// DCF77 module
// ----------------------------------------------------------------------------
#define DCF77_MIN_ZERO    70      // minimum pulse width for "zero"
#define DCF77_MAX_ZERO    120     // maximum pulse width for "zero"
#define DCF77_MIN_ONE     170     // minimum pulse width for "one"
#define DCF77_MAX_ONE     220     // maximum pulse width for "one"
#define DCF77_DEBUG               // define to add serial debugging output
int state;
int state_old;
unsigned long last_pulse_start;
unsigned long last_pulse_end;
int cbit = -1;
unsigned long timebits;
unsigned long datebits;
int hour = -1;
int minute = -1;


// ----------------------------------------------------------------------------
// Display
// ----------------------------------------------------------------------------
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, DISP_CLOCK, DISP_DATA, U8X8_PIN_NONE);
#define DISPLAY_RES_X   128
#define DISPLAY_RES_Y   64

// A type to hold a display coordinate (an X or Y value)
#if (DISPLAY_RES_X <= 255) && (DISPLAY_RES_Y <= 255)
typedef unsigned char COORD;
#else
typedef unsigned int  COORD;
#endif

#define DISPLAY_ALTERNATING_SECONDS   5
#define DOTTED_LINE_DELTA             10
//#define WITH_TEST_COORDS

// global display mode
char displayMode = 2;

// temperature history
#define HISTORY_VALUES            32      // must be power-of-two!
#define HISTORY_INTERVAL_MSEC     600000  // 600s == 10 minutes
//#define HISTORY_INTERVAL_MSEC     10000   // 10s for testing

float tempHistory[HISTORY_VALUES];
int   currentHistoryIdx;
int   numHistoryValues;


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

  // initialize the DCF77 module pins (but keep powered off, needs ~1s startup delay)
  pinMode(DCF77POWERPIN, OUTPUT);
  pinMode(DCF77SIGNALPIN, INPUT);
  digitalWrite(DCF77POWERPIN, HIGH);

  // power up and initialize DHT22 sensor
  pinMode(DHTPOWERPIN, OUTPUT);
  digitalWrite(DHTPOWERPIN, HIGH);
  delay(1000);
  calibrate(REFERENCE_1, VALUE_1, REFERENCE_2, VALUE_2);
  dht.begin();
  // take initial measurement and initialize history with first value
  updateSensors(millis());
  addValueToHistory(fCurrentTemp);
  addValueToHistory(fCurrentTemp);

  // power up the DCF77 module
  digitalWrite(DCF77POWERPIN, LOW);
  last_pulse_start = millis();
  last_pulse_end = last_pulse_start;

  // initialize serial command interface
  Serial.begin(SERIAL_BAUDRATE);
  cmdInit(&Serial);
  cmdAdd("rs", readSensor);
  #ifdef WITH_TEST_COORDS
  cmdAdd("tc", testCoords);
  #endif
  cmdAdd("dm", setDisplayMode);
}


// ----------------------------------------------------------------------------
// The cyclic main loop
// ----------------------------------------------------------------------------
void loop(void)
{
  unsigned long currentTime = millis();
  bool runOtherTasks;
  
  // process the DCF77 time signal
  runOtherTasks = dcf77_process(currentTime);

  if (runOtherTasks)
  {
    // handle the connected sensors
    updateSensors(currentTime);
  
    // handle the display
    updateDisplay(currentTime);

    // handle serial interface
    cmdPoll();
  }
}

// ----------------------------------------------------------------------------
// Calculate calibration values calibrationFactor and calibrationOffset from
// two measured values
// ref1/ref2: reference temperatures
// val1/val2: measured values at the given temperature
// ----------------------------------------------------------------------------
void calibrate(float ref1, float val1, float ref2, float val2)
{
  calibrationFactor = (ref2 - ref1) / (val2 - val1);
  calibrationOffset = (val2 * ref1 - val1 * ref2) / (val2 - val1);
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
        fCurrentTemp = event.temperature * calibrationFactor + calibrationOffset;
      }
      dht.humidity().getEvent(&event);
      if (!isnan(event.relative_humidity)) {
        fCurrentHumidity = event.relative_humidity;
      }
      trackHistory(currentTime);
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
// Track the history of the temperature values
// ----------------------------------------------------------------------------
void trackHistory(unsigned long currentTime)
{
  static unsigned long lastHistoryValueTaken = 0UL;

  if ((currentTime - lastHistoryValueTaken) >= HISTORY_INTERVAL_MSEC)
  {
    lastHistoryValueTaken = currentTime;
    addValueToHistory(fCurrentTemp);
  }
}

// ----------------------------------------------------------------------------
// Add temperature value to history
// ----------------------------------------------------------------------------
void addValueToHistory(float value)
{
  // increment index and do a fast modulo calculation to handle overflow
  currentHistoryIdx = (currentHistoryIdx + 1) & (HISTORY_VALUES - 1);
  tempHistory[currentHistoryIdx] = value;
  if (numHistoryValues < HISTORY_VALUES)
  {
    numHistoryValues++;
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
  rescaleYAxis();
  u8g2.firstPage();
  do
  {
    drawAxes();
    drawGraph();
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
float fMaxX = (float)(HISTORY_VALUES-1);
COORD marginBotPx = 10;
COORD marginTopPx = 1;
COORD marginLeftPx = 15;
COORD marginRightPx = 1;


// ----------------------------------------------------------------------------
// Recalculate minimum and maximum Y axis values
// ----------------------------------------------------------------------------
void rescaleYAxis()
{
  float maxval = -1e15;
  float minval = 1e15;

  // find min/max value in history buffer
  for (int i = 0; i < numHistoryValues; i++)
  {
    int index = (currentHistoryIdx - i) & (HISTORY_VALUES - 1);
    if (tempHistory[index] < minval)
    {
      minval = tempHistory[index];
    }
    if (tempHistory[index] > maxval)
    {
      maxval = tempHistory[index];
    }
  }
  fMinY = floor(minval) - 1.0f;
  fMaxY = floor(maxval + 1.0f) + 1.0f;
  
  // make sure the difference can be divided by 2
  if ((int)(fMaxY - fMinY) & 1)
  {
    fMaxY += 1.0f;
  }
}

// ----------------------------------------------------------------------------
// Transform a floating point value to absolute display coordinates
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
  drawDottedHLine(fTick, DOTTED_LINE_DELTA);

  fTick = floor(fMaxY - 1.0f);
  pointToDisplayCoords(fMinX, fTick, x, y);
  u8g2.drawPixel(x-1, y);
  u8g2.drawPixel(x-2, y);
  s = String(fTick, 0);
  s.toCharArray(buf, sizeof(buf));
  u8g2.drawStr(0, y+3, buf);
  drawDottedHLine(fTick, DOTTED_LINE_DELTA);

  fTick = floor((fMinY + fMaxY) / 2.0);
  pointToDisplayCoords(fMinX, fTick, x, y);
  u8g2.drawPixel(x-1, y);
  u8g2.drawPixel(x-2, y);
  drawDottedHLine(fTick, DOTTED_LINE_DELTA);
}

// ----------------------------------------------------------------------------
// Draw a dotted horizontal auxillary line (with delta pixels between two dots)
// ----------------------------------------------------------------------------
void drawDottedHLine(float y, COORD delta)
{
  COORD xstart, ycoord, xend;
  pointToDisplayCoords(fMinX, y, xstart, ycoord);
  pointToDisplayCoords(fMaxX, y, xend, ycoord);

  while (xstart <= xend)
  {
    u8g2.drawPixel(xstart, ycoord);
    xstart += delta;
  }
}

// ----------------------------------------------------------------------------
// Draw graph of the history values
// ----------------------------------------------------------------------------
void drawGraph()
{
  for (int i = 1; i < numHistoryValues; i++)
  {
    int index_left = (currentHistoryIdx - i) & (HISTORY_VALUES - 1);
    int index_right  = (currentHistoryIdx + 1 - i) & (HISTORY_VALUES - 1);
    float pos_left = (float)(HISTORY_VALUES - 1) - i;
    float pos_right = (float)(HISTORY_VALUES - i);
    COORD leftx, lefty;
    COORD rightx, righty;
    pointToDisplayCoords(pos_left, tempHistory[index_left], leftx, lefty);
    pointToDisplayCoords(pos_right, tempHistory[index_right], rightx, righty);
    u8g2.drawLine(leftx, lefty, rightx, righty);
  }
}

// ----------------------------------------------------------------------------
// Process the DCF77 signal.
// Return true if a high-to-low-transition was detected.
// ----------------------------------------------------------------------------
#define DCF77_DEBOUNCE_CYCLES   2
bool dcf77_process(unsigned long currentTime)
{
  static unsigned long last_update = 0;
  static int debounce_count = 0;
  bool high2low = false;

  // only process every 2 ms to get some debouncing and glitch immunity
  if (currentTime - last_update > 2)
  {
    int current_pin_value = digitalRead(DCF77SIGNALPIN);
    last_update = currentTime;

    // debounce the pin
    if (current_pin_value == HIGH)
    {
      if (debounce_count < DCF77_DEBOUNCE_CYCLES)
      {
        debounce_count++;
      }
      else
      {
        state = HIGH;
      }
    }
    else
    {
      if (debounce_count > 0)
      {
        debounce_count--;
      }
      else
      {
        state = LOW;
      }
    }
    
    if ((state == LOW) && (state_old == HIGH))
    {
      // end of pulse
      int pulse;
      int bitvalue = -1;
      high2low = true;
  
      last_pulse_end = currentTime;
      pulse = last_pulse_end - last_pulse_start;
      if ((pulse >= DCF77_MIN_ZERO) && (pulse <= DCF77_MAX_ZERO))
      {
        bitvalue = 0;
      }
      else if ((pulse >= DCF77_MIN_ONE) && (pulse <= DCF77_MAX_ONE))
      {
        bitvalue = 1;
      }
      
      #ifdef DCF77_DEBUG
      // print bitstream for debugging
      Serial.print(bitvalue);
      #endif
      
      if (bitvalue < 0)
      {
        // error, discard cycle
        cbit = -1;
        #ifdef DCF77_DEBUG
        // print invalid cycle time
        Serial.write('(');
        Serial.print(pulse);
        Serial.write(')');
        #endif
      }
      else
      {
        if (cbit == 20)
        {
          timebits = 0;
          datebits = 0;
        }
        else if ((cbit >= 21) && (cbit <= 35))
        {
          timebits |= bitvalue << (cbit - 21);
        }
        else if ((cbit >= 36) && (cbit <= 58))
        {
          datebits |= (unsigned long)bitvalue << (cbit - 36);
        }
      }    
    }
    else if ((state == HIGH) && (state_old == LOW))
    {
      // start of pulse
      last_pulse_start = currentTime;
      if (last_pulse_start - last_pulse_end > 1500)
      {
        #ifdef DCF77_DEBUG
        if (cbit < 0)
        {
          // last cycle was invalid, fresh sync
          Serial.println();
          Serial.println("--sync--");
        }
        else
        {
          // cycle valid, print time
          minute = (timebits & 0x0F) + 10 * ((timebits >> 4) & 0x07);
          hour = ((timebits >> 8) & 0x0F) + 10 * ((timebits >> 12) & 0x03);
          Serial.println();
          if (hour < 10)
          {
            Serial.write('0');
          }
          Serial.print(hour);
          Serial.write(':');
          if (minute < 10)
          {
            Serial.write('0');
          }
          Serial.println(minute);
        }
        #endif
        // start new cycle
        cbit = 0;
      }
      else if (cbit >= 0)
      {
        cbit++;
      }
    }
    state_old = state;

  }
  return high2low;
}

