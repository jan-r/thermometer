/*
  Implementation of class DCF77_Module
  
  Copyright (c) 2017 jan-r @ GitHub
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
*/
#include "dcf77.h"
#include <Time.h>

#define DCF77_MIN_ZERO    55      // minimum pulse width for "zero"
#define DCF77_MAX_ZERO    120     // maximum pulse width for "zero"
#define DCF77_MIN_ONE     155     // minimum pulse width for "one"
#define DCF77_MAX_ONE     220     // maximum pulse width for "one"

#define DCF77_DEBOUNCE_CYCLES   2


// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------
DCF77_Module::DCF77_Module(int powerPin, int signalPin)
: pinPower(powerPin), pinSignal(signalPin), currentBitIndex(-1)
{
  pinMode(pinPower, OUTPUT);
  pinMode(pinSignal, INPUT);
  disable();
}

void DCF77_Module::enable()
{
  digitalWrite(pinPower, LOW);
  lastPulseStart = millis();
  lastPulseEnd = lastPulseStart;
  isPowered = true;
}

void DCF77_Module::disable()
{
  digitalWrite(pinPower, HIGH);
  isPowered = false;
}

// ----------------------------------------------------------------------------
// Process the DCF77 signal.
// Return true if a high-to-low-transition was detected.
// ----------------------------------------------------------------------------
bool DCF77_Module::process(unsigned long currentTime)
{
  static unsigned long last_update = 0;
  static int debounce_count = 0;
  bool high2low = false;

  // only process the signal if the module is powered on
  if (!isPowered)
  {
    // in this case, we can always act as if we just detected a bit so other
    // tasks can be triggered immediately
    high2low = true;
  }
  else
  {
    // only process every 2 ms to get some debouncing and glitch immunity
    if (currentTime - last_update > 2)
    {
      int current_pin_value = digitalRead(pinSignal);
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
          tcoLineState = HIGH;
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
          tcoLineState = LOW;
        }
      }
      
      if ((tcoLineState == LOW) && (tcoLineStateOld == HIGH))
      {
        // end of pulse
        int pulse;
        int bitvalue = -1;
        high2low = true;
    
        lastPulseEnd = currentTime;
        pulse = lastPulseEnd - lastPulseStart;
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
          currentBitIndex = -1;
          #ifdef DCF77_DEBUG
          // print invalid cycle time
          Serial.write('(');
          Serial.print(pulse);
          Serial.write(')');
          #endif
        }
        else
        {
          // valid bit length, store value
          if (currentBitIndex >= 32)
          {
            bits[1] |= (unsigned long)bitvalue << (currentBitIndex - 32);
          }
          else if (currentBitIndex >= 0)
          {
            bits[0] |= (unsigned long)bitvalue << currentBitIndex;
          }
        }    
      }
      else if ((tcoLineState == HIGH) && (tcoLineStateOld == LOW))
      {
        // start of pulse
        lastPulseStart = currentTime;
        if (lastPulseStart - lastPulseEnd > 1500)
        {
          // start of new one-minute-cycle detected
          if ((currentBitIndex == 58) || (currentBitIndex == 59))
          {
            // cycle valid, sync internal clock
            int h = hour();
            int m = minute();
            int d = day();
            int mo = month();
            int y = year() + 2000;
    
            if (checkRcvdStream())
            {
              setTime(h, m, 0, d, mo, y);
            }
            #ifdef DCF77_DEBUG
            else
            {
              Serial.println();
              Serial.println("pe");
              Serial.println(bits[0], HEX);
              Serial.println(bits[1], HEX);
            }
            #endif
  
            #ifdef DCF77_DEBUG
            Serial.println();
            Serial.print(h);
            Serial.write(':');
            Serial.print(m);
            Serial.write(' ');
            Serial.print(d);
            Serial.write('.');
            Serial.print(mo);
            Serial.write('.');
            Serial.println(y);
            #endif
          }
          else
          {
            // last cycle was invalid, fresh sync
            #ifdef DCF77_DEBUG
            Serial.println();
            Serial.println("--sync--");
            #endif
          }
  
          // start new cycle
          currentBitIndex = 0;
          bits[0] = bits[1] = 0UL;
        }
        else if (currentBitIndex >= 0)
        {
          currentBitIndex++;
        }
      }
      tcoLineStateOld = tcoLineState;
  
    }
  }
  return high2low;
}

int DCF77_Module::hour()
{
  unsigned long timebits = (bits[0] >> 21) | (bits[1] << 11);
  return ((timebits >> 8) & 0x0F) + 10 * ((timebits >> 12) & 0x03);
}

int DCF77_Module::minute()
{
  unsigned long timebits = (bits[0] >> 21) | (bits[1] << 11);
  return (timebits & 0x0F) + 10 * ((timebits >> 4) & 0x07);
}

int DCF77_Module::day()
{
  return ((bits[1] >> 4) & 0xFU) + 10 * ((bits[1] >> 8) & 0x3);
}

int DCF77_Module::month()
{
  return ((bits[1] >> 13) & 0xFU) + 10 * ((bits[1] >> 17) & 0x1);
}

int DCF77_Module::year()
{
  return ((bits[1] >> 18) & 0xFU) + 10 * ((bits[1] >> 22) & 0xFU);
}

bool DCF77_Module::checkParity(unsigned long bitsToCheck)
{
  char parity = 0;
  for (char i = 0; i < 32; i++)
  {
    parity ^= bitsToCheck & 1;
    bitsToCheck >>= 1;
  }
  return (parity == 0);
}

bool DCF77_Module::checkRcvdStream()
{
  unsigned long bitsToCheck;
  bool isOk = true;

  // -----------------------------------------------------------------
  // Parity checks
  // -----------------------------------------------------------------
  // minutes (bit 28:21)
  bitsToCheck = bits[0] & 0x1FE00000UL;
  if (!checkParity(bitsToCheck))
  {
    isOk = false;
  }

  // hours (bit 35:29)
  bitsToCheck = (bits[0] & 0xE0000000UL) | (bits[1] & 0x0000000FUL);
  if (!checkParity(bitsToCheck))
  {
    isOk = false;
  }

  // date (bit 58:36)
  bitsToCheck = bits[1] & 0x7FFFFF0UL;
  if (!checkParity(bitsToCheck))
  {
    isOk = false;
  }

  // -----------------------------------------------------------------
  // Sanity checks
  // -----------------------------------------------------------------
  int val = minute();
  if ((val < 0) || (val > 59))
  {
    isOk = false;
  }

  val = hour();
  if ((val < 0) || (val > 23))
  {
    isOk = false;
  }

  val = day();
  if ((val < 1) || (val > 31))
  {
    isOk = false;
  }

  val = month();
  if ((val < 1) || (val > 12))
  {
    isOk = false;
  }
  
  return isOk;
}

