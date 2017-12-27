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

#define DCF77_MIN_ZERO    70      // minimum pulse width for "zero"
#define DCF77_MAX_ZERO    120     // maximum pulse width for "zero"
#define DCF77_MIN_ONE     170     // minimum pulse width for "one"
#define DCF77_MAX_ONE     220     // maximum pulse width for "one"

#define DCF77_DEBOUNCE_CYCLES   2


// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------
DCF77_Module::DCF77_Module(int powerPin, int signalPin)
: pinPower(powerPin), pinSignal(signalPin), cbit(-1), hour(-1), minute(-1)
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
}

void DCF77_Module::disable()
{
  digitalWrite(pinPower, HIGH);
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
    
    if ((state == LOW) && (stateOld == HIGH))
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
    else if ((state == HIGH) && (stateOld == LOW))
    {
      // start of pulse
      lastPulseStart = currentTime;
      if (lastPulseStart - lastPulseEnd > 1500)
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
    stateOld = state;

  }
  return high2low;
}

