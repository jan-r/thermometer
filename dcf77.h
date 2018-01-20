/*
  Definition of class DCF77_Module

  A class to control a DCF77 time receiver module I found in a broken alarm clock.
  Should work with similar modules from the usual sources.
  
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

#ifndef DCF77_H_
#define DCF77_H_

#include <Arduino.h>

//#define DCF77_DEBUG               // define to add serial debugging output


class DCF77_Module
{
public:
  // Constructor
  DCF77_Module(int powerPin, int signalPin);
  
  // Process the DCF77 signal
  bool process(unsigned long currentTime);

  // Enable the module
  void enable();

  // Disable the module
  void disable();

private:
  int pinPower;
  int pinSignal;
  char tcoLineState;
  char tcoLineStateOld;
  unsigned long lastPulseStart;
  unsigned long lastPulseEnd;
  char currentBitIndex;
  unsigned long bits[2];
  bool isPowered;

  int hour();
  int minute();
  int day();
  int month();
  int year();

  bool checkParity(unsigned long bitsToCheck);
  bool checkRcvdStream();
};




#endif // DCF77_H_

