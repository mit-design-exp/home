/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows 
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Arduino.h"
#include "core_pins.h"
#include "pins_arduino.h"
#include "HardwareSerial.h"
#include "IntervalTimer.h"

// Import required libraries
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <Wire.h>           // For I2C comm, but needed for not getting compile error
// DISPLAY 
#define OLED_RESET  16  // Pin 15 -RESET digital signal
#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
ArducamSSD1306 display(OLED_RESET); // FOR I2C

// Capacitance VARIABLE AND CONSTANT DECLARATIONS
float RESISTANCE= 1.0e6;               // Resistance [Ohm] of R in circuit
int NUMBER_OF_MEASUREMENTS = 20;      // number of successive measurements to make before reporting C
unsigned long cycles;                 // cycle count till digital hi
float cap;                            // estimated capacitance
float time_to_digital_hi;             // time to digital Hi
int osciMonitor=0;

// Tone variables for playing tone with frequency dependent on A9 or A7
#define vAverages 20
#define adcRes 14
#define dacRes 12
#define adcMax 16384
#define toneMotorMax 100.0
#define toneMotorMin 0.5
#define motorPwmFreq 50e3
#define motorOnRatio 2
#define tonePianoMax 2000
#define tonePianoMin 20
#define toneWirelessMax 200e3
#define toneWirelessMin 10e3
#define toneOut1 14
#define toneOut2 15
#define mode0 2
#define mode1 3
#define mode2 4
#define display1 19
#define display2 18
#define loopMonitor 10
#define labCap 0
#define labPiano 1
#define labBrushless 2
#define labWireless 3 
#define hBridgeEnable 7

#define plusOut 22
#define zeroOut 20
#define potV A7
#define sigV A9

unsigned int labNum = 0;
float capScale = 0.025;

// Timing variables and definitons
elapsedMicros checkTime; // used to estimate precise deltaT.
elapsedMicros dispUpdateTime; // used to slow display updates during capacitance reading

// Variables for generating tones
float toneFq=0.0;
float toneFq2=0.0;
float toneFreqSettle=0.0;
IntervalTimer tone_timer;
IntervalTimer tone_timer2;  // for Brushless motor.
boolean flipper = true;

void toneSetup()
{
  pinMode(toneOut1, OUTPUT);
  pinMode(toneOut2, OUTPUT);
  digitalWriteFast(toneOut1, LOW);
  digitalWriteFast(toneOut2, LOW);
}

// Plays tone on pin, and complement on pin+1
int fracCnt = 0;
void tone(float frequency)
{
  // Reset outputs and timer
  tonestop();
  fracCnt = 0;
  float usec = 5.0e5/frequency;
  if((labNum != labBrushless) || (labNum != labPiano)) { // One pin tone.
    digitalWriteFast(toneOut1, LOW);
    digitalWriteFast(toneOut2, LOW); 
  } else {
    digitalWriteFast(toneOut1, LOW);
    digitalWriteFast(toneOut2, HIGH); 
  }
  tone_timer.begin(tone_interrupt, usec);
}

void tone2(float frequency)
{
  float usec = 5.0e5/frequency;
  tone_timer2.begin(tone2_interrupt, usec);
  
}

void tonestop()
{
    tone_timer.end();
    tone_timer2.end();
}


void tone_interrupt(void)
{
  if (labNum == labPiano) {
    if(fracCnt == 0) {
      fracCnt = 1;
      digitalWriteFast(toneOut1, LOW);
      digitalWriteFast(toneOut2, LOW);
    } else {
      fracCnt -= 1;
      digitalWriteFast(toneOut1, HIGH);  
      digitalWriteFast(toneOut2, LOW);   
    }
  }
  else if(labNum == labBrushless) { // Brushless toggles one pin or other.
    int pin1 = toneOut1;
    int pin2 = toneOut2;
    if (flipper) {
      pin1 = toneOut2;
      pin2 = toneOut1;
    }
    if(fracCnt == 0) {
      fracCnt = motorOnRatio;
      digitalWriteFast(pin1, HIGH);
      digitalWriteFast(pin2, HIGH);
    } else {
      fracCnt -= 1;
      digitalWriteFast(pin1, HIGH);
      digitalWriteFast(pin2, LOW);     
    }
  } else {
    if(fracCnt == 0) {
      fracCnt = 1;
      digitalWriteFast(toneOut1, LOW);
      digitalWriteFast(toneOut2, HIGH);
    } else {
      fracCnt -= 1;
      digitalWriteFast(toneOut1, HIGH);
      digitalWriteFast(toneOut2, LOW);    
    }
  }

  if (labNum != labCap) {
    osciMonitor = (osciMonitor==0) ? 4095 : 0;
    analogWrite(A14,osciMonitor);
  }
}

// Second Tone justs flips the flipper.
void tone2_interrupt(void)
{
  flipper = !flipper;
}


float capCalc(int pinA, int pinB, float R_meg) {
int numCavg = 3;
long numCsLoops = 0;
long numCspmLoops = 0;
long numLoops = 0;
  // Measure three time intervals,
  // 1) T1 = Both pinA and pinB set low, then pulled up together, measures R*cstray
  // 2) T2 = PinA held high, pinB set low, and then pulled up, measures 0.5*R*(Cm+cstray) 
  // 3) T3 = PinB held high, pinA set low, and then pulled up, measures 0.5*R*(Cm+cstray) 
  // Cm = (T2+T3 - T1)/R.

  pinMode(pinA,OUTPUT); 
  pinMode(pinB,OUTPUT);
  
  for (int j = numCavg; j > 0; j--) {
    // Measure parasitic cap from both pins to ground
    digitalWrite(pinA,LOW);
    digitalWrite(pinB,LOW);
    delayMicroseconds(500);
    pinMode(pinB,INPUT);
    pinMode(pinA,INPUT); 
    for (numLoops = 0; numLoops < 100000; numLoops++) {
      if((digitalReadFast(pinB) + digitalReadFast(pinA)) == 2) break;
    }
    pinMode(pinA,OUTPUT); 
    pinMode(pinB,OUTPUT);
    numCsLoops += numLoops;

    // Measure cap by letting A side charge up.
    digitalWrite(pinA,LOW);
    digitalWrite(pinB,HIGH);
    delayMicroseconds(500); // Really important for caps 1000pf or more
    pinMode(pinA,INPUT); 
    for (numLoops = 0; numLoops < 100000; numLoops++) {
      if((digitalReadFast(pinA) + digitalReadFast(pinA)) == 2) break;
    }
    pinMode(pinA,OUTPUT); 
    numCspmLoops += numLoops;

    // Measure cap letting B side charge up
    digitalWrite(pinA,HIGH);
    digitalWrite(pinB,LOW);
    delayMicroseconds(500);
    pinMode(pinB,INPUT); 
    for (numLoops = 0; numLoops < 100000; numLoops++) {
      if((digitalReadFast(pinB) + digitalReadFast(pinB)) == 2) break;
    }
    pinMode(pinB,OUTPUT);
    numCspmLoops += numLoops;
//    
//    Serial.print(numCsLoops);
//    Serial.print(" b ");
//    Serial.println(numCspmLoops);

  }
//  Serial.print(numCsLoops);
//  Serial.print(" ");
//  Serial.println(numCspmLoops);
  float deltaLoops = numCspmLoops - numCsLoops;
  return((deltaLoops/float(numCavg))/R_meg);
}

void lab_display(float display_val){
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  //display.println("Accelerometer");
  switch (labNum) {
  case 0:
      display.println("Accelerometer");
      display.setTextSize(2);
      display.setCursor(25,25);
      display.println(String(display_val, 3) );
      display.setCursor(50,50);
      display.println("pF");
    break;
  case 1:
      display.println("Resistor Piano");
      display.setTextSize(2);
      display.setCursor(25,25);
      display.println(String(display_val, 3) );
      display.setCursor(50,50);
      display.println("V");
    break;
  case 2:
      display.println("Brushless Motor");
      display.setTextSize(2);
      display.setCursor(25,25);
      display.println(String(display_val, 3) );
      display.setCursor(50,50);
      display.println("Hz");
    break;
  case 3:
      display.println("Wireless Power");
      display.setTextSize(2);
      display.setCursor(25,25);
      display.println(String(display_val, 3) );
      display.setCursor(50,50);
      display.println("kHz");
    break;
  default:
      display.clearDisplay();
    break;
   }
   display.display();
}

// Set up capacitonce sensor and tone inputs.
void setup() {
  // Set up inputs
  Serial.begin(115200);
  pinMode(display1, OUTPUT);
  pinMode(display2, OUTPUT);
  display.begin();  // Switch OLED
  display.clearDisplay();
  display.setTextColor(WHITE);

  analogReadResolution(adcRes);
  analogWriteResolution(dacRes);
  pinMode(potV, INPUT);
  pinMode(sigV, INPUT);
  pinMode(mode0, INPUT);
  pinMode(mode1, INPUT); 
  pinMode(mode2, INPUT); 
  pinMode(zeroOut, OUTPUT);
  digitalWrite(zeroOut, LOW);
  pinMode(plusOut, OUTPUT);
  digitalWrite(plusOut, HIGH);
  pinMode(loopMonitor, OUTPUT);
  
  toneSetup();
  flipper = false;
  pinMode(hBridgeEnable, OUTPUT);
  digitalWrite(hBridgeEnable, LOW);
}  


bool monitorSwitch = true;

void loop() {
  monitorSwitch=!monitorSwitch;
  digitalWrite(loopMonitor,monitorSwitch);
  labNum = digitalRead(mode0)+digitalRead(mode1)+digitalRead(mode2);
  float display_val = 0.0;
  float delayPeriod = 0.0;  // Time between loops in millisec
  float dispPeriod = 1e5;  // Time between display updates in usecs
  
  // Read potentiometer or input signal voltage, average and scale to zero->one.
  int inV = (labNum == labPiano) ? sigV : potV;
  int vInt = 0;
  for(int i = 0; i < vAverages; i++) vInt += analogRead(inV); 
  float vIn = float(vInt)/float(vAverages*adcMax);

  
  //Serial.println(labNum);
  if (labNum == labCap) {
    dispPeriod = 1e6;
    digitalWrite(hBridgeEnable, LOW);
    tonestop();
    float capVal = capCalc(0,1,1.0);
    display_val=capScale*capVal;
    analogWrite(A14, int(10.0*display_val));
  } 
  else {
    digitalWrite(hBridgeEnable, LOW);
    float toneMin = toneMotorMin;
    float toneMax = toneMotorMax;
    delayPeriod = 50;
    
    if (labNum == labPiano) { // Piano lab
      toneMin = tonePianoMin; 
      toneMax = tonePianoMax; 
    }
    else if (labNum == labWireless) { // Wireless lab
      toneMin = toneWirelessMin; 
      toneMax = toneWirelessMax;  
    }

    float toneFreq = toneMin + int(float((toneMax-toneMin))*vIn);
    toneFreq = min(max(toneMin,toneFreq), toneMax);
    if (labNum == labPiano) {
      digitalWrite(hBridgeEnable, LOW);
      // Detect signal has settled, eliminate small jitters once settled.
      if ((toneFreq > 1.01*toneFreqSettle) || (toneFreq < 0.99*toneFreqSettle)) {
        toneFreqSettle = toneFreq;
      } else if (toneFq == 0 
                   || ((toneFreq < 0.95*toneMax) && ((toneFreq < 0.995*toneFq) || (toneFreq > 1.005*toneFq)))) {
        toneFq = toneFreq;
        tone(toneFreq);
        Serial.println(toneFreq);
      } 
      display_val=3.3*vIn;
    } else if (labNum == labWireless) {
      // Check that wireless center freq changes significantly.
      digitalWrite(hBridgeEnable, HIGH);
      if ((toneFreq < 0.99*toneFq) || (toneFreq > 1.01*toneFq)) {
        toneFq = toneFreq;
        tone(toneFreq);
      }
      display_val=float(toneFreq/1000.0);
    } else { // Make brushless motor change with cube of Vin.
      digitalWrite(hBridgeEnable, HIGH);
      if (toneFq != motorPwmFreq) {
        toneFq = motorPwmFreq;
        tone(motorPwmFreq); 
      }
      float expVin = vIn*vIn*vIn*vIn; //2.0*(vIn*(1+vIn*(0.5+0.33*vIn)));
      toneFreq = toneMin + (toneMax-toneMin)*expVin;
      toneFreq = min(max(toneMin,toneFreq), toneMax);
      if ((toneFreq < 0.99*toneFq2) || (toneFreq > 1.01*toneFq2)) {
        toneFq2 = toneFreq;
        tone2(toneFreq);
      }
      display_val=float(toneFreq);  
    }
  }
  // Only occasionally update display and capacitance scaling.
  if(float(dispUpdateTime) > dispPeriod) {
    lab_display(display_val); 
    dispUpdateTime = 0;
    if(labNum == labCap) capScale = 0.1+0.1*vIn;
  }
  delay(delayPeriod);
}
