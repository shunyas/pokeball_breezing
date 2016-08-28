/*
 * Breezing Light for Pokeball
 * 
 * Copyright (c) 2016 Shunya Sato
 * Author: Shunya Sato

 Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
 
 */

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Adafruit_NeoPixel.h>

// Utility macros to reduce current consumption
// http://www.technoblogy.com/show?KX0
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

#define DEBUG true
#define SOUND true  // true: turn sound on, false: turn sound off for testing silently

#define PIXEL_COUNT 1
#define MAX_SLEEP_ITERATIONS 3

const int piezoPin = 3;
const int pixelPin = 10; // somehow A0 doesn't work...??
// A0 works fine on Arduino Uno. But not on ATmega328 on a breadboard with 8MHz internal clock
// v1.1 hardware must implement jumper wire from Pin 10 to A0
const int pixPWRPin = A1;
const int modePins[] = {6, 5}; // use pin 10 for neopixel

int sleepIterations = 0;
volatile bool watchdogActivated = false;

// Parameter 1 = number of pixels in strip,  neopixel stick has 8
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, pixelPin, NEO_GRB + NEO_KHZ800);
int brightness = 126;

// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  watchdogActivated = true;
}

void go_sleep(){
  Serial.println(F("Going to sleep!"));
  lightOFF();
  digitalWrite(pixPWRPin, HIGH);  // cut power for neopixel
  noInterrupts(); // disable interrupts i.e. cli();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable(); // Set the SE (sleep enable) bit.
  //attachInterrupt(1, sleep_isr, RISING);
  sleep_bod_disable();
  interrupts(); // enable interrupts i.e. sei();
  sleep_cpu();  // Put the device into sleep mode. The SE bit must be set beforehand, and it is recommended to clear it afterwards.

  /* wake up here */
  sleep_disable();
  //digitalWrite(pixPWRPin, LOW);  // turn on to begin LOW: ON, HIGH: OFF
  interrupts(); // end of some_condition
  Serial.println(F("I'm awake!"));
  //init_vals();
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("### Breezing Pokeball ###"));
  pinMode(A0, INPUT); // using A0 to control neopixel didn't work somehow...
  digitalWrite(A0, LOW); // make sure internal pullup is not set

  pinMode(piezoPin, INPUT);
  pinMode(pixPWRPin, OUTPUT);
  digitalWrite(pixPWRPin, LOW);  // turn on to begin
  adc_disable();  // turn off ADC circuit to reduce current consumption in sleep
  strip.begin();
  strip.setBrightness(brightness);
  strip.setPixelColor(0, 255,255,255);
  strip.show();
  delay(100);
  strip.setPixelColor(0,0,0,0);
  strip.show(); // Initialize all pixels to 'off'
  Serial.println(F("Neopixel OFF"));
  digitalWrite(pixPWRPin, HIGH);  // cut power for neopixel

  // Below taken from
  // https://github.com/tdicola/Low_Power_Wifi_Datalogger/blob/master/Example_2_Power_Down_Sleep/Example_2_Power_Down_Sleep.ino
  
  // Setup the watchdog timer to run an interrupt which
  // wakes the Arduino from sleep every 8 seconds.
  
  // Note that the default behavior of resetting the Arduino
  // with the watchdog will be disabled.
  
  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
  noInterrupts();
  
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);
  
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  
  // Enable interrupts again.
  interrupts();
  breeze_once();
}

void loop() {
  if (watchdogActivated){
    watchdogActivated = false;
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS){
      sleepIterations = 0;
      breeze_once();
    }
  }
  go_sleep();
}

void breeze_once(){
  digitalWrite(pixPWRPin, LOW);  // turn on to begin
  Serial.println(F("Neopixel ON"));
  for (int i=0; i<=255; i++){
    //Serial.println(i);
    strip.setPixelColor(0, i, i, i);
    strip.show();
    delay(5);
  }
  for (int i=255; i>=0; i--){
    //Serial.println(i);
    strip.setPixelColor(0, i, i, i);
    strip.show();
    delay(10);
  }
  digitalWrite(pixPWRPin, HIGH);  // cut power for neopixel
  Serial.println(F("Neopixel OFF"));
  delay(100);
}

void lightOFF(){
  for (int i=0; i<PIXEL_COUNT; i++){
    strip.setPixelColor(i, 0,0,0);
  }
  strip.show();
}

