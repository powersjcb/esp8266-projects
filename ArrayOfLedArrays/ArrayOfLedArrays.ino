// ArrayOfLedArrays - see https://github.com/FastLED/FastLED/wiki/Multiple-Controller-Examples for more info on
// using multiple controllers.  In this example, we're going to set up three NEOPIXEL strips on three
// different pins, each strip getting its own CRGB array to be played with, only this time they're going
// to be all parts of an array of arrays.

#include "FastLED.h"

#define NUM_STRIPS 1
#define NUM_LEDS_PER_STRIP 29
#define DATA_PIN D7
#define CLOCK_PIN D6
#define COLOR_ORDER BGR

CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

// For mirroring strips, all the "special" stuff happens just in setup.  We
// just addLeds multiple times, once for each strip
void setup() {
  // tell FastLED there's 60 NEOPIXEL leds on pin 10
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds[0], NUM_LEDS_PER_STRIP);
}


unsigned short period = 2;

void loop() {
  // This outer loop will go over each strip, one at a time
  unsigned long time = millis();
  
  for (int x = 0; x < NUM_STRIPS; x++) {
    // This inner loop will go over each led in the current strip, one at a time
    
    for (int i = 0; i < NUM_LEDS_PER_STRIP; i++) {
      if ((time / 1000) % period == 0) {
        leds[x][i] = CRGB(55,0,0);
      } else {
        leds[x][i] = CRGB(0,0,0);
      }
      FastLED.show();
    }
  }
}
