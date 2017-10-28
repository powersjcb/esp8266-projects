// Simple NeoPixel test.  Lights just a few pixels at a time so a
// 1m strip can safely be powered from Arduino 5V pin.  Arduino
// may nonetheless hiccup when LEDs are first connected and not
// accept code.  So upload code first, unplug USB, connect pixels
// to GND FIRST, then +5V and digital pin 6, then re-plug USB.
// A working strip will show a few pixels moving down the line,
// cycling between red, green and blue.  If you get no response,
// might be connected to wrong end of strip (the end wires, if
// any, are no indication -- look instead for the data direction
// arrows printed on the strip).

#include <Adafruit_NeoPixel.h>

#define PIN 5
#define N_LEDS 2

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
}

void loop() {
  blink(strip.Color(20, 0, 0)); // Red
  blink(strip.Color(0, 20, 0)); // Green
  blink(strip.Color(0, 0, 20)); // Blue
}

static void blink(uint32_t c) {
    strip.setPixelColor(0, c); // Draw new pixel
    strip.setPixelColor(1, c); // Draw new pixel
    strip.show();
}
