#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_ESP8266_RAW_PIN_ORDER
#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#define FASTLED_ESP8266_D1_PIN_ORDER
#include <FastLED.h>


#define PIN 12
#define N_LEDS 144


CRGB leds[N_LEDS];

void setup() {
  Serial.begin(9600);
  FastLED.setMaxPowerInVoltsAndMilliamps(5,1500); 
  FastLED.addLeds<NEOPIXEL, PIN>(leds, N_LEDS);
}

void loop() {
  static uint8_t hue_position = 0;
  
  for (uint8_t i=0; i < N_LEDS; i++) {
    leds[i] = CHSV(hue_position+i, 20, 20);
  }
  
  hue_position = hue_position + 1;
  
  FastLED.show();
  delay(10);
}
