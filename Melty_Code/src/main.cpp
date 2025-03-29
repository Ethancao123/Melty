#include <Arduino.h>
#include <FastLED.h>

#include "DShotESC.h"

#include "elrs.hpp"
#include "telemwifi.hpp"

#define DEBUG

#define LED_PIN     D4
#define CHIPSET     WS2812B
#define NUM_LEDS    6

CRGB leds[NUM_LEDS];

#define ESC0_PIN      GPIO_NUM_5
#define ESC1_PIN      GPIO_NUM_6
DShotESC esc0;
DShotESC esc1;

bool failsafe = false;
void setup() {
  #ifdef DEBUG
  delay(1000);
  Serial.begin(115200);
  Serial.println("COM Serial initialized");
  #endif
  initELRS();
  initTelem();

  FastLED.addLeds<WS2812B, LED_PIN>(leds, NUM_LEDS);

  esc0.install(ESC0_PIN, RMT_CHANNEL_0);
  esc1.install(ESC1_PIN, RMT_CHANNEL_1);
  esc0.init();
  esc1.init();
  esc0.set3DMode(true);
  esc1.set3DMode(true);
  for (int i = 0; i < 5; i++)
	{
		esc0.beep(i);
    esc1.beep(i);
	}
}

void loop() {
  #ifdef DEBUG
  // printChannels();
  // if(failsafe)
  //   Serial.println("FAILSAFE!");
  #endif
  //stuff to loop even in failsafe
  failsafe = !crsf.isLinkUp();
  FastLED.show();
  loopELRS();
  if(failsafe) {
    for(int i = 0; i < NUM_LEDS; i++)
      leds[i] = CRGB::Red;
    return;
  }
  //stuff to not loop if in failsafe

    
  
}