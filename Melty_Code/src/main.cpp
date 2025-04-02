#include <Arduino.h>
#define FASTLED_RMT_BUILTIN_DRIVER 1
#include <FastLED.h>
#include "SparkFun_LIS331.h"
#include "ESP32_SoftWire.h"

#include "DShotESC.h"

#include "elrs.hpp"
#include "telemwifi.hpp"

#define DEBUG

#define LED_PIN     D3
#define NUM_LEDS    6

CRGB leds[NUM_LEDS];

#define ESC0_PIN      GPIO_NUM_6
#define ESC1_PIN      GPIO_NUM_7
DShotESC esc0;
DShotESC esc1;

#define XL_CS D1
#define XL_MOSI D10
#define XL_MISO D9
#define XL_SCK D8

LIS331 xl;
int16_t x, y, z;
SoftWire Wire;
bool failsafe = false;

bool firstLoop = true;

enum States {
  FAILSAFE,
  DRIVING,
  MELTING,
  // AUTO
};

States state = FAILSAFE;

void setup() {
  // enableLoopWDT();
  #ifdef DEBUG
  delay(1000);
  Serial.begin(115200);
  Serial.println("COM Serial initialized");
  #endif
  initELRS();
  // initTelem();

  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  FastLED.setBrightness(10);

  x = 0;
  y = 0;
  z = 0;

  digitalWrite(XL_MISO, HIGH);
  Wire.begin(XL_MOSI, XL_SCK, 400000);
  xl.setI2CAddr(0x1B);
  xl.begin(LIS331::USE_I2C); // Selects the bus to be used and sets
                          //  the power up bit on the accelerometer.
                          //  Also zeroes out all accelerometer
                          //  registers that are user writable.

  failsafe = !crsf.isLinkUp();
  for(int i = 0; i < NUM_LEDS; i++)
      leds[i] = CRGB::Purple;
  FastLED.show();
  xl.readAxes(x,y,z);
  loopELRS();
  rmt_driver_uninstall(RMT_CHANNEL_0);
  rmt_driver_uninstall(RMT_CHANNEL_1);
  esc0.install(ESC0_PIN, RMT_CHANNEL_0);
  esc1.install(ESC1_PIN, RMT_CHANNEL_1);
  esc0.init();
  esc1.init();
  esc0.set3DMode(true);
  esc1.set3DMode(true);
  esc0.sendMotorStop();
  esc1.sendMotorStop();
  for (int i = 0; i < 5; i++)
	{
		esc0.beep(i);
    esc1.beep(i);
	}
}

void loop() {
  #ifdef DEBUG
  printChannels();
  if(failsafe)
    Serial.println("FAILSAFE!");
    Serial.print("state: ");
    Serial.println(state);
    Serial.print("Accel: ");
    Serial.print(xl.convertToG(6,x));
    Serial.print(", ");
    Serial.print(xl.convertToG(6,y));
    Serial.print(", ");
    Serial.println(xl.convertToG(6,z));
  #endif

  //stuff to loop even in failsafe
  failsafe = !crsf.isLinkUp();
  FastLED.show();
  xl.readAxes(x,y,z);
  loopELRS();
  if(failsafe) {
    state = FAILSAFE;
    for(int i = 0; i < NUM_LEDS; i++)
      leds[i] = CRGB::Red;
    Serial.println("motors stopping");
    esc0.sendMotorStop();
    esc1.sendMotorStop();
    return;
  }
  if(crsf.getChannel(5) > 1600) {
    state = MELTING;
  } else {
    state = DRIVING;
  }
  if(state = DRIVING) {
    1 == 1;
  }

  if(state = MELTING) {
    1 == 1;
  }
  //stuff to not loop if in failsafe
}