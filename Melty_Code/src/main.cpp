#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "SparkFun_LIS331.h"
#include "ESP32_SoftWire.h"

#include "DShotESC.h"

#include "elrs.hpp"
#include "telemwifi.hpp"

#define DEBUG

#define LED_PIN     D3
#define NUM_LEDS    6

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

#define ESC0_PIN      GPIO_NUM_6
#define ESC1_PIN      GPIO_NUM_5
DShotESC esc0;
DShotESC esc1;
float th0 = 0;
float th1 = 0;

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

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(10); // Set BRIGHTNESS to about 1/5 (max = 255)

  x = 0;
  y = 0;
  z = 0;

  digitalWrite(XL_MISO, HIGH);
  Wire.begin(XL_MOSI, XL_SCK, 400000);
  xl.setI2CAddr(0x19);
  xl.begin(LIS331::USE_I2C); // Selects the bus to be used and sets
                          //  the power up bit on the accelerometer.
                          //  Also zeroes out all accelerometer
                          //  registers that are user writable.

  failsafe = !crsf.isLinkUp();
  for(int i = 0; i < NUM_LEDS; i++){
    strip.setPixelColor(i, strip.Color(255,0,255));
  }
  strip.show();
  xl.readAxes(x,y,z);
  loopELRS();
  esc0.install(ESC0_PIN, RMT_CHANNEL_3);
  esc1.install(ESC1_PIN, RMT_CHANNEL_2);
  esc0.init();
  esc1.init();
  delay(1000);
  esc0.set3DMode(true);
  esc1.set3DMode(true);
  esc0.sendMotorStop();
  esc1.sendMotorStop();
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
    Serial.print("Throttle: ");
    Serial.print(th0);
    Serial.print(", ");
    Serial.println(th1);
  #endif

  //stuff to loop even in failsafe
  failsafe = !crsf.isLinkUp();
  strip.show();
  xl.readAxes(x,y,z);
  loopELRS();
  if(failsafe) {
    state = FAILSAFE;
    for(int i = 0; i < NUM_LEDS; i++)
      strip.setPixelColor(i, strip.Color(255,0,0));
    Serial.println("motors stopping");
    esc0.sendMotorStop();
    esc1.sendMotorStop();
    esc0.beep(2);
    esc1.beep(2);
    return;
  }
  th0 = 0;
  th1 = 0;
  for(int i = 0; i < NUM_LEDS; i++)
    strip.setPixelColor(i, strip.Color(0,0,255));
  if(crsf.getChannel(5) > 1600) {
    state = MELTING;
  } else {
    state = DRIVING;
  }
  if(state == DRIVING) {
    th0 = (crsf.getChannel(2)-1500) + (crsf.getChannel(1)-1500)/2;
    th1 = (crsf.getChannel(2)-1500) - (crsf.getChannel(1)-1500)/2;
    
  }

  if(state == MELTING) {
    1 == 1;
  }
  //stuff to not loop if in failsafe
  esc0.sendThrottle3D(th0);
  esc1.sendThrottle3D(th1);
}
