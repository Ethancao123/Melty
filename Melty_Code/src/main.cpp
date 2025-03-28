#include <Arduino.h>
#include "elrs.hpp"
#include "telem.hpp"

#define DEBUG

bool failsafe = false;

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("COM Serial initialized");
  #endif
  initELRS();
  // initTelem();
}

void loop() {
  #ifdef DEBUG
  printChannels();
  if(failsafe)
    Serial.println("FAILSAFE!");
  #endif
  failsafe = !crsf.isLinkUp();
  loopELRS();
  if(failsafe)
    return;
  
}

// #include "NimBLEDevice.h"

// void setup() {
//     NimBLEDevice::init("NimBLE");
    
//     NimBLEServer *pServer = NimBLEDevice::createServer();
//     NimBLEService *pService = pServer->createService("ABCD");
//     NimBLECharacteristic *pCharacteristic = pService->createCharacteristic("1234");
    
//     pService->start();
//     pCharacteristic->setValue("Hello BLE");
    
//     NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
//     pAdvertising->addServiceUUID("ABCD"); // advertise the UUID of our service
//     pAdvertising->setName("NimBLE"); // advertise the device name
//     pAdvertising->start(); 
// }

// void loop(){}