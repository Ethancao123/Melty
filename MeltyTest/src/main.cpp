#include <Arduino.h>
#ifdef OPEN_DRAIN
  #undef OPEN_DRAIN
#endif
#include <esp_now.h>
#include <WiFi.h>
#include "SparkFun_LIS331.h"
#include "ESP32_SoftWire.h"

// --- REPLACE WITH YOUR TEST STAND ESP32 MAC ADDRESS ---
uint8_t testStandAddress[] = {0xD8, 0x3B, 0xDA, 0x41, 0x63, 0x14}; //d8:3b:da:41:63:14

// Pins from your main.cpp
#define XL_MOSI D10
#define XL_SCK D8
#define ACCELMULT 100

// Data packet structure matching the receiver
typedef struct struct_message {
    uint32_t timestamp;
    float accel_x;
    float accel_y;
    float accel_z;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

LIS331 xl;
SoftWire SWire;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional debugging: Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  
  // Initialize LIS331 exactly as in main.cpp
  SWire.begin(XL_MOSI, XL_SCK, 400000);
  xl.setI2CAddr(0x19);
  xl.begin(LIS331::USE_I2C); 
  xl.setPowerMode(LIS331::NORMAL);
  xl.setODR(LIS331::DR_1000HZ);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, testStandAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  int16_t x, y, z;
  xl.readAxes(x, y, z);

  myData.timestamp = micros();
  // Using the convertToG scaling from main.cpp
  myData.accel_x = xl.convertToG(ACCELMULT, x);
  myData.accel_y = xl.convertToG(ACCELMULT, y);
  myData.accel_z = xl.convertToG(ACCELMULT, z);

  esp_now_send(testStandAddress, (uint8_t *) &myData, sizeof(myData));
  
  delay(10); // 100Hz transmission rate
}