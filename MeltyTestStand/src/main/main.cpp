#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>

// MT6701 I2C Address (default is 0x06)
const int MT6701_ADDR = 0x06;

// BLHeli ESC Pin on test stand
const int ESC_PIN = D5; 
Servo esc;

// Data packet from Robot
typedef struct struct_message {
    uint32_t timestamp;
    float accel_x;
    float accel_y;
    float accel_z;
} struct_message;

struct_message robotData;
volatile bool newData = false;

// RPM Calculation Variables
float previousAngle = 0;
uint32_t lastRpmTime = 0;
float currentRpm = 0;

// Command and Safety Variables
uint32_t lastCommandTime = 0;
int currentThrottle = 1000; // Default to motor off
const uint32_t TIMEOUT_MS = 1000; // 1 second without Python = Failsafe

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&robotData, incomingData, sizeof(robotData));
  newData = true;
}

float readMT6701Angle() {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(0x03); 
  Wire.endTransmission(false);
  Wire.requestFrom(MT6701_ADDR, 2, true);
  
  if(Wire.available() <= 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    uint16_t rawAngle = (highByte << 6) | (lowByte >> 2);
    return (rawAngle / 16384.0) * 360.0; 
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(1000); 
  delay(3000); // Allow BLHeli time to initialize and beep
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  
  previousAngle = readMT6701Angle();
  lastRpmTime = millis();
}

void loop() {
  // 1. Read Serial Commands from Python
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int commandedPWM = input.toInt();
    
    // Bounds check the BLHeli microseconds command
    if (commandedPWM >= 1000 && commandedPWM <= 2000) {
      currentThrottle = commandedPWM;
      lastCommandTime = millis(); // Reset failsafe timer
    }
  }

  // 2. Safety Failsafe Check
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    currentThrottle = 1000; // Force idle if Python disconnects/times out
  }

  // Write the evaluated throttle to the ESC
  esc.writeMicroseconds(currentThrottle);

  // 3. Calculate Ground Truth RPM
  uint32_t currentTime = millis();
  if (currentTime - lastRpmTime >= 50) { 
    float currentAngle = readMT6701Angle();
    
    float deltaAngle = currentAngle - previousAngle;
    if (deltaAngle < -180.0) deltaAngle += 360.0;
    else if (deltaAngle > 180.0) deltaAngle -= 360.0;
    
    float dt = (currentTime - lastRpmTime) / 1000.0; 
    currentRpm = (deltaAngle / 360.0) / dt * 60.0; 
    
    previousAngle = currentAngle;
    lastRpmTime = currentTime;
  }

  // 4. Output Serial Data
  if (newData) {
    newData = false;
    // Format: Timestamp_Robot, Commanded_PWM, Accel_X, Accel_Y, Accel_Z, RPM
    Serial.print(robotData.timestamp); Serial.print(",");
    Serial.print(currentThrottle); Serial.print(",");
    Serial.print(robotData.accel_x); Serial.print(",");
    Serial.print(robotData.accel_y); Serial.print(",");
    Serial.print(robotData.accel_z); Serial.print(",");
    Serial.println(abs(currentRpm));
  }
}