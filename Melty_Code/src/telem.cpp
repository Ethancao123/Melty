#include "telem.hpp"

BLECharacteristic *pTelemetryChar;
BLEServer *pServer;

void initTelem() {
    NimBLEDevice::init("Walnut");
    pServer = NimBLEDevice::createServer();
    BLEService *pService = pServer->createService("WALL");

    pTelemetryChar = pService->createCharacteristic("1234");

    pService->start();
    pServer->getAdvertising()->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID("WALL");
    pAdvertising->setName("Walnut");
    pAdvertising->start();
}

void sendTelem(String msg) {
    uint8_t data[10];
    data[5] = 5;
    pTelemetryChar->setValue(data,10);
}

