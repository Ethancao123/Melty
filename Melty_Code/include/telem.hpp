#include <NimBLEDevice.h>
#define SERVICE_UUID "d855bae0-36cf-4663-a96c-c83b841a8e58"
#define CHARACTERISTIC_UUID "5d0c62b3-fc1c-409e-ab3a-acb0144da664"

void initTelem();
void sendTelem(String msg);