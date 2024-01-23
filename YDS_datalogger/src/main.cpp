
#include <Arduino.h>
#include <handlecommand.h>
#include <BLE.h>
#include <yds.h>

// Definition for L9637D pins
#define DASH_RX 1
#define DASH_TX 38 // Left for future use

void setup() {
    Serial.begin(115200);
    Serial1.begin(16040, DASH_RX, DASH_TX);
    Serial.println("Yamaha ESP32 S3 BLE elm327 Emulator");
    delay(1000);

    MyCallbacks *myCallbacks = new MyCallbacks();
    bool success = Device::getInstance().start(myCallbacks);
    if (!success) {
        Serial.println("Failed to start BLE");
    }
}

// Function to read and process serial data, and handle UART data
void readAndProcessSerialData()
{
  while (Serial.available())
  {
    byte incomingByte = Serial.read();
    processSerialByte(incomingByte);
  }
}


void loop()
{
  handleBikeOffCondition();
  readAndProcessSerialData();
}

