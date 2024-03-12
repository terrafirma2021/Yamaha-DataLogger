#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
#include <BLE2902.h>
#include <BLECharacteristic.h>
#include <BLEDevice.h>



// Function declaration for handling commands
std::string handleCommand(const std::string &command);

// ELM327 Service and Characteristic UUIDs
const uint16_t ELM327_SERVICE_UUID = 0xFFF0;
const uint16_t ELM327_RX = 0xFFF1;
const uint16_t ELM327_TX = 0xFFF2;

// UART Service and Characteristic UUIDs
const char UART_SERVICE_UUID[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
const char UART_RX[] = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
const char UART_TX[] = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

// Debug level for printing debug messages
#define DEBUG_LEVEL 0

class MyCallbacks;

// Declare sendUart function outside of the class
void sendUart();

// Device class for handling BLE operations
class Device final : public BLEServerCallbacks
{
public:
    ~Device() override = default;

    // Singleton pattern to get instance of Device
    static Device &getInstance()
    {
        static Device instance;
        return instance;
    }

    // Setter for setting ATH1 active mode
    void setATH1Active(bool active)
    {
        ath1Active = active;
    }

    // Check if device is connected
    bool isConnected() const
    {
        return server->getConnectedCount() > 0;
    }

    // Getters for BLE characteristics
    BLECharacteristic *getUartTxCharacteristic() const
    {
        return uartTxCharacteristic;
    }

    BLECharacteristic *getElm327TxCharacteristic() const
    {
        return elm327TxCharacteristic;
    }

    BLECharacteristic *getUartRxCharacteristic() const
    {
        return uartRxCharacteristic;
    }

    // BLE Device Initialization and Service Setup
    bool start(BLECharacteristicCallbacks *callbacks)
    {
        // Initialize BLE device
        BLEDevice::init("Carista");
        BLEDevice::setPower(ESP_PWR_LVL_P9);

        // Create BLE server and set callbacks
        server = BLEDevice::createServer();
        server->setCallbacks(this);

        // Create the ELM327 service and characteristics
        service = server->createService(BLEUUID(ELM327_SERVICE_UUID));
        elm327RxCharacteristic = createCharacteristic(ELM327_RX, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
        elm327TxCharacteristic = createCharacteristic(ELM327_TX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
        elm327TxCharacteristic->setCallbacks(callbacks);

        // UART Service and Characteristics
        BLEUUID uartServiceUUID = BLEUUID(UART_SERVICE_UUID);
        BLEService *uartService = server->createService(uartServiceUUID);

        if (uartService)
        {
            // UART RX Characteristic
            uartRxCharacteristic = uartService->createCharacteristic(
                BLEUUID(UART_RX),
                BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
            if (uartRxCharacteristic)
            {
                uartRxCharacteristic->addDescriptor(new BLE2902());
            }

            // UART TX Characteristic
            uartTxCharacteristic = uartService->createCharacteristic(
                BLEUUID(UART_TX),
                BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
            if (uartTxCharacteristic)
            {
                uartTxCharacteristic->addDescriptor(new BLE2902());
                uartTxCharacteristic->setCallbacks(callbacks);
            }
        }

        // Start BLE services
        service->start();
        uartService->start();
        setupAdvertising();

        return true;
    }

    // BLE Server Callback Methods

    void sendElm(const char *data)
    {
        if (clientConnected)
        {
            elm327RxCharacteristic->setValue((uint8_t *)data, strlen(data));
            elm327RxCharacteristic->notify();
#if DEBUG_LEVEL >= 1
            Serial.print("Sent to Elm: ");
            Serial.println(data);
#endif
        }
    }

    void sendUart()
    {
        if (Serial.available() > 0)
        {
            String serialData = Serial.readStringUntil('\n');
            serialData += '\n';
            uartRxCharacteristic->setValue(serialData.c_str());
            uartRxCharacteristic->notify();

            Serial.print("Sent over BLE: ");
            Serial.println(serialData);
#if DEBUG_LEVEL >= 1
            Serial.print("Sent over BLE: ");
            Serial.println(serialData);
#endif
        }
    }

    void onConnect(BLEServer *) override
    {
        clientConnected = true;
#if DEBUG_LEVEL >= 1
        Serial.println("Device connected");
#endif
        handleATSCommand = false;
        handleATHCommand = false;
#if DEBUG_LEVEL >= 1
        Serial.println("AT Commands Reset");
#endif
    }

    void onDisconnect(BLEServer *) override
    {
        clientConnected = false;
#if DEBUG_LEVEL >= 1
        Serial.println("Device disconnected");
#endif
        BLEDevice::startAdvertising();
    }

    bool handleATHCommand = false;
    bool handleATSCommand = false;
    bool ath1Active = false;

    // Modify the response based on device settings
    std::string sendResponse(const std::string &command, const std::string &response)
    {
        Device &device = Device::getInstance();
        std::string finalResponse = response;

        if (device.ath1Active && command.rfind("01", 0) == 0)
        {
            finalResponse = "7E8 06 " + finalResponse;
#if DEBUG_LEVEL == 2
            Serial.println("ATH1 Active: Headers appended");
#endif
        }

        if (device.handleATSCommand)
        {
            finalResponse.erase(std::remove(finalResponse.begin(), finalResponse.end(), ' '), finalResponse.end());
#if DEBUG_LEVEL == 2
            Serial.println("ATS1 Active: Spaces removed");
#endif
        }

#if DEBUG_LEVEL == 2
        Serial.println(("Final Response: " + finalResponse).c_str());
#endif
        return finalResponse;
    }

private:
    Device()
        : server(), service(), elm327RxCharacteristic(),
          elm327TxCharacteristic(), uartRxCharacteristic(),
          uartTxCharacteristic(), clientConnected(false) {}

    // Helper function to create BLE characteristic
    BLECharacteristic *createCharacteristic(uint16_t uuid, uint32_t properties)
    {
        BLECharacteristic *characteristic = service->createCharacteristic(BLEUUID(uuid), properties);
        characteristic->addDescriptor(new BLE2902());
        return characteristic;
    }

    // Setup advertising for BLE device
    void setupAdvertising()
    {
        BLEAdvertising *advertising = BLEDevice::getAdvertising();
        advertising->addServiceUUID(service->getUUID());
        advertising->setScanResponse(false);
        BLEDevice::startAdvertising();
    }

    BLEServer *server;
    BLEService *service;
    BLECharacteristic *elm327RxCharacteristic;
    BLECharacteristic *elm327TxCharacteristic;
    BLECharacteristic *uartRxCharacteristic;
    BLECharacteristic *uartTxCharacteristic;
    bool clientConnected;
};

class MyCallbacks : public BLECharacteristicCallbacks
{
public:
    void onWrite(BLECharacteristic *characteristic) override
    {
      
        if (characteristic == Device::getInstance().getUartTxCharacteristic())
        {
            Device::getInstance().sendUart();
            // Handle UART TX characteristic data
            uint8_t *value = characteristic->getData();
            size_t len = characteristic->getLength();

            for (size_t i = 0; i < len; i++)
            {
                Serial.print(static_cast<char>(value[i]));
            }
        }
        else if (characteristic == Device::getInstance().getElm327TxCharacteristic())
        {
            // Handling ELM327 TX characteristic data
            uint8_t *value = characteristic->getData();
            size_t len = characteristic->getLength();

            size_t start = 0;
            while (start < len && (value[start] == ' ' || value[start] == '\t' || value[start] == '\n' || value[start] == '\r'))
            {
                start++;
            }

            while (len > start && (value[len - 1] == ' ' || value[len - 1] == '\t' || value[len - 1] == '\n' || value[len - 1] == '\r'))
            {
                len--;
            }

            for (size_t i = start; i < len; i++)
            {
                value[i] = toupper(value[i]);
            }

#if DEBUG_LEVEL >= 1
            Serial.print("Received: ");
            Serial.write(value + start, len - start);
            Serial.println();
            Serial.print("Received data length: ");
            Serial.println(len - start);
#endif

            std::string command((char *)(value + start), len - start);
            std::string response = handleCommand(command);
            std::string finalResponse = Device::getInstance().sendResponse(command, response + "\r>");
            Device::getInstance().sendElm(finalResponse.c_str());

#if DEBUG_LEVEL >= 2
            Serial.print("onWrite: Sent final response: ");
            Serial.println(finalResponse.c_str());
#endif
        }
    }

private:
    // Function definition for sendUart

};

#endif
