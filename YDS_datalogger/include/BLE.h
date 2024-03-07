// BLE.h
#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
#include <BLE2902.h>
#include <BLECharacteristic.h>
#include <BLEDevice.h>

std::string handleCommand(const std::string &command);

// ELM327 Service and Characteristic UUIDs
const uint16_t ELM327_SERVICE_UUID = 0xFFF0;
const uint16_t ELM327_RX_UUID = 0xFFF1;
const uint16_t ELM327_TX_UUID = 0xFFF2;

// UART Service and Characteristic UUIDs
const char UART_SERVICE_UUID[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
const char UART_RX_UUID[] = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
const char UART_TX_UUID[] = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";

class Device final : public BLEServerCallbacks
{
public:
    ~Device() noexcept override = default;

    static Device &getInstance() noexcept
    {
        static Device instance;
        return instance;
    }

    void setATH1Active(bool active)
    {
        ath1Active = active;
    }

    bool isConnected() const noexcept
    {
        return server->getConnectedCount() > 0;
    }

    // BLE Device Initialization and Service Setup
    bool start(BLECharacteristicCallbacks *callbacks) noexcept
    {
        BLEDevice::init("Carista");
        BLEDevice::setPower(ESP_PWR_LVL_P9);

        server = BLEDevice::createServer();
        server->setCallbacks(this);

        // Create the ELM327 service and characteristics
        service = server->createService(BLEUUID(ELM327_SERVICE_UUID));
        rxCharacteristic = createCharacteristic(ELM327_RX_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
        txCharacteristic = createCharacteristic(ELM327_TX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
        txCharacteristic->setCallbacks(callbacks);

        // UART Service and Characteristics
        BLEUUID uartServiceUUID = BLEUUID(UART_SERVICE_UUID);
        BLEService *uartService = server->createService(uartServiceUUID);

        if (uartService != nullptr) {
            uartRxCharacteristic = uartService->createCharacteristic(
                BLEUUID(UART_RX_UUID),
                BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
            if (uartRxCharacteristic != nullptr) {
                uartRxCharacteristic->addDescriptor(new BLE2902());
                uartRxCharacteristic->setCallbacks(callbacks);
            }

            uartTxCharacteristic = uartService->createCharacteristic(
                BLEUUID(UART_TX_UUID),
                BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
            if (uartTxCharacteristic != nullptr) {
                uartTxCharacteristic->addDescriptor(new BLE2902());
            }
        }

        service->start();
        if (uartService != nullptr) {
            uartService->start();
        }
        setupAdvertising();

        return true;
    }

    // BLE Server Callback Methods
    void send(const char *data) noexcept
    {
        if (clientConnected)
        {
            rxCharacteristic->setValue((uint8_t *)data, strlen(data));
            rxCharacteristic->notify();
#if DEBUG_LEVEL >= 1
            Serial.print("Sent: ");
            Serial.println(data);
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

private:
    Device() noexcept
        : server(nullptr), service(nullptr), rxCharacteristic(nullptr),
          txCharacteristic(nullptr), uartRxCharacteristic(nullptr),
          uartTxCharacteristic(nullptr), clientConnected(false) {}

    BLECharacteristic *createCharacteristic(uint16_t uuid, uint32_t properties)
    {
        BLECharacteristic *characteristic = service->createCharacteristic(BLEUUID(uuid), properties);
        characteristic->addDescriptor(new BLE2902());
        return characteristic;
    }

    void setupAdvertising()
    {
        BLEAdvertising *advertising = BLEDevice::getAdvertising();
        advertising->addServiceUUID(service->getUUID());
        advertising->setScanResponse(false);
        BLEDevice::startAdvertising();
    }

    BLEServer *server;
    BLEService *service;
    BLECharacteristic *rxCharacteristic;
    BLECharacteristic *txCharacteristic;
    BLECharacteristic *uartRxCharacteristic = nullptr;
    BLECharacteristic *uartTxCharacteristic = nullptr;
    bool clientConnected;
};

// Handle the BLE 
class MyCallbacks : public BLECharacteristicCallbacks
{
public:
    void onWrite(BLECharacteristic *characteristic) override
    {
        {
            // This is ELM327 data
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
            std::string finalResponse = sendResponse(command, response + "\r>");
            Device::getInstance().send(finalResponse.c_str());

#if DEBUG_LEVEL >= 2
            Serial.print("onWrite: Sent final response: ");
            Serial.println(finalResponse.c_str());
#endif

        }
    }

private:
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
};

#endif
