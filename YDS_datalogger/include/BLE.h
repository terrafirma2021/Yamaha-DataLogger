#include <Arduino.h>
#include <BLE2902.h>
#include <BLECharacteristic.h>
#include <BLEDevice.h>
#include <string>

// debug
bool Debug_RX = false;
bool Debug_TX = false;
bool Debug_PIDS = false;

// forward declaration for CalculateGear_Flag;
extern bool CalculateGear_Flag;

// forward declaration for Disable bike timer
extern bool DisableBikeOff_Flag;

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

class MyCallbacks;

/// Device class for handling BLE operations
class Device : public BLEServerCallbacks
{
public:
    BLEServer *server;
    BLEService *serviceELM;
    BLEService *serviceUAR;
    BLECharacteristic *ELMTX;
    BLECharacteristic *ELMRX;
    BLECharacteristic *UartRX;
    BLECharacteristic *UartTX;
    bool clientConnected;
    bool handleATHCommand = false;
    bool handleATSCommand = false;
    bool ath1Active = false;

    ~Device() override = default;

    void onConnect(BLEServer *) override
    {
        clientConnected = true;
        Serial.println("Device connected");
        handleATSCommand = false;
        handleATHCommand = false;
    }

    void onDisconnect(BLEServer *) override
    {
        clientConnected = false;
        Serial.println("Device disconnected");
        BLEDevice::startAdvertising();
    }

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

    // BLE Device Initialization and Service Setup
    bool start(BLECharacteristicCallbacks *callbacks)
    {
        // Initialize BLE device
        BLEDevice::init("Carista");
        BLEDevice::setPower(ESP_PWR_LVL_P9);
        BLEDevice::setMTU(517);

        // Create BLE server and set callbacks
        server = BLEDevice::createServer();
        server->setCallbacks(this);

        // Create the ELM327 service and characteristics ( Swapped RX/TX naming for easy read)
        serviceELM = server->createService(BLEUUID(ELM327_SERVICE_UUID));
        ELMTX = createCharacteristic(ELM327_RX, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

        ELMRX = createCharacteristic(ELM327_TX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
        ELMRX->setCallbacks(callbacks);

        // Create the UART service and characteristics (Swapped RX/TX naming for easy read)
        BLEUUID uartServiceUUID = BLEUUID(UART_SERVICE_UUID);
        serviceUAR = server->createService(uartServiceUUID);

        UartRX = serviceUAR->createCharacteristic(
            BLEUUID(UART_TX),
            BLECharacteristic::PROPERTY_WRITE);

        UartRX->setCallbacks(callbacks);

        UartTX = serviceUAR->createCharacteristic(
            BLEUUID(UART_RX),
            BLECharacteristic::PROPERTY_NOTIFY);
        UartTX->addDescriptor(new BLE2902());

        serviceELM->start();
        serviceUAR->start();

        BLEAdvertising *advertising = BLEDevice::getAdvertising();
        advertising->addServiceUUID(serviceELM->getUUID());
        advertising->addServiceUUID(serviceUAR->getUUID());
        advertising->start();

        return true;
    }

    void sendElm(std::string data)
    {
        if (!clientConnected)
            return;

        ELMTX->setValue(data);
        ELMTX->notify();
        if (Debug_TX)
        {
            Serial.print("Sent to Elm: ");
            Serial.println(data.c_str());
        }
    }

    void sendUART(std::string text)
    {

        if (!clientConnected)
            return;

        text += '\n';
        UartTX->setValue(text);
        UartTX->notify();
    }

    // Modify the response based on device settings
    std::string sendResponse(const std::string &command, const std::string &response)
    {
        Device &device = Device::getInstance();
        std::string amendedResponse = response;

        if (device.ath1Active && command.rfind("01", 0) == 0)
        {
            amendedResponse = "7E8 06 " + amendedResponse;
            if (Debug_RX)
            {
                Serial.println("ATH1 Active: Headers appended");
                std::string msg =
                    "ATH1 Active: Headers appended " + amendedResponse;
            }
        }

        if (device.handleATSCommand)
        {
            amendedResponse.erase(std::remove(amendedResponse.begin(), amendedResponse.end(), ' '), amendedResponse.end());
            if (Debug_RX)
            {
                Serial.println(("ATS1 Active: Spaces removed " + amendedResponse).c_str());
                std::string msg =
                    "ATS1 Active: Spaces removed " + amendedResponse;
            }
        }
        return amendedResponse;
    }

private:
    Device()
        : server(), serviceELM(), serviceUAR(), ELMTX(),
          ELMRX(), UartRX(),
          UartTX(), clientConnected(false) {}

    // Helper function to create BLE characteristic
    BLECharacteristic *createCharacteristic(uint16_t uuid, uint32_t properties)
    {
        BLECharacteristic *characteristic = serviceELM->createCharacteristic(BLEUUID(uuid), properties);
        characteristic->addDescriptor(new BLE2902());
        return characteristic;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
public:
    void onWrite(BLECharacteristic *characteristic) override
    {
        if (characteristic == Device::getInstance().UartRX)
        {
            // Get the data from the characteristic as uint8_t array
            uint8_t *data = characteristic->getData();
            size_t len = characteristic->getLength();

            // If there is data
            if (len > 0)
            {
                UA_RX(data, len); // Send Uart RX
            }
        }
        else if (characteristic == Device::getInstance().ELMRX)

        {
            // Get the data from the characteristic as uint8_t array
            uint8_t *data = characteristic->getData();
            size_t len = characteristic->getLength();

            // If there is data
            if (len > 0)
            {
                ELM_RX(data, len); // fix BLE Stack canary watchpoint triggered (BTC_TASK)
            }
        }
    }

    void UA_RX(uint8_t *data, size_t len)
    {
        // replace this lame String with proper std::string
        std::string command;

        for (size_t i = 0; i < len; i++)
        {
            if (data[i] != '\n')
            {
                command += (char)data[i];
                continue;
            }

            trimInPlace(command);
            toUpperCaseInPlace(command);

            // Process the command based on the received string
            if (command == "DEBUG OFF" || command == "DEBUG 0")
            {
                Debug_RX = false;
                Debug_TX = false;
                Debug_PIDS = false;
                Serial.println("Command Received: Debug Off");
                msg("Command Received: Debug Off");
            }
            else if (command == "DEBUG RX")
            {
                Debug_RX = true;
                Debug_TX = false;
                Debug_PIDS = false;
                Serial.println("Command Received: Debug RX Enabled");
                msg("Command Received: Debug ELM RX Enabled");
            }
            else if (command == "DEBUG TX")
            {
                Debug_RX = false;
                Debug_TX = true;
                Debug_PIDS = false;
                Serial.println("Command Received: Debug TX Enabled");
                msg("Command Received: Debug TX Enabled");
            }
            else if (command == "DEBUG PID")
            {
                Debug_RX = false;
                Debug_TX = false;
                Debug_PIDS = true;
                Serial.println("Command Received: Enabled PID Debug");
                msg("Command Received: Enabled PID Debug");
            }
            else if (command == "RESET")
            {
                Serial.println("Command Received: Reset ESP");
                msg("Command Received: Reset ESP");
                ESP.restart();
            }
            else if (command == "GEAR LEARN")
            {
                CalculateGear_Flag = true;
                Serial.println("Command Received: Starting Gear Learning");
                Serial.println("Read the tutorial on how to use this feature");
                msg("Command Received: Starting Gear Learning");
            }
            else if (command == "BIKE OFF")
            {
                DisableBikeOff_Flag = true;
                Serial.println("Command Received: Disabled Bike timer");
                msg("Command Received: Disabled Bike timer");
            }
            else if (command == "BIKE ON")
            {
                DisableBikeOff_Flag = false;
                Serial.println("Command Received: Enabled Bike timer");
                msg("Command Received: Enabled Bike timer");
            }
            else
            {
                Serial.print("No command found: ");
                Serial.println(command.c_str());
                msg("No command found: " + command);
            }
            // Clear the command string for the next command
            command = "";
        }
    }

    // void ELM_RX(BLECharacteristic *characteristic)
    void ELM_RX(uint8_t *data, size_t len)
    {
        // Find the first non-whitespace character
        size_t start = 0;
        while (start < len && isspace(data[start]))
        {
            start++;
        }

        // Find the last non-whitespace character
        while (len > start && isspace(data[len - 1]))
        {
            len--;
        }

        // Convert to uppercase in place
        for (size_t i = start; i < len; i++)
        {
            data[i] = toupper(data[i]);
        }

        if (Debug_RX)
        {
            Serial.print("Received: ");
            Serial.write(data + start, len - start);
            Serial.println();
        }

        std::string res;
        {
            std::string command((char *)(data + start), len - start);
            std::string response = handleCommand(command);
            res = Device::getInstance().sendResponse(command, response + "\r>");
        }

        Device::getInstance().sendElm(res);
    }

private:
    void toUpperCaseInPlace(std::string &str)
    {
        for (auto &c : str)
        {
            c = std::toupper(static_cast<unsigned char>(c)); // Convert each character to uppercase
        }
    }

    // Device::getInstance().sendUART("Message");
    void trimInPlace(std::string &str)
    {
        // Lambda function to check if a character is not a whitespace
        auto notSpace = [](int ch)
        { return !std::isspace(static_cast<unsigned char>(ch)); };

        // Find the first non-whitespace character
        auto start = std::find_if(str.begin(), str.end(), notSpace);
        // Erase leading whitespace
        str.erase(str.begin(), start);

        // Find the last non-whitespace character
        auto end = std::find_if(str.rbegin(), str.rend(), notSpace).base();
        // Erase trailing whitespace
        str.erase(end, str.end());
    }

    void msg(std::string msg)
    {
        Device::getInstance().sendUART(msg);
    }
};
