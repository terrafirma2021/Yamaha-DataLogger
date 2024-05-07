#pragma once
#include <Arduino.h>
#include <BLE2902.h>
#include <BLECharacteristic.h>
#include <BLEDevice.h>
#include <string>
#include <queue>
#include <string>

// debug
bool Debug_RX = false;
bool Debug_TX = false;
bool Debug_PIDS = false;
bool Debug_YAM = true;

// Main
extern void sendResponse(const std::string &message);
extern void receiveResponse(std::string);
extern void menu(std::string command);

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

// Device class for handling BLE operations
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
    privateSendResponse("Device connected");
    menu("MENU");
    handleATSCommand = false;
    handleATHCommand = false;
  }

  void onDisconnect(BLEServer *) override
  {
    clientConnected = false;
    privateSendResponse("Device disconnected");
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

  void bleElmQue(std::string data)
  {
    if (!clientConnected)
      return;

    // Enqueue new data
    ElmTXQueue.push(data);

    // Ensure the queue doesn't exceed 50 messages
    if (ElmTXQueue.size() > 50)
    {
      ElmTXQueue.pop(); // Remove the oldest data if over the limit
    }
  }

  void bleElmSend()
  {
    // Check if there are messages to send and if the client is connected
    if (!ElmTXQueue.empty() && clientConnected)
    {
      // Get the message at the front of the queue
      const std::string &data = ElmTXQueue.front();

      // Attempt to send the message
      ELMTX->setValue(data);
      ELMTX->notify();

      // Optional debug message
      if (Debug_TX)
      {
        privateSendResponse("Sent to Elm: " + data);
      }

      // Remove the message from the queue after sending
      ElmTXQueue.pop();
    }
  }

  void bleUartQue(const std::string &message)
  {
    if (!clientConnected)
    {
      return;
    }
    // Add the message to the queue
    uartTXQueue.push(message + '\n'); // Ensure newline

    // Check if the queue size exceeds the limit
    if (uartTXQueue.size() > 50)
    {
      uartTXQueue.pop(); // Remove the oldest message from the front of the queue
    }
  }

  void bleUartSend()
  {
    if (!clientConnected || uartTXQueue.empty())
    {
      return;
    }

    const std::string &text = uartTXQueue.front();
    UartTX->setValue(text);
    UartTX->notify();
    uartTXQueue.pop();
  }

  // Modify the response based on device settings
  std::string modifySendResponse(const std::string &command,
                                 const std::string &response)
  {
    Device &device = Device::getInstance();
    std::string amendedResponse = response;

    if (device.ath1Active && command.rfind("01", 0) == 0)
    {
      amendedResponse = "7E8 06 " + amendedResponse;
      if (Debug_RX)
      {
        privateSendResponse("ATH1 Active: Headers appended " + amendedResponse);
      }
    }

    if (device.handleATSCommand)
    {
      amendedResponse.erase(std::remove(amendedResponse.begin(), amendedResponse.end(), ' '), amendedResponse.end());
      if (Debug_RX)
      {
        privateSendResponse("ATS1 Active: Spaces removed " + amendedResponse);
      }
    }
    return amendedResponse;
  }

  static void privateSendResponse(const std::string &message)
  {
    Serial.println(message.c_str());
    getInstance().bleUartQue(message); // Utilizes getInstance to call another method
  }

private:
  Device() : server(),
             serviceELM(),
             serviceUAR(),
             ELMTX(),
             ELMRX(),
             UartRX(),
             UartTX(),
             clientConnected(false) {}

  std::queue<std::string> uartTXQueue;
  std::queue<std::string> ElmTXQueue;

  // Prevent copy construction and assignment
  Device(const Device &) = delete;
  Device &operator=(const Device &) = delete;

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
  // Singleton pattern to get instance of MyCallbacks
  static MyCallbacks &getInstance()
  {
    static MyCallbacks instance;
    return instance;
  }





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
        bleUartRx(data, len); // Send Uart RX
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
        bleElmRx(data, len);
      }
    }
  }

  void bleUartRx(uint8_t *data, size_t len)
  {
    static std::string input; // Persist input across calls

    for (size_t i = 0; i < len; ++i)
    {
      if (data[i] == '\n')
      {
        if (!input.empty())
        {
          bleUartRxQue.push(input);
          input.clear();
        }
      }
      else
      {
        input += static_cast<char>(data[i]); // Safely appending data to the string
      }
    }

    // Optionally, handle any remaining input without a trailing newline
    if (!input.empty())
    {
      bleUartRxQue.push(input);
      input.clear();
    }

    // Check if the queue size exceeds the limit
    if (bleUartRxQue.size() > 20)
    {
      bleUartRxQue.pop(); // Remove the oldest command from the front of the queue
    }
  }

  static void processUartRXQueue()
  {
    if (!bleUartRxQue.empty())
    {
      std::string command = bleUartRxQue.front();
      bleUartRxQue.pop();

      // Handle the command
      receiveResponse(command);
    }
  }
void bleElmRx(uint8_t *data, size_t len)
{
  size_t start = 0;
  if (start < len && isspace(data[start]))
  {
    start++;
  }

  size_t end = len;
  if (end > start && isspace(data[end - 1]))
  {
    end--;
  }

  for (size_t i = start; i < end; i++)
  {
    data[i] = toupper(data[i]);
  }

  if (Debug_RX)
  {
    std::string dataStr(reinterpret_cast<char *>(data + start), end - start);
    std::string message = "Received: " + dataStr;
    sendResponse(message);
  }

  std::string command(reinterpret_cast<char *>(data + start), end - start);
  bleElmRxQueue.push(command);
}

static void processElmRxQueue()
{
  if (!bleElmRxQueue.empty())
  {
    std::string command = bleElmRxQueue.front();
    bleElmRxQueue.pop();

    std::string response = handleCommand(command);

    std::string modifiedResponse = Device::getInstance().modifySendResponse(command, response + "\r>");
    Device::getInstance().bleElmQue(modifiedResponse);
  }
}

private:
  void toUpperCaseInPlace(std::string &str)
  {
    for (auto &c : str)
    {
      c = std::toupper(static_cast<unsigned char>(c)); // Convert each character to uppercase
    }
  }

  static std::queue<std::string> bleUartRxQue; // Queue holding incoming commands
  static std::queue<std::string> bleElmRxQueue;

  // Device::getInstance().sendUART("Message");
  void trimInPlace(std::string &str)
  {
    // Lambda function to check if a character is not a whitespace
    auto notSpace = [](int ch)
    {
      return !std::isspace(static_cast<unsigned char>(ch));
    };

    // Find the first non-whitespace character
    auto start = std::find_if(str.begin(), str.end(), notSpace);
    // Erase leading whitespace
    str.erase(str.begin(), start);

    // Find the last non-whitespace character
    auto end = std::find_if(str.rbegin(), str.rend(), notSpace).base();
    // Erase trailing whitespace
    str.erase(end, str.end());
  }
};