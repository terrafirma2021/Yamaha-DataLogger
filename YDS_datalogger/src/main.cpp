// Including necessary libraries for the Arduino BLE functionality and additional functionality.
#include <Arduino.h>
#include <BLE2902.h>
#include <BLECharacteristic.h>
#include <BLEDevice.h>
#include <yds.h>

// Definition of constants for RX and TX pins.
#define DASH_RX 1
#define DASH_TX 38

// Namespace elm327 is defined to encapsulate the Bluetooth functionality specific to ELM327 devices.
namespace elm327
{

  // UUIDs for the BLE service and characteristics are defined here.
  const uint16_t SERVICE_UUID = 0xFFF0;
  const uint16_t RX_UUID = 0xFFF1;
  const uint16_t TX_UUID = 0xFFF2;

// Define a debug level for controlling the verbosity of serial output for debugging.
#define DEBUG_LEVEL 1

  // Device class definition, inheriting from BLEServerCallbacks for handling BLE events.
  class Device final : public BLEServerCallbacks
  {
  public:
    ~Device() noexcept override = default; // Destructor.

    // Singleton pattern to ensure only one instance of the device.
    static Device &getInstance() noexcept
    {
      static Device instance;
      return instance;
    }

    // Method to set ATH1 status.
    void setATH1Active(bool active)
    {
      ath1Active = active;
    }

    // Method to check if the device is connected.
    bool isConnected() const noexcept
    {
      return server->getConnectedCount() > 0;
    }

    // Method to start the BLE service.
    bool start(BLECharacteristicCallbacks *callbacks) noexcept
    {
      BLEDevice::init("Carista");          // Initialize BLE device with a name.
      BLEDevice::setPower(ESP_PWR_LVL_P9); // Set the power level of the BLE device.

      // Creating BLE server and setting it up.
      server = BLEDevice::createServer();
      server->setCallbacks(this);

      // Creating a BLE service with the defined UUID.
      service = server->createService(BLEUUID(SERVICE_UUID));

      // Creating RX and TX characteristics with appropriate properties.
      rxCharacteristic = createCharacteristic(RX_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
      txCharacteristic = createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
      txCharacteristic->setCallbacks(callbacks);

      // Starting the service and setting up advertising.
      service->start();
      setupAdvertising();

      return true;
    }

    // Method to send data to the client.
    void send(const char *data) noexcept
    {
      if (clientConnected)
      {
        rxCharacteristic->setValue((uint8_t *)data, strlen(data));
        rxCharacteristic->notify();
#if DEBUG_LEVEL >= 1
        Serial.print("Sent: ");
        Serial.println(data); // Debug print.
#endif
      }
    }

    // Callback when device gets connected.
    void onConnect(BLEServer *) override
    {
      clientConnected = true;
#if DEBUG_LEVEL >= 1
      Serial.println("Device connected"); // Debug print.
#endif
      // Reset state of certain commands on connect.
      handleATSCommand = false;
      handleATHCommand = false;
#if DEBUG_LEVEL >= 1
      Serial.println("AT Commands Reset"); // Debug print.
#endif
    }

    // Callback when device gets disconnected.
    void onDisconnect(BLEServer *) override
    {
      clientConnected = false;
#if DEBUG_LEVEL >= 1
      Serial.println("Device disconnected"); // Debug print.
#endif
      BLEDevice::startAdvertising(); // Restart advertising on disconnect.
    }

    // Variables to track the state of certain AT commands.
    bool handleATHCommand = false;
    bool handleATSCommand = false;
    bool ath1Active = false;

  private:
    // Private constructor for the singleton pattern.
    Device() noexcept
        : server(nullptr), service(nullptr), rxCharacteristic(nullptr),
          txCharacteristic(nullptr), clientConnected(false) {}

    // Helper method to create BLE characteristics.
    BLECharacteristic *createCharacteristic(uint16_t uuid, uint32_t properties)
    {
      BLECharacteristic *characteristic = service->createCharacteristic(BLEUUID(uuid), properties);
      characteristic->addDescriptor(new BLE2902());
      return characteristic;
    }

    // Method to setup BLE advertising.
    void setupAdvertising()
    {
      BLEAdvertising *advertising = BLEDevice::getAdvertising();
      advertising->addServiceUUID(service->getUUID());
      advertising->setScanResponse(false);
      BLEDevice::startAdvertising(); // Start advertising.
    }

    // Member variables for BLE server, service, characteristics, and connection status.
    BLEServer *server;
    BLEService *service;
    BLECharacteristic *rxCharacteristic;
    BLECharacteristic *txCharacteristic;
    bool clientConnected;
  };

  // Class for handling BLE characteristic callbacks.
  class MyCallbacks : public BLECharacteristicCallbacks
  {
  public:
    // Override for handling 'write' events on the BLE characteristic.
    void onWrite(BLECharacteristic *characteristic) override
    {
      uint8_t *value = characteristic->getData();
      size_t len = characteristic->getLength();

      // Trim whitespace and new lines from the start and end of the received value.
      size_t start = 0;
      while (start < len && (value[start] == ' ' || value[start] == '\t' || value[start] == '\n' || value[start] == '\r'))
      {
        start++;
      }

      while (len > start && (value[len - 1] == ' ' || value[len - 1] == '\t' || value[len - 1] == '\n' || value[len - 1] == '\r'))
      {
        len--;
      }
      // Convert data to uppercase
      for (size_t i = start; i < len; i++)
      {
        value[i] = toupper(value[i]);
      }

#if DEBUG_LEVEL >= 1
      Serial.print("Received: ");
      Serial.write(value + start, len - start); // Debug print for received value.
      Serial.println();
      Serial.print("Received data length: ");
      Serial.println(len - start); // Debug print for data length.
#endif

      // Processing the received command and preparing a response.
      std::string command((char *)(value + start), len - start);
      std::string response = handleCommand(command);
      std::string finalResponse = sendResponse(command, response + "\r>");
      Device::getInstance().send(finalResponse.c_str());

#if DEBUG_LEVEL >= 2
      Serial.print("onWrite: Sent final response: ");
      Serial.println(finalResponse.c_str()); // Debug print for final response.
#endif
    }

  private:
    std::string handleCommand(const std::string &command)
    {
      std::string response;
      if (command == "ATI" || command == "AT@1")
      {
        response = "ELM327 v2.1";
      }
      else if (command == "ATS1" || command == "AT S1")
      {
        Device::getInstance().handleATSCommand = false; // Turn Spaces On
        response = "OK";
      }
      else if (command == "ATS0" || command == "AT S0")
      {
        Device::getInstance().handleATSCommand = true; // Turn Spaces Off
        response = "OK";
      }
      else if (command == "ATH1" || command == "AT H1")
      {
        response = "OK";
        Device::getInstance().handleATHCommand = true;
        Device::getInstance().setATH1Active(true); // Set ATH1 active
      }
      else if (command == "ATH0" || command == "AT H0")
      {
        Device::getInstance().handleATHCommand = false;
        Device::getInstance().setATH1Active(false); // Reset ATH1 active
        response = "OK";
      }
      else if (command == "ATZ")
      { // Reset
        Device::getInstance().handleATHCommand = false;
        Device::getInstance().setATH1Active(false);
        Device::getInstance().handleATSCommand = false;
        response = "OK";
      }
      else if (command == "ATE0" || command == "ATPC" || command == "ATM0" ||
               command == "ATL0" || command == "ATST62" || command == "ATSP0" ||
               command == "ATSP0" || command == "ATAT1" || command == "ATAT2" ||
               command == "ATAT2" || command == "ATSP6" || command == "ATSPA6")
      {
        response = "OK";
      }
      else if (command == "ATDPN")
      {
        response = "6"; // Protocol Number 6 CAN bus
      }
      else if (command == "0100")
      { // PID 1 - Supported PIDs [01-20]
        response = "41 00 08 18 00 01";
      }
      else if (command == "0120")
      { // PID 2 - Supported PIDs [21-40]
        response = "41 20 00 00 00 01";
      }
      else if (command == "0140")
      { // PID 3 - Supported PIDs [41-60]
        response = "41 40 00 00 00 01";
      }
      else if (command == "0160")
      { // PID 4 - Supported PIDs [61-80]
        response = "41 60 00 00 00 01";
      }
      else if (command == "0180")
      { // PID 5 - Supported PIDs [81-A0]
        response = "41 80 00 00 00 00";
      }
      else if (command == "01A0")
      { // PID 6 - Supported PIDs [A1-C0]
        response = "41 A0 10 00 00 00 00";
      }
      else if (command == "01C0")
      { // PID 7 - Supported PIDs [C1-E0]
        response = "41 C0 NO DATA";

        // Handle PID Requests
      }
      else if (command == "0105")
      { // Engine Coolant Temperature (PID 0105)
        response = "41 05 " + std::to_string(Coolant_PID);
      }
      else if (command == "010C")
      { // RPM (PID 010C)
        response = "41 0C " + std::to_string(RPM_PID);
      }
      else if (command == "010D")
      { // Vehicle Speed (PID 010D)
        response = "41 0D " + std::to_string(Speed_PID);
      }
      else if (command == "01A4")
      { // Transmission Actual Gear  (PID 01A4)
        response = "41 0D 0" + std::to_string(Gear_PID);
      }
      else if (command == "0902")
      { // VIN (PID 0902)
        response = "49 02 00 00 59 41 4D 41 48 41 45 53 50 33 32 4F 44 42";
      }
      else if (command == "0904")
      { // Calibration ID (PID 0904)
        response = "49 04 00 00 00 00";
      }
      else if (command == "090A")
      { // ECU Name (PID 090A)
        response = "49 0A 45 53 50 33 32 37 45 6D 75 6C 61 74 6F 72 00 00 00 00 00 00";
      }
      else if (command == "01009")
      { // ???
        response = "41 009 NO DATA";
      }
      else
      {
#if DEBUG_LEVEL >= 1
        Serial.println(("Unknown command or length mismatch. Received: " + command).c_str());
#endif
      }
      return response;
    }

    // Method to format and send response back to the client.
    std::string sendResponse(const std::string &command, const std::string &response)
    {
      Device &device = Device::getInstance();
      std::string finalResponse = response;

      if (device.ath1Active && command.rfind("01", 0) == 0)
      {
        finalResponse = "7E8 06 " + finalResponse;
        debugPrint("ATH1 Active: Headers appended");
      }

      if (device.handleATSCommand)
      {
        finalResponse.erase(std::remove(finalResponse.begin(), finalResponse.end(), ' '), finalResponse.end());
        debugPrint("ATS1 Active: Spaces removed");
      }

      debugPrint("Final Response: " + finalResponse);
      return finalResponse;
    }

    void debugPrint(const std::string &message)
    {
#if DEBUG_LEVEL == 2
      Serial.println(message.c_str());
#endif
    }
  };

} // namespace elm327

// Arduino setup function - initializes serial communication and starts the BLE service.
void setup()
{
  Serial.begin(115200);
  Serial1.begin(16040, DASH_RX, DASH_TX);
  Serial.println("Yamaha ESP32 S3 BLE elm327 Emulator");
  delay(1000); // Delay for 1000ms
  elm327::MyCallbacks *myCallbacks = new elm327::MyCallbacks();
  bool success = elm327::Device::getInstance().start(myCallbacks);
  if (!success)
  {
    Serial.println("Failed to start BLE");
  }
}

void loop()
{
  handleBikeOffCondition();
  readAndProcessSerialData();
}
