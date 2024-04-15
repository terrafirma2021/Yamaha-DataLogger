#include <Arduino.h>
#include <handlecommand.h>
#include <BLE.h>
#include <gear.h>
#include <spifffs.h>
#include <menu.h>
#include <vector>
#include <unordered_map>
#include "esp_timer.h"
#include "LCD.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2lib.h>
#include <iostream>
#include <string>
#include <Wire.h>
#include <numeric>
#include <SPI.h>
#include <SPIFFS.h>


// Define L9637D Pins
#define YAM_TX 1
#define YAM_RX 38

// Debug Pins
/// #define YAM_TX 12
// #define YAM_RX 33

// Define SDA and SCL pins
#define MY_SCL_PIN 15
#define MY_SDA_PIN 16

// Debug Level
extern bool Debug_RX;
extern bool Debug_TX;
extern bool Debug_PIDS;
bool DisableBikeOff_Flag = false;

// BLE Connected bool
extern bool clientConnected;

// Time thresholds and timeouts
uint32_t Time = esp_timer_get_time() / 1000;
const uint16_t BIKE_OFF_TIMEOUT_TIMER = 5000; // 5 seconds in microseconds
uint32_t lastByteTime = 0; 

// BLE Timers
static uint32_t lastSendTimeUartTX = 0;
static uint32_t lastSendTimeUartRX = 0;
static uint32_t lastSendTimeElmTX = 0;
static uint32_t lastSendTimeElmRX = 0;
const uint16_t sendIntervalUartTX = 1;
const uint16_t sendIntervalUartRX = 2;
const uint16_t sendIntervalElmTX = 3;
const uint16_t sendIntervalElmRX = 4;

// Magic numbers
const byte DIAG_START_BYTE = 0xCD;

// Buffer sizes
#define VEHICLE_SPEED_RAW_BUFFER_SIZE 8
#define ECU_BUFFER_SIZE 5

// Yamaha RX Buffers
using t_buffer_item = uint8_t;
using t_buffer = std::vector<t_buffer_item>;
byte Vehicle_Speed_Raw_Buffer[VEHICLE_SPEED_RAW_BUFFER_SIZE];
byte ECU_Buffer[ECU_BUFFER_SIZE];

// Immo discarded bytes
static uint32_t discardedBytesCount = 0;

// Buffer indices and time variables
byte VehicleSpeedRawBufferIndex = 0;
byte ECUBufferIndex = 0;
byte IMMOIndex = 0;

// Immo flags
bool is3E = false;
bool isIMMOHandled = false;

// ECU Mode
bool NormalData = false;
bool frameEndDetected = false;
bool diagMenu = false;

// Gears
uint8_t gear_speed = 0;
uint16_t gear_rpm = 0;
bool Gear_Speed_Ready = false;
bool Gear_RPM_Ready = false;
extern bool gearLearning;
extern bool ratioReset;
extern std::vector <float> constRatios;

// Top Speed
byte MaxSpeed = 0;

// BLE Arrays
std::queue<std::string> MyCallbacks::bleUartRxQue;
std::queue<std::string> MyCallbacks::bleElmRxQueue;

// PIDS
uint16_t RPM_PID;        // RPM * 50 = RAW
uint8_t Speed_PID;       // RAW km/h
uint8_t Coolant_PID;     // Temp = -30
uint8_t Error_PID;       // Error code
uint8_t Gear_PID = 0;    // RAW 
uint8_t Temp_PID;        // MCU Temp C
uint8_t CPU_PID;         // CPU Freq mhz
uint8_t RAM_Free_PID;    // Free Ram kb
uint8_t Max_Speed_PID;   // Max Speed Reached
uint16_t MCU_Uptime_PID; // Seconds

// Function declarations
void setup();
void loop();
void mainTime();
void YamahaRX();
void processIMMOSequence(t_buffer &buffer, t_buffer_item receivedByte);
void alignedFrame(const t_buffer &buffer);
void handleDiagData(const t_buffer &buffer);
void handleNormalData(const t_buffer &buffer);
void sendResponse(const std::string &message);
void serialRX();
void receiveResponse(std::string message);
void handleBikeOffCondition();
void calculateRPM(t_buffer_item rpmByte);
void calculateVehicleSpeed(t_buffer_item speedByte);
void extractErrorCode(t_buffer_item Error);
void calculateCoolantTemp(t_buffer_item Temp);
void maximumSpeed();
void updateMcuPidValues();
void debugPIDS();
void toUpperCaseInPlace(std::string &str);
void trimInPlace(std::string &str);
extern void gears();
extern void menu(std::string command);
void bleTimers();
extern void loadSpiffRatios();


// Setup
void setup()
{
  delay(1000);
  Serial.begin(115200);
  Serial1.begin(16040, SERIAL_8N1, YAM_RX, YAM_TX);
  Serial.println("Yamaha ELM327 Datalogger");
  Serial.print("MCU Temperature: ");
  Serial.print(temperatureRead());
  Serial.println(" °C");
  Serial.print("CPU Frequency: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(" MHz");
  Serial.print("Free Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.print("Max Alloc Heap: ");
  Serial.print(ESP.getMaxAllocHeap());
  Serial.println(" bytes");
  Wire.begin(MY_SDA_PIN, MY_SCL_PIN);
  u8g2.begin();
  MyCallbacks *myCallbacks = new MyCallbacks();
  bool success = Device::getInstance().start(myCallbacks);
  if (!success)
  {
    sendResponse("Failed to start BLE");
  }
  if (!SPIFFS.begin())
  {
    sendResponse("SPIFFS mount failed");
    return;
  }
  // Init the Gear ratios from the spiffs
  loadSpiffRatios();
  menu("MENU");
}

void loop()
{
  mainTime();
  bleTimers();
  handleBikeOffCondition();
  YamahaRX();
  displayData();
  serialRX();
  debugPIDS();
  updateMcuPidValues();
  gears();
}

void mainTime(){
  Time = esp_timer_get_time() / 1000;
}

void YamahaRX()
{
  static t_buffer buffer;

  if (!Serial1.available())
  {
    return;
  }

  // Update lastByteTime with esp_timer_get_time()
  lastByteTime = esp_timer_get_time() / 1000;

  // Read a byte from Serial1
  t_buffer_item receivedByte = Serial1.read();

  // Check and handle the first byte of the IMMO sequence immediately
  if (!is3E && receivedByte == 0x3E)
  {
    is3E = true;
    buffer.push_back(receivedByte);
    sendResponse("Starting IMMO sequence.");
    return;
  }

  if (is3E && !isIMMOHandled)
  {
    // Continue handling the IMMO sequence
    processIMMOSequence(buffer, receivedByte);
  }
  else if (isIMMOHandled)
  {
    // Ensure buffer does not exceed 5 bytes
    if (buffer.size() >= 5)
    {
      buffer.erase(buffer.begin());
    }
    buffer.push_back(receivedByte);

    // Process only when buffer is exactly 5 bytes
    if (buffer.size() == 5)
    {
      bool allZero = std::all_of(buffer.begin(), buffer.end(), [](uint32_t i)
                                 { return i == 0; });

      if (!allZero)
      {
        t_buffer_item sum = std::accumulate(buffer.begin(), buffer.end() - 1, 0);
        if (sum == buffer.back())
        {
          alignedFrame(buffer);
          buffer.clear(); // Clear the buffer for the next frame
        }
      }
    }
  }
}

void processIMMOSequence(t_buffer &buffer, t_buffer_item receivedByte)
{
  // Push the received byte into the buffer
  buffer.push_back(receivedByte);

  if (buffer.size() == 61)
  {

    if (buffer.back() == 0xCD)
    {
      diagMenu = true; // Diag menu init
      buffer.clear();
      sendResponse("Diag start initiated.");
    }
    else
    {
      buffer.clear();
    }

    isIMMOHandled = true;
    sendResponse("Normal start initiated.");
  }
}

void alignedFrame(const t_buffer &buffer)
{
  // First, ensure that the buffer size is as expected.
  if (buffer.size() != 5)
  {
    return;
  }

  if (diagMenu)
  {
    handleDiagData(buffer);
  }
  else
  {
    handleNormalData(buffer);
  }
}

void handleDiagData(const t_buffer &buffer)
{
  sendResponse("Diag menu!");
}

void handleNormalData(const t_buffer &buffer)
{

  maximumSpeed();

  if (buffer.size() != 5)
  {
    return;
  }

  calculateRPM(buffer[0]);
  calculateVehicleSpeed(buffer[1]);
  extractErrorCode(buffer[2]);
  calculateCoolantTemp(buffer[3]);
}

void sendResponse(const std::string &message)
{
  // Print the message
  Serial.println(message.c_str());

  // Check for empty command
  if (message.length() == 0)
  {
    Serial.println("Empty command received.");
    return; // Exit if the command is empty
  }

  Device::getInstance().bleUartQue(message);
}

std::string serialRXBuffer;  // Buffer to accumulate characters

void serialRX() {
    // Check if the serial connection is still active
    if (!Serial) {
        return;
    }

    if (Serial.available() > 0) {  // Check if there is data to process
        char ch = Serial.read();  // Read one character from the serial

        if (ch == '\n') {  // Check if the character is a newline
            if (!serialRXBuffer.empty()) {
                receiveResponse(serialRXBuffer);  // Process the accumulated input
                serialRXBuffer.clear();  // Clear the buffer after processing
            }
        } else {
            serialRXBuffer += ch;  // Add character to buffer if not newline
        }
    }
}



void receiveResponse(std::string message)
{
  trimInPlace(message);
  toUpperCaseInPlace(message);

  

  if (message.compare("DEBUG OFF") == 0 || message.compare("DEBUG 0") == 0)
  {
    sendResponse("Command Received: Debug Off");
    Debug_RX = false;
    Debug_TX = false;
    Debug_PIDS = false;
    DisableBikeOff_Flag = false;
    return;
  }

  if (message.compare("DEBUG RX") == 0)
  {
    sendResponse("Command Received: Debug RX");
    Debug_RX = true;
    Debug_TX = false;
    Debug_PIDS = false;
    return;
  }

  if (message.compare("DEBUG TX") == 0)
  {
    sendResponse("Command Received: Debug TX");
    Debug_RX = false;
    Debug_TX = true;
    Debug_PIDS = false;
    return;
  }

  if (message.compare("DEBUG PID") == 0 || message.compare("DEBUG PIDS") == 0)
  {
    sendResponse("Command Received: Debug PIDS");
    Debug_RX = false;
    Debug_TX = false;
    Debug_PIDS = true;
    return;
  }

  if (message.compare("BIKE ON") == 0)
  {
    sendResponse("Command Received: Bike timer disabled");
    DisableBikeOff_Flag = false;
    return;
  }

  if (message.compare("BIKE OFF") == 0)
  {
    DisableBikeOff_Flag = true;
    sendResponse("Command Received: Disabled Bike timer");
    return;
  }

  if (message.compare("RATIO RESET") == 0)
  {
    sendResponse("Command Received: Ratio reset test");
    ratioReset = true;
    return;
  }

  if (message.compare("GEARS") == 0)
  {
    sendResponse("Command Received: GEARS, Starting Gear training");
    gearLearning = true;
    return;
  }

  if (message.compare("RESET") == 0)
  {
    sendResponse("Command Received: Bye!");
    ESP.restart();
    return;
  }

  menu(message);  // calls spiffs.h
}


void handleBikeOffCondition()
{
  // Early return if the bike off condition handling is disabled or lastByteTime is zero
  if (DisableBikeOff_Flag || lastByteTime == 0)
  {
    return;
  }
      // Get the current time in microseconds
  uint64_t timeElapsed = Time - lastByteTime; // Calculate time elapsed since the last byte received

  if (timeElapsed > BIKE_OFF_TIMEOUT_TIMER)
  {
    // Reset flags and variables related to bike off condition
    is3E = false;
    isIMMOHandled = false;
    NormalData = false;
    frameEndDetected = false;
    diagMenu = false;
    ECUBufferIndex = 0;
    lastByteTime = 0;
    Gear_PID = 0;
    // Reset Gear
    ratioArray.clear();
    sendResponse("\nBike Off Detected");
  }
}

void calculateRPM(t_buffer_item rpmByte)
{

  uint8_t RPM = rpmByte;

  // Assign the RPM value directly to RPM_PID and gear_rpm.
  RPM_PID = RPM * 50; // Correct up to 12500
  gear_rpm = RPM_PID;
}

void calculateVehicleSpeed(t_buffer_item speedByte)
{

  if (VehicleSpeedRawBufferIndex < VEHICLE_SPEED_RAW_BUFFER_SIZE)
  {
    Vehicle_Speed_Raw_Buffer[VehicleSpeedRawBufferIndex++] = speedByte;
  }

  // Check if the buffer is full.
  if (VehicleSpeedRawBufferIndex == VEHICLE_SPEED_RAW_BUFFER_SIZE)
  {
    int totalSpeed = 0;
    for (int i = 0; i < VEHICLE_SPEED_RAW_BUFFER_SIZE; ++i)
    {
      totalSpeed += Vehicle_Speed_Raw_Buffer[i]; // Accumulate speed data.
    }

    // Process the accumulated speed data.
    Speed_PID = totalSpeed;
    MaxSpeed = totalSpeed;

    gear_speed = Speed_PID;

    // Keep sync of Speed/RPM
    Gear_Speed_Ready = true;
    Gear_RPM_Ready = true;

    // Reset the buffer index to 0 for the next frame.
    VehicleSpeedRawBufferIndex = 0;
  }
}

void extractErrorCode(t_buffer_item Error)
{
  Error_PID = Error;
}

void calculateCoolantTemp(t_buffer_item Temp)
{
  Coolant_PID = Temp - 30;
}

void maximumSpeed()
{
  static byte topSpeed = 10; // Static variable initialization
  if (Speed_PID > topSpeed)
  {
    topSpeed = Speed_PID; // Update if the current Speed_PID is greater
    Max_Speed_PID = topSpeed;
  }
}

void updateMcuPidValues()
{
  static uint32_t lastUpdateTime = 0;
  const uint16_t updateInterval = 500;        // Interval in milisecomds

  // Check if the update interval has passed
  if ((Time - lastUpdateTime) >= updateInterval)
  {
    // Update the last update time to the current time
    lastUpdateTime = Time;
    MCU_Uptime_PID = Time / 1000; // Convert milliseconds to seconds

    // Read the current temperature and update the temperature PID value
    float temperature = temperatureRead();
    Temp_PID = temperature;

    // Update CPU frequency PID value
    CPU_PID = ESP.getCpuFreqMHz();

    // Update free RAM PID value, converting bytes to kilobytes
    RAM_Free_PID = ESP.getFreeHeap() / 1024;
  }
}

void debugPIDS()
{
  if (Debug_PIDS)
  {
    sendResponse(std::string("RPM: ") + std::to_string(RPM_PID) + "\n" +
                 "Vehicle Speed: " + std::to_string(Speed_PID) + "\n" +
                 "Current Gear: " + std::to_string(Gear_PID) + "\n" +
                 "Coolant Temp: " + std::to_string(Coolant_PID) + "\n" +
                 "Error Code: " + std::to_string(Error_PID) + "\n" +
                 "MCU Temp: " + std::to_string(Temp_PID) + "\n" +
                 "CPU Mhz: " + std::to_string(CPU_PID) + "\n" +
                 "Ram Free: " + std::to_string(RAM_Free_PID) + "\n" +
                 "Max Speed: " + std::to_string(Max_Speed_PID) + "\n" +
                 "MCU Uptime Seconds: " + std::to_string(MCU_Uptime_PID) + "\n");
  }
}

void toUpperCaseInPlace(std::string &str)
{
  for (auto &c : str)
  {
    // Convert each character to uppercase
    c = std::toupper(static_cast<unsigned char>(c));
  }
}

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


void bleTimers()
{

  // Existing UART TX processing
  if (Time - lastSendTimeUartTX >= sendIntervalUartTX)
  {
    Device::getInstance().bleUartSend();
    lastSendTimeUartTX = Time;
  }

  // Elm TX queue processing
  if (Time - lastSendTimeElmTX >= sendIntervalElmTX)
  {
    Device::getInstance().bleElmSend();
    lastSendTimeElmTX = Time;
  }

  // Existing UART RX processing
  if (Time - lastSendTimeUartRX >= sendIntervalUartRX)
  {
    MyCallbacks::processUartRXQueue();
    lastSendTimeUartRX = Time;
  }

  // NEW: Elm RX queue processing
  if (Time - lastSendTimeElmRX >= sendIntervalElmRX)
  {
    MyCallbacks::processElmRxQueue();
    lastSendTimeElmRX = Time;
  }
}

