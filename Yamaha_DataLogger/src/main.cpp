#include <Arduino.h>
#include <handlecommand.h>
#include <BLE.h>
#include <vector>
#include <unordered_map>
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "LCD.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2lib.h>
#include <iostream>
#include <string>
#include <Wire.h>
#include <numeric>

// Define L9637D Pins
 #define YAM_TX 1
 #define YAM_RX 38

// Debug Test Pins for L9637D
///#define YAM_TX 12
//#define YAM_RX 33

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

// Watchdog timer Fix
bool isTempReady = false;

// Time thresholds and timeouts
const unsigned long BIKE_OFF_TIMEOUT_TIMER = 5000000;   // 5 seconds in microseconds

// Magic numbers
const byte DIAG_START_BYTE = 0xCD;

// Buffer sizes
#define VEHICLE_SPEED_RAW_BUFFER_SIZE 8
#define ECU_BUFFER_SIZE 5

// Buffers
byte Vehicle_Speed_Raw_Buffer[VEHICLE_SPEED_RAW_BUFFER_SIZE];
byte ECU_Buffer[ECU_BUFFER_SIZE];

// Immo discarded bytes
static int discardedBytesCount = 0;

// using byte = unsigned char;
// using t_buffer_item = byte;
using t_buffer_item = uint8_t;
using t_buffer = std::vector<t_buffer_item>;

// Buffer indices and time variables
byte VehicleSpeedRawBufferIndex = 0;
byte ECUBufferIndex = 0;
byte IMMOIndex = 0;

// Immo flags
bool is3E = false;
bool isIMMOHandled = false;

bool NormalData = false;
bool frameEndDetected = false;
bool diagMenu = false;

// Function declarations
void setup();
void loop();
void serialRX();
void YamahaRX();
void processIMMOSequence(t_buffer &buffer, t_buffer_item receivedByte);
void alignedFrame(const t_buffer &buffer);
void handleDiagData(const t_buffer &buffer);
void handleNormalData(const t_buffer &buffer);
void sendResponse(const std::string &message);
void serialRX();
void receiveResponse(std::string);
void handleBikeOffCondition();
void calculateRPM(t_buffer_item rpmByte);
void calculateVehicleSpeed(t_buffer_item speedByte);
void extractErrorCode(t_buffer_item Error);
void calculateCoolantTemp(t_buffer_item Temp);
void nvs_defaults(const char *Namespace);
void nvs_myinit();
void CalculateGear(uint16_t currentSpeed, uint16_t currentRPM);
void UpdateRatioBuffer(uint16_t currentSpeed, uint16_t currentRPM);
void ProcessGearRatio();
uint16_t CalculateMostFrequentRatio();
void StoreGearRatio(uint16_t gearRatio);
void CheckAndHandleGearChange();
void CheckAndFinalizeGearCalculations();
void WriteCurrentGearToNVS();
void updateGearPID();
void maximumSpeed();
void updateMcuPidValues();
void debugPIDS();
void toUpperCaseInPlace(std::string &str);
void trimInPlace(std::string &str);

// Calculate Gear Constants
const uint16_t Gear_Vector_Size = 333; // 15*333 = 4995 ms (Gear detection time)
const uint8_t Gear_max = 6;            // Maximum number of gears
const uint8_t Neutral_Threshold = 10;  // Tolerance threshold for neutral detection (Speed)
const uint8_t Ratio_Threshold = 10;    // Threshold for detecting gear change based on ratio difference (Need to Tune)
const uint32_t Gear5_Timer = 9000000;  // Timer duration for determining if the bike has 5 gears (9 seconds in microseconds)

// Calculate Gear variables
uint8_t current_gear = 0;
uint16_t gear_Current_Ratio = 0;

// Calculate Gear Flags
bool gear_rpm_Flag = false;
bool gear_speed_Flag = false;
bool GearIs5 = false;
bool GearsAreCalc = false;
bool CalculateGear_Flag = false;

// Gear Calculation
byte gear_speed = 0;
byte gear_rpm = 0;

// Top Speed
byte MaxSpeed = 0;

// Calculate Gear Buffers
std::vector<uint16_t> ratio_buffer;
std::vector<std::vector<uint16_t>> current_gear_buffer(Gear_max);
std::vector<uint16_t> last_ratio(Gear_max, 0);
uint64_t gear5_timer_start = 0;

// NVS related variables
uint16_t stored_gear_buffer[] = {6};
bool nvs_data_read = false;

// Global variable to store the retrieved gear values
const int MAX_STORED_GEARS = 6; // Maximum number of stored gears in the buffer
uint16_t NVS_stored_gear_buffer[MAX_STORED_GEARS];
uint16_t NVS_index_current_gear = 0; // Variable to store the current gear ratio index from NVS

// PIDS
uint16_t RPM_PID;        // RPM * 50 = RAW
uint8_t Speed_PID;       // RAW km/h
uint8_t Coolant_PID;     // Temp = RAW
uint8_t Error_PID;       // Error code
uint8_t Gear_PID;        // RAW 00-05
uint8_t Temp_PID;        // MCU Temp
uint8_t CPU_PID;         // CPU Freq mhz
uint8_t RAM_Free_PID;    // Free Ram
uint8_t Max_Speed_PID;   // Max Speed Reached
uint16_t MCU_Uptime_PID; // How long have we been alive

// Setup
void setup()
{
    delay(1000);
    Serial.begin(115200);
    Serial1.begin(16040, SERIAL_8N1, YAM_RX, YAM_TX);
    Serial.println("Yamaha ELM327 Datalogger");
    Serial.println("MCU Temperature: " + String(temperatureRead()) + " °C");
    Serial.println("CPU Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");
    Serial.println("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
    Serial.println("Max Alloc Heap: " + String(ESP.getMaxAllocHeap()) + " bytes"); // Maximum allocatable block of memory
    nvs_myinit();
    Wire.begin(MY_SDA_PIN, MY_SCL_PIN);
    u8g2.begin();
    MyCallbacks *myCallbacks = new MyCallbacks();
    bool success = Device::getInstance().start(myCallbacks);
    if (!success)
    {
        sendResponse("Failed to start BLE");
    }
}

void loop()
{
    handleBikeOffCondition();
    YamahaRX();
    displayData();
    serialRX();
    debugPIDS();
    updateMcuPidValues();
}

void YamahaRX()
{
    static t_buffer buffer;

    if (!Serial1.available())
    {
        return;
    }
    
    // Update the timestamp of the last received byte
    lastByteTime = esp_timer_get_time();

    t_buffer_item receivedByte = Serial1.read();

    // Print everything
    Serial.println(receivedByte, HEX); // Print in hexadecimal format


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
            bool allZero = std::all_of(buffer.begin(), buffer.end(), [](int i)
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
    updateGearPID(); 
}


void sendResponse(const std::string &message)
{
    Serial.println(message.c_str());         
    Device::getInstance().sendUART(message); 
}

void serialRX()
{
    if (!Serial.available())
    {
        return;
    }

    std::string input = "";
    while (Serial.available())
    {
        char ch = Serial.read();
        if (ch == '\n')
        {
            break;
        }
        input += ch;
    }

    if (!input.empty())
    {
        receiveResponse(input); // Process the input string if it's not empty
    }
}

// Handle Handle BLE Uart/Serial receive response
void receiveResponse(std::string message)
{
    trimInPlace(message);
    toUpperCaseInPlace(message);

    if (message == "DEBUG OFF" || message == "DEBUG 0")
    {
        sendResponse("Command Received: Debug Off");
        Debug_RX = false;
        Debug_TX = false;
        Debug_PIDS = false;
        DisableBikeOff_Flag = false;
        return;
    }

    if (message == "DEBUG RX")
    {
        sendResponse("Command Received: Debug RX");
        Debug_RX = true;
        Debug_TX = false;
        Debug_PIDS = false;
        return;
    }

    if (message == "DEBUG TX")
    {
        sendResponse("Command Received: Debug TX");
        Debug_RX = false;
        Debug_TX = true;
        Debug_PIDS = false;
        return;
    }

    if (message == "DEBUG PID" || message == "DEBUG PIDS")
    {
        sendResponse("Command Received: Debug PIDS");
        Debug_RX = false;
        Debug_TX = false;
        Debug_PIDS = true;
        return;
    }

    if (message == "BIKE ON")
    {
        sendResponse("Command Received: Bike timer disabled");
        DisableBikeOff_Flag = false;
        return;
    }

    if (message == "BIKE OFF")
    {
        DisableBikeOff_Flag = true;
        sendResponse("Command Received: Disabled Bike timer");
        return;
    }

    if (message == "RESET")
    {
        sendResponse("Command Received: Bye!");
        ESP.restart();
        return;
    }

    if (message == "GEAR LEARN")
    {
        sendResponse("Command Received: Starting Gear Learning");
        sendResponse("Read the tutorial on how to use this feature");
        CalculateGear_Flag = true;
        return;
    }

    sendResponse("Command Not Known: " + message);
}


void handleBikeOffCondition()
{
    // Early return if the bike off condition handling is disabled or lastByteTime is zero
    if (DisableBikeOff_Flag || lastByteTime == 0)
    {
        return;
    }

    auto currentTime = esp_timer_get_time(); // Get the current time in microseconds
    auto timeElapsed = currentTime - lastByteTime; // Calculate time elapsed since the last byte received

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
        sendResponse("Bike Off Detected");
    }
}


void calculateRPM(t_buffer_item rpmByte)
{

    uint16_t RPM = rpmByte; // Directly use rpmByte

    // Assign the RPM value directly to RPM_PID and gear_rpm.
    RPM_PID = RPM * 50; // Needs changing
    gear_rpm = RPM_PID;

    // Set the flag to indicate a new byte has arrived.
    gear_rpm_Flag = true;
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

        gear_speed_Flag = true; 

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

void nvs_defaults(const char *Namespace)
{
    nvs_handle my_handle;
    esp_err_t err = nvs_open(Namespace, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGI("NVS", "Namespace not found, creating it with default values.");
        }
        else
        {
            ESP_LOGE("NVS", "Error (%s) opening NVS handle!", esp_err_to_name(err));
            return;
        }
    }

    // Simplify the setting of default values and error handling
    if ((err = nvs_set_i32(my_handle, "default_int", 123)) != ESP_OK ||
        (err = nvs_set_str(my_handle, "default_str", "Hello NVS")) != ESP_OK ||
        (err = nvs_commit(my_handle)) != ESP_OK)
    {
        ESP_LOGE("NVS", "Failed to set/commit defaults! Error: %s", esp_err_to_name(err));
    }

    nvs_close(my_handle);
}

// Function to initialize NVS
void nvs_myinit()
{
    const char *Namespace = "storage";
    esp_err_t err = nvs_flash_init();
    // Handle potential need to erase and reinitialize NVS
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t nvs_handle;
    err = nvs_open(Namespace, NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_defaults(Namespace);
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return;
    }

    // Simplified gear value reading logic
    for (int i = 0; i < MAX_STORED_GEARS; i++)
    {
        uint16_t gear_value;
        err = nvs_get_u16(nvs_handle, std::to_string(i).c_str(), &gear_value);
        if (err == ESP_OK)
        {
            NVS_stored_gear_buffer[i] = gear_value;
            ESP_LOGI("NVS", "Read gear value %d from NVS and appended to the buffer at index %d", gear_value, i);
        }
        else if (err != ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGE("NVS", "Error (%s) reading gear value from NVS at index %d", esp_err_to_name(err), i);
        }
    }

    nvs_close(nvs_handle);
}

void CalculateGear(uint16_t currentSpeed, uint16_t currentRPM)
{
    if (gear_speed_Flag && gear_rpm_Flag && CalculateGear_Flag && !GearsAreCalc)
    {
        UpdateRatioBuffer(currentSpeed, currentRPM);
        if (ratio_buffer.size() >= Gear_Vector_Size)
        {
            ProcessGearRatio();
            CheckAndHandleGearChange();
            CheckAndFinalizeGearCalculations();
        }
    }
}

void UpdateRatioBuffer(uint16_t currentSpeed, uint16_t currentRPM)
{
    gear_speed = currentSpeed;
    gear_rpm = currentRPM;
    gear_Current_Ratio = static_cast<float>(currentSpeed) / currentRPM;
    ratio_buffer.push_back(gear_Current_Ratio);
}

void ProcessGearRatio()
{
    auto gearRatio = CalculateMostFrequentRatio();
    StoreGearRatio(gearRatio);
}

uint16_t CalculateMostFrequentRatio()
{
    std::unordered_map<uint16_t, int> frequencyMap;
    uint16_t gearRatio = 0;
    int maxFrequency = 0;
    for (const auto &value : ratio_buffer)
    {
        frequencyMap[static_cast<uint16_t>(value)]++;
    }
    for (const auto &pair : frequencyMap)
    {
        if (pair.second > maxFrequency)
        {
            maxFrequency = pair.second;
            gearRatio = pair.first;
        }
    }
    return gearRatio;
}

void StoreGearRatio(uint16_t gearRatio)
{
    current_gear_buffer[current_gear].push_back(gearRatio);
    last_ratio[current_gear] = gearRatio;
    std::stringstream msgStream;
    msgStream << "Stored gear ratio " << gearRatio << " in current_gear_buffer at index " << current_gear_buffer[current_gear].size() - 1;
    sendResponse(msgStream.str());
}

void CheckAndHandleGearChange()
{
    if (std::abs(gear_Current_Ratio - last_ratio[current_gear]) > Ratio_Threshold)
    {
        current_gear = (current_gear + 1) % Gear_max;
        ratio_buffer.clear();
    }
    if (current_gear == 4 && !GearIs5)
    {
        gear5_timer_start = esp_timer_get_time();
    }
}

void CheckAndFinalizeGearCalculations()
{
    if (current_gear == 5)
    {
        if (!GearIs5 || (esp_timer_get_time() - gear5_timer_start) >= Gear5_Timer)
        {
            GearsAreCalc = true;
            WriteCurrentGearToNVS();
            CalculateGear_Flag = false;
        }
    }
}

void WriteCurrentGearToNVS()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK)
    {
        err = nvs_set_u8(nvs_handle, "current_gear", current_gear);
        if (err == ESP_OK)
        {
            err = nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    }
    if (err != ESP_OK)
    {
        sendResponse("Error writing to NVS!");
    }
}

void updateGearPID()
{
    Gear_PID = 0; // Default to 0 indicating no match found or not read

    for (size_t i = 0; i < 6; ++i)
    {
        if (gear_Current_Ratio == NVS_stored_gear_buffer[i])
        {
            Gear_PID = i + 1; // Set Gear_PID to the matched index + 1
            break;            // Exit the loop on finding a match
        }
    }
}

void maximumSpeed() {
    static byte topSpeed = 10; // Static variable initialization
    if (Speed_PID > topSpeed) {
        topSpeed = Speed_PID; // Update if the current Speed_PID is greater
        Max_Speed_PID = topSpeed;
    }
}

void updateMcuPidValues()
{
    static unsigned long lastUpdateTime = 0;
    const unsigned long updateInterval = 1000;        // Interval in microseconds
    unsigned long currentTime = esp_timer_get_time(); // Get current time in microseconds

    // Check if the update interval has passed
    if ((currentTime - lastUpdateTime) >= updateInterval)
    {
        // Update the last update time to the current time
        lastUpdateTime = currentTime;

        // Update uptime PID value, ensuring it wraps around to avoid overflow
        unsigned long uptimeSeconds = currentTime / 1000000; // Convert microseconds to seconds
        MCU_Uptime_PID = uptimeSeconds % 65535;

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
        Serial.print("RPM: ");
        Serial.println(RPM_PID);
        Serial.print("Vehicle Speed: ");
        Serial.println(Speed_PID);
        Serial.print("Current Gear: ");
        Serial.println(Gear_PID);
        Serial.print("Coolant Temp: ");
        Serial.println(Coolant_PID);
        Serial.print("Error Code: ");
        Serial.println(Error_PID);
        Serial.print("MCU Temp: ");
        Serial.println(Temp_PID);
        Serial.print("CPU Mhz: ");
        Serial.println(CPU_PID);
        Serial.print("Ram Free: ");
        Serial.println(RAM_Free_PID);
        Serial.print("Max Speed: ");
        Serial.println(Max_Speed_PID);
        Serial.print("MCU Uptime Seconds:");
        Serial.println(MCU_Uptime_PID);
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
