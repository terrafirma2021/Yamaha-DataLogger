#include <Arduino.h>
#include <handlecommand.h>
#include <BLE.h>
#include <vector>
#include <unordered_map>
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <LCD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2lib.h>
#include <iostream>
#include <string>

// Define L9637D Pins
#define YAM_TX 1
#define YAM_RX 38

// Debug Test Pins for L9637D
// #define YAM_TX 12
// #define YAM_RX 33

// Debug Level
extern bool Debug_RX;
extern bool Debug_TX;
extern bool Debug_PIDS;
bool DisableBikeOff_Flag = false;

//BLE Connected bool
extern bool clientConnected;


//Watchdog timer Fix
bool isTempReady = false;

// Time thresholds and timeouts
const unsigned long FRAME_END_THRESHOLD_TIMER = 3000;   // 5 milliseconds in microseconds
const unsigned long DIAG_START_TIMEOUT_TIMER = 2000000; // 2 seconds in microseconds
const unsigned long BIKE_OFF_TIMEOUT_TIMER = 9000000;   // 5 seconds in microseconds

// Magic numbers
const byte IMMO_START_BYTE = 0x3E;
const byte DIAG_START_BYTE = 0xCD;
const byte NORMAL_OPERATION = 0xFE;

// Buffer sizes
#define VEHICLE_SPEED_RAW_BUFFER_SIZE 8
#define ECU_BUFFER_SIZE 5
#define IMMO_BUFFER_SIZE 55

// Buffers
byte Vehicle_Speed_Raw_Buffer[VEHICLE_SPEED_RAW_BUFFER_SIZE];
byte ECU_Buffer[ECU_BUFFER_SIZE];
byte IMMO_Buffer[IMMO_BUFFER_SIZE];

// Buffer indices and time variables
byte VehicleSpeedRawBufferIndex = 0;
byte ECUBufferIndex = 0;
byte IMMOIndex = 0;

// Communication flags
bool isIMMOHandled = false;
bool isIMMOInProgress = false;
bool DiagIsStarting = false;
bool NormalOperation = false;
bool frameEndDetected = false;

// Function declarations
void setup();
void loop();
void YamahaInbound();
void serialTerminal();
void processSerialByte(byte incomingByte);
void handleIMMOSequence(byte incomingByte);
void handleDiagStart(byte incomingByte);
void HandleNormalOperation(byte incomingByte);
void handleBikeOffCondition();
void processECUData();
void calculateRPM();
void calculateVehicleSpeed();
void extractErrorCode();
void calculateCoolantTemp();
void CalculateGear(uint16_t currentSpeed, uint16_t currentRpm);
void handleCurrentGearPID(bool nvs_data_read);
void nvs_myinit();
void MCU_PIDS();
void maximumSpeed();
void sendUart(std::string msg);
void DebugPIDS();

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
uint16_t RPM_PID;       // RPM * 50 = RAW
uint8_t Speed_PID;      // RAW km/h
uint8_t Coolant_PID;    // Temp = RAW
uint8_t Error_PID;      // Error code
uint8_t Gear_PID;       // RAW 00-05
uint8_t Temp_PID;      // MCU Temp
uint8_t CPU_PID;        // CPU Freq mhz
uint8_t RAM_Free_PID;  // Free Ram
uint8_t Max_Speed_PID;  // Max Speed Reached

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
    u8g2.begin();
    MyCallbacks *myCallbacks = new MyCallbacks();
    bool success = Device::getInstance().start(myCallbacks);
    if (!success)
    {
        Serial.println("Failed to start BLE");
    }
}

void loop()
{
    handleBikeOffCondition();
    YamahaInbound();
    displayData();
    serialTerminal();
    DebugPIDS();
    MCU_PIDS();
}

void YamahaInbound()
{
    if (Serial1.available())
    {
        byte incomingByte = Serial1.read();
        processSerialByte(incomingByte);

        // Update the timestamp of the last received byte
        lastByteTime = esp_timer_get_time();
    }
}

void serialTerminal() 
{
    // Check if serial data is available
    if (Serial.available()) 
    {
        String input = Serial.readStringUntil('\n');
        input.toUpperCase();

        if (input.equals("DEBUG OFF") || input.equals("DEBUG 0")) 
        {
            (Serial.println("Command Received: Debug Off"));
            Debug_RX = false;
            Debug_TX = false;
            Debug_PIDS = false;
            DisableBikeOff_Flag = false;
        } 
        else if (input.equals("DEBUG RX")) 
        {
            (Serial.println("Command Received: Debug RX"));
            Debug_RX = true;
            Debug_TX = false;
            Debug_PIDS = false;
        } 
        else if (input.equals("DEBUG TX")) 
        {
            (Serial.println("Command Received: Debug TX"));
            Debug_RX = false;
            Debug_TX = true;
            Debug_PIDS = false;
            
        } 
        else if (input.equals("DEBUG PIDS")) 
        {
            (Serial.println("Command Received: Debug PIDS"));
            Debug_RX = false;
            Debug_TX = false;
            Debug_PIDS = true;
        } 
        else if (input.equals("BIKE ON")) 
        {
            (Serial.println("command Received: Bike timer disabled"));
            DisableBikeOff_Flag = true;
        }
        else if (input.equals("RESET")) 
        {
            (Serial.println("command Received: Bye!"));
            ESP.restart();
        }
    }
}

// Process each byte received from the serial port
void processSerialByte(byte incomingByte)
{
    // First handle any ongoing IMMO sequence
    if (!isIMMOHandled)
    {
        handleIMMOSequence(incomingByte);
    }
    // Next Handle DiagStart sequence
    if (!DiagIsStarting && incomingByte == DIAG_START_BYTE)
    {
        handleDiagStart(incomingByte);
    }
    // Proceed only if NormalOperation is flagged true
    if (NormalOperation)
    {
        HandleNormalOperation(incomingByte);
    }
}

void handleIMMOSequence(byte incomingByte)
{
    // Check if the incoming byte is the start of the IMMO sequence
    if (incomingByte == IMMO_START_BYTE && !isIMMOInProgress)
    {
        Serial.println("IMMO Start Detected, Starting IMMO Sequence");
        std::string msg = "IMMO Start Detected, Starting IMMO Sequence";
        sendUart(msg);
        isIMMOInProgress = true; // Set flag to indicate IMMO sequence is in progress
        IMMOIndex = 0;           // Reset the buffer index
    }

    // Process IMMO bytes if the IMMO sequence is in progress
    if (isIMMOInProgress)
    {
        // Add the incoming byte to the buffer
        IMMO_Buffer[IMMOIndex] = incomingByte;
        // Increment the buffer index after storing the byte
        IMMOIndex++;

        // Check if the IMMO buffer is full
        if (IMMOIndex >= IMMO_BUFFER_SIZE)
        {
            isIMMOHandled = true;     // Set IMMO handled flag
            isIMMOInProgress = false; // Reset IMMO in progress flag
        }

        // Check the last byte in IMMO_Buffer
        byte lastImmoByte = IMMO_Buffer[IMMOIndex - 1];
        if (lastImmoByte == DIAG_START_BYTE)
        {
            DiagIsStarting = true;
            Serial.println("Diagnostic Menu Starting");
            std::string msg = "Diagnostic Menu Starting";
            sendUart(msg);
            isIMMOHandled = true;     // Set IMMO handled if diagnostic menu starts
            isIMMOInProgress = false; // Reset IMMO in progress flag
        }
        else if (lastImmoByte == NORMAL_OPERATION)
        {
            NormalOperation = true;
            Serial.println("Normal Operation Detected");
            std::string msg = "Normal Operation Detected";
            sendUart(msg);
            isIMMOHandled = true;     // Set IMMO handled if normal operation starts
            isIMMOInProgress = false; // Reset IMMO in progress flag
        }
    }
}

void handleDiagStart(byte incomingByte)
{
    // Get the current time in microseconds
    auto currentTime = esp_timer_get_time();

    // Calculate the time elapsed since the last byte was received
    auto timeElapsed = currentTime - lastByteTime;

    // Check if it's time to consider the DiagStart sequence
    if (DiagIsStarting && timeElapsed > BIKE_OFF_TIMEOUT_TIMER)
    {
        // Print message indicating DIAG Menu initialization
        Serial.println("DIAG Menu init");
        std::string msg = "DIAG Menu init";
        sendUart(msg);
    }
}

void HandleNormalOperation(byte incomingByte)
{
    // Fill the buffer with incoming bytes
    ECU_Buffer[ECUBufferIndex++] = incomingByte;

    // Check if the buffer contains at least 5 bytes
    if (ECUBufferIndex == ECU_BUFFER_SIZE)
    {
        // Calculate the checksum by summing the first 4 bytes in the buffer
        byte checksum = ECU_Buffer[0] + ECU_Buffer[1] + ECU_Buffer[2] + ECU_Buffer[3];

        // Compare the calculated checksum with the checksum byte (5th byte) in the buffer
        if (checksum == ECU_Buffer[4])
        {
            // If the checksum is valid, process the ECU data
            processECUData();

            // Reset the buffer index to 0 for the next frame
            ECUBufferIndex = 0;
        }
        else
        {
            // If the checksum is not valid
            // Shift bytes to the left to ensure the VehicleSpeed has new data
            for (int i = 0; i < 4; i++)
            {
                ECU_Buffer[i] = ECU_Buffer[i + 1];
            }
            // Decrement the buffer index after shifting
            ECUBufferIndex--;
        }
    }
}

void processECUData()
{
    calculateRPM();
    calculateVehicleSpeed();
    extractErrorCode();
    calculateCoolantTemp();
    handleCurrentGearPID(nvs_data_read);
}

void handleBikeOffCondition()
{
    // Check if the bike off condition is disabled
    if (DisableBikeOff_Flag)
        return;

    // Check if lastByteTime is not 0
    if (lastByteTime != 0)
    {
        // Get the current time in microseconds
        auto currentTime = esp_timer_get_time();

        // Calculate the time elapsed since the last byte was received
        auto timeElapsed = currentTime - lastByteTime;
        // Check if it's time to consider the bike off condition
        if (timeElapsed > BIKE_OFF_TIMEOUT_TIMER)

        {
            // Reset flags and variables related to bike off condition
            isIMMOHandled = false;
            isIMMOInProgress = false;
            DiagIsStarting = false;
            NormalOperation = false;
            frameEndDetected = false;

            ECUBufferIndex = 0;
            lastByteTime = 0;

            // Print message indicating bike off condition detected
            Serial.println("Bike Off Detected");

            std::string msg = "Bike Off Detected";
            sendUart(msg);
        }
    }
}

void calculateRPM()
{
    // Extract RPM data from the ECU buffer
    uint16_t RPM = ECU_Buffer[0];

    // Adjust RPM using the multiplication factor
    RPM *= 50; // Adjusted ECU multiplication factor

    // Assign the adjusted RPM directly to RPM_PID
    RPM_PID = RPM;

    // Assign the value to the calculateGear_RPM
    gear_rpm = RPM;

    // Set Flag to tell Calculate Gear function new byte has arrived
    gear_rpm_Flag = true;
}

void calculateVehicleSpeed()
{
    // Store the byte in the buffer
    Vehicle_Speed_Raw_Buffer[VehicleSpeedRawBufferIndex++] = ECU_Buffer[1];

    // Check if the buffer is full
    if (VehicleSpeedRawBufferIndex == VEHICLE_SPEED_RAW_BUFFER_SIZE)
    {
        int totalSpeed = 0; // Renamed variable for clarity
        for (int i = 0; i < VEHICLE_SPEED_RAW_BUFFER_SIZE; ++i)
        {
            totalSpeed += Vehicle_Speed_Raw_Buffer[i]; // Accumulate speed data
        }

        // Store the total speed directly in Speed_PID
        Speed_PID = totalSpeed;

        // write the Speed For Maxmium Speed Function
        MaxSpeed = totalSpeed;

        // Write the Speed_PID to the gear_speed buffer
        gear_speed = Speed_PID;

        // Set calculateGear_Speed buffer equal to Speed_PID
        gear_speed = gear_speed;

        // set calculateGear_Speed Flag true
        gear_speed_Flag = true;

        // Reset the buffer index to 0 for the next frame
        VehicleSpeedRawBufferIndex = 0;
    }
}

void extractErrorCode()
{
    // Extract the Error Code from the ECU data
    uint8_t Error = ECU_Buffer[2];
    Error_PID = Error;
}

void calculateCoolantTemp()
{
    uint8_t coolantTemp = ECU_Buffer[3];

    // Store the result in Coolant_PID
    Coolant_PID = coolantTemp;
}

void nvs_defaults(const char *mynamespace) {
    nvs_handle my_handle;
    esp_err_t err;

    // Try to open the namespace in NVS_READWRITE mode.
    err = nvs_open(mynamespace, NVS_READWRITE, &my_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // If the namespace was not found, it needs to be created.
        // Note: nvs_open does not explicitly create a namespace if it doesn't exist.
        // The namespace is created implicitly when a key-value pair is set.
        ESP_LOGI("NVS", "Namespace not found, creating it with default values.");
    } else if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return;
    }

    // At this point, the namespace is open and we can set default values.

    // Set default integer value
    err = nvs_set_i32(my_handle, "default_int", 123);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to set default_int!");
    }

    // Set default string value
    err = nvs_set_str(my_handle, "default_str", "Hello NVS");
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to set default_str!");
    }

    // After setting your defaults, commit them to make sure they are saved.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to commit defaults!");
    }

    // Always close the NVS handle when done to free resources.
    // nvs_close(my_handle);
}

// Function to initialize NVS
void nvs_myinit()
{
    const char *mynamespace = "storage";

    // Initialize NVS flash
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS storage named "storage"
    nvs_handle_t nvs_handle;
    err = nvs_open(mynamespace, NVS_READONLY, &nvs_handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_defaults(mynamespace);
    }
    else if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        std::string err1 = std::string(esp_err_to_name(err));
        std::string msg = "Error (%s) committing data to NVS!\n" + err1;
        sendUart(msg);
        return;
    }

    // Read each stored gear value from NVS and append it to the buffer
    for (int i = 0; i < MAX_STORED_GEARS; i++)
    {
        const char *const key = std::to_string(i).c_str();

        uint16_t gear_value;
        err = nvs_get_u16(nvs_handle, key, &gear_value);

        if (err == ESP_OK)
        {
            NVS_stored_gear_buffer[i] = gear_value;
            printf("Read gear value %d from NVS and appended to the buffer at index %d\n", gear_value, i);
            std::string Geara = std::string(gear_value, i);
            std::string msg = "Read gear value %d from NVS and appended to the buffer at index %d\n" + Geara;
            NVS_index_current_gear++; // Update the NVS index current gear
            sendUart(msg);
        }
        else if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            printf("No gear value found at index %d in NVS\n", i);
            std::string msg = "No gear value found at index %d in NVS"; // err add index
            sendUart(msg);
        }
        else
        {
            printf("Error (%s) reading gear value from NVS at index %d\n", esp_err_to_name(err), i);
            std::string err1 = std::string(esp_err_to_name(err));
            std::string msg = "Error (%s) reading gear value from NVS at index %d\n" + err1;
            sendUart(msg);
        }
    }

    // Close NVS
    nvs_close(nvs_handle);
}

// Function to calculate gear
void CalculateGear(uint16_t currentSpeed, uint16_t currentRpm)
{
    // BLE Serial request to start gear ratio tuning, check if both gear_speed and gear_rpm are set before processing
    if (gear_speed_Flag && gear_rpm_Flag && CalculateGear_Flag && !GearsAreCalc)
    {

        // Set the current speed and RPM
        gear_speed = currentSpeed;
        gear_rpm = currentRpm;

        // Calculate the ratio of speed to RPM
        gear_Current_Ratio = currentSpeed / currentRpm;

        // Write the ratio to the buffer
        ratio_buffer.push_back(gear_Current_Ratio);

        // Check if the buffer is full
        if (ratio_buffer.size() >= Gear_Vector_Size)
        {
            // Calculate the gear ratio for the current gear
            std::unordered_map<uint16_t, int> frequencyMap;
            uint16_t gearRatio = 0;
            int maxFrequency = 0;

            // Count frequency of each value in the dataset
            for (const auto &value : ratio_buffer)
            {
                frequencyMap[value]++;
            }

            // Find the value with the highest frequency (mode)
            for (const auto &pair : frequencyMap)
            {
                if (pair.second > maxFrequency)
                {
                    maxFrequency = pair.second;
                    gearRatio = pair.first;
                }
            }

            // Store the gear ratio in the current_gear_buffer for the current gear
            current_gear_buffer[current_gear].push_back(gearRatio);
            last_ratio[current_gear] = gearRatio;

            // Serial print to indicate the index and value written
            Serial.print("Stored gear ratio ");
            Serial.print(gearRatio);
            Serial.print(" in current_gear_buffer at index ");
            Serial.println(current_gear_buffer[current_gear].size() - 1);
            // std::string Geara = std::string(current_gear_buffer[current_gear].size() - 1);
            // std::string msg = "Stored gear ratio\n" + Geara;
            // Device::getInstance().sendUart(msg);

            // Check for gear change based on ratio difference
            if (std::abs(gear_Current_Ratio - last_ratio[current_gear]) > Ratio_Threshold)
            {
                // Move to the next gear
                current_gear = (current_gear + 1) % Gear_max;
                // Clear the ratio buffer for the next gear
                ratio_buffer.clear();
            }

            // Start the timer for 5th gear if the 5th gear is reached
            if (current_gear == 4 && !GearIs5)
            {
                gear5_timer_start = esp_timer_get_time();
            }

            // Check if gear information has been calculated for all gears
            if (current_gear == 5)
            {
                // If the buffer reaches 6th gear before the timer expires or if the timer expires before the buffer reaches 6th gear, set the flag
                if (!GearIs5 || (esp_timer_get_time() - gear5_timer_start) >= Gear5_Timer)
                {
                    GearsAreCalc = true;

                    // Write current gear to NVS
                    if (!nvs_data_read)
                    {
                        nvs_handle_t nvs_handle;
                        esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
                        std::string err1 = std::string(esp_err_t());
                        if (err != ESP_OK)
                        {
                            printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
                            std::string err1 = std::string(esp_err_to_name(err));
                            std::string msg = "Error (%s) opening NVS handle!\n" + err1;
                            sendUart(msg);
                            return;
                        }

                        // Write current_gear value to NVS
                        err = nvs_set_u8(nvs_handle, "current_gear", current_gear);
                        if (err != ESP_OK)
                        {
                            printf("Error (%s) writing data to NVS!\n", esp_err_to_name(err));
                            std::string err1 = std::string(esp_err_to_name(err));
                            std::string msg = "Error (%s) writing data to NVS!\n" + err1;
                            sendUart(msg);
                            nvs_close(nvs_handle);
                            return;
                        }

                        // Commit the value written to NVS
                        err = nvs_commit(nvs_handle);
                        if (err != ESP_OK)
                        {
                            printf("Error (%s) committing data to NVS!\n", esp_err_to_name(err));
                            std::string err1 = std::string(esp_err_to_name(err));
                            std::string msg = "Error (%s) committing data to NVS!\n" + err1;
                            sendUart(msg);
                        }

                        // Close NVS
                        nvs_close(nvs_handle);

                        // Update the flag after writing to NVS
                        nvs_data_read = true;

                        // set Flags False
                        CalculateGear_Flag = false;
                    }
                }
            }
        }
    }
}

// Function to handle current gear PID using the contents of the NVS_stored_gear_buffer
void handleCurrentGearPID(bool nvs_data_read)
{
    if (nvs_data_read)
    {
        // Compare gear_Current_Ratio with stored gear ratio
        for (size_t i = 0; i < 6; ++i)
        { // Using the known size of the array (6)
            if (gear_Current_Ratio == NVS_stored_gear_buffer[i])
            {
                // Set GEAR_PID to the index number of the matched value inside the stored_gear_buffer
                Gear_PID = i + 1; // Bump the index value by + 1
                return;           // Exit the loop once a match is found
            }
        }
        // If no match is found, set GEAR_PID to 0 // Need to implement smoothing ideally to return less false neutral (TO DO!)
        Gear_PID = 0;
    }
}

void maximumSpeed()
{
    byte topSpeed = 0;

    if (MaxSpeed > topSpeed)
    {
        topSpeed = MaxSpeed;
    }

    Max_Speed_PID = topSpeed;
}

void MCU_PIDS()
{
    static unsigned long previousTime = 0;
    const long MCUTimer = 1000; // 1 Milisecond to cure Watchdog timer bug (Could be set to less frequent updates if needed)
    float temperature;
    unsigned long currentTime = esp_timer_get_time();
    
    // Check if it's time to update all PID values
    if (currentTime - previousTime >= MCUTimer) {
        previousTime = currentTime;
        
        temperature = temperatureRead();       
        Temp_PID = temperature;
        CPU_PID = ESP.getCpuFreqMHz();
        RAM_Free_PID = ESP.getFreeHeap() / 1024;
    }
}

void sendUart(std::string msg)
    {
        Device::getInstance().sendUART(msg);
    }


void DebugPIDS()
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
    }
}
