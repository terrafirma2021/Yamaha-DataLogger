#include <Arduino.h>
#include <handlecommand.h>
#include <BLE.h>
#include <vector>
#include <unordered_map>
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

// Constants for L9637D pins
#define YAM_TX 12
#define YAM_RX 33

// Time thresholds and timeouts
const unsigned long FRAME_END_THRESHOLD_TIMER = 3000; // 5 milliseconds in microseconds
const unsigned long DIAG_START_TIMEOUT_TIMER = 2000000;   // 2 seconds in microseconds
const unsigned long BIKE_OFF_TIMEOUT_TIMER = 9000000;  // 5 seconds in microseconds

// Magic numbers
const byte IMMO_START_BYTE = 0x3E;
const byte DIAG_START_BYTE = 0xCD;
const byte NORMAL_OPERATION = 0xFE;

// Debugging
#define DEBUG_LEVEL 0

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
auto lastByteTime = 0;

// Communication flags
bool isIMMOHandled = false;
bool isIMMOInProgress = false;
bool DiagIsStarting = false;
bool NormalOperation = false;
bool frameEndDetected = false;

// Function declarations
void setup();
void loop();
void readSerialData();
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

// Calculate Gear Constants
const uint16_t Gear_Vector_Size = 333;       // 15*333 = 4995 ms (Gear detection time)
const uint8_t Gear_max = 6;                  // Maximum number of gears
const uint8_t Neutral_Threshold = 10;        // Tolerance threshold for neutral detection (Speed) 
const uint8_t Ratio_Threshold = 10;          // Threshold for detecting gear change based on ratio difference (Need to Tune)
const uint32_t Gear5_Timer = 9000000;        // Timer duration for determining if the bike has 5 gears (9 seconds in microseconds)

// Calculate Gear variables
uint8_t current_gear = 0;
uint16_t gear_Current_Ratio = 0;

// Calculate Gear Flags
bool gear_rpm_Flag = false;
bool gear_speed_Flag = false;
bool GearIs5 = false;
bool GearsAreCalc = false;

// Gear Calculation
byte gear_speed = 0;
byte gear_rpm = 0;

// PIDS
uint16_t RPM_PID;    // RPM * 50 = RAW
uint8_t Coolant_PID; // Temp = RAW
uint8_t Speed_PID;   // RAW km/h
uint8_t Gear_PID;    // RAW 00-05
uint8_t Error_PID;   // Error code

// Calculate Gear Buffers
std::vector<uint16_t> ratio_buffer;
std::vector<std::vector<uint16_t>> current_gear_buffer(Gear_max);
std::vector<uint16_t> last_ratio(Gear_max, 0);
uint64_t gear5_timer_start = 0;

// NVS related variables
uint16_t stored_gear_buffer[] = {6};
bool nvs_data_read = false;

// Calulate Gear Functions
void init_nvs();
void CalculateGear(uint16_t currentSpeed, uint16_t currentRpm);
void handleCurrentGearPID(bool nvs_data_read);


void setup()
{
    delay(1000);
    Serial.begin(115200);
    Serial1.begin(16040, SERIAL_8N1, YAM_RX, YAM_TX);
    Serial.println("Yamaha ELM327 Datalogger");
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
    readSerialData();
}

void readSerialData()
{
    if (Serial1.available())
    {
        byte incomingByte = Serial1.read();
        processSerialByte(incomingByte);

        // Update the timestamp of the last received byte
        lastByteTime = esp_timer_get_time();
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
            isIMMOHandled = true;     // Set IMMO handled if diagnostic menu starts
            isIMMOInProgress = false; // Reset IMMO in progress flag
        }
        else if (lastImmoByte == NORMAL_OPERATION)
        {
            NormalOperation = true;
            Serial.println("Normal Operation Detected");
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

    if (DEBUG_LEVEL >= 1)
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
    }
}

void handleBikeOffCondition()
{
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
            bool isIMMOHandled = false;
            bool isIMMOInProgress = false;
            bool DiagIsStarting = false;
            bool NormalOperation = false;
            bool frameEndDetected = false;
            ECUBufferIndex = 0;
            lastByteTime = 0;

            // Print message indicating bike off condition detected
            Serial.println("Bike Off Detected");
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

    //Set Flag to tell Calculate Gear function new byte has arrived
    bool gear_rpm_Flag = true;
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

        // Write the Speed_PID to the gear_speed buffer
        gear_speed = Speed_PID;

        // Set calculateGear_Speed buffer equal to Speed_PID
        gear_speed = gear_speed;

        // set calculateGear_Speed Flag true
        bool gear_speed_Flag = true;

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

// Function to initialize NVS
void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

// Function to calculate gear
void CalculateGear(uint16_t currentSpeed, uint16_t currentRpm) {
    // Check if both gear_speed and gear_rpm are set before processing
    if (gear_speed_Flag && gear_rpm_Flag && !GearsAreCalc) {
        // Set the current speed and RPM
        gear_speed = currentSpeed;
        gear_rpm = currentRpm;

        // Calculate the ratio of speed to RPM
        gear_Current_Ratio = currentSpeed / currentRpm;

        // Write the ratio to the buffer
        ratio_buffer.push_back(gear_Current_Ratio);

        // Check if the buffer is full
        if (ratio_buffer.size() >= Gear_Vector_Size) {
            // Calculate the gear ratio for the current gear
            std::unordered_map<uint16_t, int> frequencyMap;
            uint16_t gearRatio = 0;
            int maxFrequency = 0;

            // Count frequency of each value in the dataset
            for (const auto& value : ratio_buffer) {
                frequencyMap[value]++;
            }

            // Find the value with the highest frequency (mode)
            for (const auto& pair : frequencyMap) {
                if (pair.second > maxFrequency) {
                    maxFrequency = pair.second;
                    gearRatio = pair.first;
                }
            }

            // Store the gear ratio in the current_gear_buffer for the current gear
            current_gear_buffer[current_gear].push_back(gearRatio);
            last_ratio[current_gear] = gearRatio;

            // Check for gear change based on ratio difference
            if (std::abs(gear_Current_Ratio - last_ratio[current_gear]) > Ratio_Threshold) {
                // Move to the next gear
                current_gear = (current_gear + 1) % Gear_max;
                // Clear the ratio buffer for the next gear
                ratio_buffer.clear();
            }

            // Start the timer for 5th gear if the 5th gear is reached
            if (current_gear == 4 && !GearIs5) {
                gear5_timer_start = esp_timer_get_time();
            }

            // Check if gear information has been calculated for all gears
            if (current_gear == 5) {
                // If the buffer reaches 6th gear before the timer expires or if the timer expires before the buffer reaches 6th gear, set the flag
                if (!GearIs5 || (esp_timer_get_time() - gear5_timer_start) >= Gear5_Timer) {
                    GearsAreCalc = true;

                    // Write current gear to NVS
                    if (!nvs_data_read) {
                        nvs_handle_t nvs_handle;
                        esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
                        if (err != ESP_OK) {
                            printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
                            return;
                        }

                        // Write current_gear value to NVS
                        err = nvs_set_u8(nvs_handle, "current_gear", current_gear);
                        if (err != ESP_OK) {
                            printf("Error (%s) writing data to NVS!\n", esp_err_to_name(err));
                            nvs_close(nvs_handle);
                            return;
                        }

                        // Commit the value written to NVS
                        err = nvs_commit(nvs_handle);
                        if (err != ESP_OK) {
                            printf("Error (%s) committing data to NVS!\n", esp_err_to_name(err));
                        }

                        // Close NVS
                        nvs_close(nvs_handle);

                        // Update the flag after writing to NVS
                        nvs_data_read = true;
                    }
                }
            }
        }
    }
}

// Function to handle current gear PID
void handleCurrentGearPID(bool nvs_data_read) {
    if (nvs_data_read) {
        // Compare gear_Current_Ratio with stored gear ratio
        for (size_t i = 0; i < 6; ++i) { // Using the known size of the array (6)
            if (gear_Current_Ratio == stored_gear_buffer[i]) {
                // Set GEAR_PID to the index number of the matched value inside the stored_gear_buffer
                Gear_PID = i;
                return; // Exit the loop once a match is found
            }
        }
        // If no match is found, set GEAR_PID to 0
        Gear_PID = 0;
    }
}
