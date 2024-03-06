#include <Arduino.h>
#include <handlecommand.h>
#include <BLE.h>

// Constants for L9637D pins
#define YAM_TX 12
#define YAM_RX 33

// Time thresholds and timeouts
const unsigned long FRAME_END_THRESHOLD_TIMER = 3000; // 5 milliseconds in microseconds
const unsigned long DIAG_START_TIMEOUT_TIMER = 2000000;   // 2 seconds
const unsigned long BIKE_OFF_TIMEOUT_TIMER = 9000000;  // 5 seconds in microseconds

// Magic numbers
const byte IMMO_START_BYTE = 0x3E;
const byte DIAG_START_BYTE = 0xCD;
const byte NORMAL_OPERATION = 0xFE;

// Debugging
#define DEBUG_LEVEL 0

// Gear detection constants
uint8_t Gear1Constant = 61;
uint8_t Gear2Constant = 90;
uint8_t Gear3Constant = 129;
uint8_t Gear4Constant = 161;
uint8_t Gear5Constant = 192;
uint8_t Tolerance = 1;

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
void calculateGear();

void setup()
{
    delay(1000);
    Serial.begin(115200);
    Serial1.begin(16040, SERIAL_8N1, YAM_RX, YAM_TX);
    Serial.println("Yamaha ESP32 S3 BLE  elm327 Emulator");
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
    calculateGear();

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

        // Reset the buffer index for the next set of data
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

void calculateGear()
{
    // Check if either Speed_PID or RPM_PID is zero
    if (Speed_PID == 0 || RPM_PID == 0)
    {
        Gear_PID = 0x00; // Neutral gear if speed or RPM is zero
        return;
    }

    // Calculate the CurrentGear based on Speed and RPM
    uint16_t CurrentGear = RPM_PID / Speed_PID;

    // Determine the gear based on CurrentGear and gear variations
    Gear_PID = 0x00; // Default to an undefined gear

    if (abs(CurrentGear - Gear1Constant) <= Tolerance)
    {
        Gear_PID = 0x01; // Vehicle is in 1st gear
    }
    else if (abs(CurrentGear - Gear2Constant) <= Tolerance)
    {
        Gear_PID = 0x02; // Vehicle is in 2nd gear
    }
    else if (abs(CurrentGear - Gear3Constant) <= Tolerance)
    {
        Gear_PID = 0x03; // Vehicle is in 3rd gear
    }
    else if (abs(CurrentGear - Gear4Constant) <= Tolerance)
    {
        Gear_PID = 0x04; // Vehicle is in 4th gear
    }
    else if (abs(CurrentGear - Gear5Constant) <= Tolerance)
    {
        Gear_PID = 0x05; // Vehicle is in 5th gear
    }
    else
    {
        Gear_PID = 0x00; // Neutral gear as a failsafe
    }
}

