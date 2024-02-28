#include <Arduino.h>
#include <handlecommand.h>
#include <BLE.h>

// Definition for L9637D pins
#define YAM_TX 12
#define YAM_RX 33 // Left for future use

// Define constants for time thresholds and timeouts
const unsigned long FRAME_END_THRESHOLD = 5;   // Threshold to detect end of a frame (5 ms)
const unsigned long DIAG_START_TIMEOUT = 2000; // Timeout for detecting 0xCD after IMMO sequence (2 seconds)
const unsigned long BIKE_OFF_TIMEOUT = 10000;  // 10 seconds BikeOff timer

// Define constants for magic numbers
const byte IMMO_START_BYTE = 0x3E;
const byte DIAG_START_BYTE = 0xCD;
const byte NORMAL_OPERATION = 0xFE;
const unsigned int IMMO_BUFFER_SIZE = 55; // Handles IMMO, + Switch state operation (last byte)

// Declaing the functions to be used
void readSerialData();
void processSerialByte(byte incomingByte);
void handleIMMOSequence(byte incomingByte);
void handleBikeOffCondition();
void processECUData();
void calculateRPM();
void calculateCoolantTemp();
void calculateVehicleSpeed();
void calculateGear();

// Use the Simple Gear Ratio Calculator on Github to get the values for your bike!

// Define constants for gear detection
uint8_t Gear1Constant = 61;
uint8_t Gear2Constant = 90;
uint8_t Gear3Constant = 129;
uint8_t Gear4Constant = 161;
uint8_t Gear5Constant = 192;
uint8_t Tolerance = 1;

// Define the size of the buffers used to store data
#define VEHICLE_SPEED_RAW_BUFFER_SIZE 8
#define ECU_BUFFER_SIZE 6

// Declare the buffers used to store data
byte Vehicle_Speed_Raw_Buffer[VEHICLE_SPEED_RAW_BUFFER_SIZE];
byte ECU_Buffer[ECU_BUFFER_SIZE];
byte IMMO_Buffer[IMMO_BUFFER_SIZE]; // IMMO sequence buffer

// Declare variables to keep track of buffer indices and time
byte VehicleSpeedRawBufferIndex = 0;
byte ECUBufferIndex = 0; // Index for ECU_Buffer
byte IMMOIndex = 0;      // Index for IMMO_Buffer
unsigned long lastByteTime = 0;
unsigned long immoSeqEndTime = 0;
unsigned long lastDataTime = 0;

// Declare flags to track the state of the communication
bool isIMMOSeq = false;
bool DiagIsStarting = false;
bool NormalOperation = false;
bool isIMMOHandled = false;

// Global variable for enabling debugging
bool enableDebugging = true; // Set to false to disable debugging

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

// Main Serial function
void ReadSerialData()
{
    if (Serial1.available())
    {
        byte incomingByte = Serial1.read();
        processSerialByte(incomingByte);
    }
}

// Function to calculate RPM from the ECU data
void calculateRPM()
{
    uint16_t RPM = ECU_Buffer[0] * 50; // Adjusted multiplication factor
    RPM = RPM / 4;                     // Adjusted division factor
    RPM_PID = RPM;
}

// Function to calculate coolant temperature from the ECU data
void calculateCoolantTemp()
{
    uint8_t coolantTemp = ECU_Buffer[2];

    // Add 40 to coolantTemp
    coolantTemp = coolantTemp + 40;

    // Store the result in Coolant_PID
    Coolant_PID = coolantTemp;
}

// Function to calculate vehicle speed from the ECU data
void calculateVehicleSpeed()
{
    // Store the byte in the buffer
    Vehicle_Speed_Raw_Buffer[VehicleSpeedRawBufferIndex++] = ECU_Buffer[3];
    // Check if the buffer is full
    if (VehicleSpeedRawBufferIndex == VEHICLE_SPEED_RAW_BUFFER_SIZE)
    {
        int sum = 0;
        for (int i = 0; i < VEHICLE_SPEED_RAW_BUFFER_SIZE; ++i)
        {
            sum += Vehicle_Speed_Raw_Buffer[i];
        }

        // Store the sum directly in Speed_PID
        Speed_PID = sum;

        // Reset the buffer index for the next set of data
        VehicleSpeedRawBufferIndex = 0;
    }
}

// Function to calculate gear and convert it to ODB standard
void calculateGear()
{
    // Check if either Speed_PID or RPM_PID is zero
    if (Speed_PID == 0 || RPM_PID == 0)
    {
        Gear_PID = 0; // Neutral gear if speed or RPM is zero
        return;
    }

    // Calculate the CurrentGear based on Speed and RPM
    uint16_t CurrentGear;

    CurrentGear = static_cast<uint16_t>((30 / 1200) * 10000);

    // Determine the gear based on CurrentGear and gear variations
    Gear_PID = 0; // Default to an undefined gear

    if (abs(CurrentGear - Gear1Constant) <= Tolerance)
    {
        Gear_PID = 1; // Vehicle is in 1st gear
    }
    else if (abs(CurrentGear - Gear2Constant) <= Tolerance)
    {
        Gear_PID = 2; // Vehicle is in 2nd gear
    }
    else if (abs(CurrentGear - Gear3Constant) <= Tolerance)
    {
        Gear_PID = 3; // Vehicle is in 3rd gear
    }
    else if (abs(CurrentGear - Gear4Constant) <= Tolerance)
    {
        Gear_PID = 4; // Vehicle is in 4th gear
    }
    else if (abs(CurrentGear - Gear5Constant) <= Tolerance)
    {
        Gear_PID = 5; // Vehicle is in 5th gear
    }
    else
    {
        Gear_PID = 0; // Neutral gear as a failsafe
    }

    // Convert Gear_pid to ODB standard
    Gear_PID = Gear_PID * 1000;
}

// Function to process the ECU data
void processECUData()
{
    calculateRPM();
    calculateCoolantTemp();
    calculateVehicleSpeed();

    if (enableDebugging)
    {
        Serial.print("RPM: ");
        Serial.println(RPM_PID);
        Serial.print("Coolant Temp: ");
        Serial.println(Coolant_PID);
        Serial.print("Vehicle Speed: ");
        Serial.println(Speed_PID);
    }
}

// Function to handle the IMMO sequence
void handleIMMOSequence(byte incomingByte)
{
    // Check if the IMMO sequence has not been handled yet
    if (!isIMMOHandled)
    {
        // Check if the incoming byte matches the IMMO start byte
        if (incomingByte == IMMO_START_BYTE)
        {
            Serial.println("IMMO Start Detected, Starting IMMO Sequence");
            IMMOIndex = 0;
            isIMMOSeq = true;
            isIMMOHandled = true; // Mark the IMMO sequence as handled
        }
        else
        {
            // Ignore junk bytes before IMMO Start by returning early
            return;
        }
    }

    // Check if the IMMO sequence is in progress
    if (isIMMOSeq)
    {
        IMMO_Buffer[IMMOIndex++] = incomingByte;

        // Check if the IMMO buffer is full
        if (IMMOIndex >= IMMO_BUFFER_SIZE)
        {
            Serial.println("IMMO End, Processing IMMO Data");
            IMMOIndex = 0;

            // Check the last byte in IMMO_Buffer
            byte lastByte = IMMO_Buffer[IMMO_BUFFER_SIZE - 1];
            if (lastByte == DIAG_START_BYTE)
            {
                DiagIsStarting = true;
                Serial.println("Diagnostic Menu Starting");
            }
            else if (lastByte == NORMAL_OPERATION)
            {
                NormalOperation = true;
               // ECU_Buffer[0] = 0xFE;
                Serial.println("Normal Operation Detected");
            }

            isIMMOSeq = false;
            // Set PIDS to zero, Ready for Normal Operation
            Speed_PID = 0;
            Gear_PID = 0;
            Coolant_PID = 0;
        }
    }
}

// Function to handle the bike off condition
void handleBikeOffCondition()
{
    if ((millis() - lastDataTime > BIKE_OFF_TIMEOUT) && (lastDataTime != 0))
    {
        isIMMOSeq = false;
        DiagIsStarting = false;
        isIMMOHandled = false;
        lastDataTime = 0;
        Serial.println("Bike Off Detected");
    }
}

// function to handle serial data
void processSerialByte(byte incomingByte)
{
    lastDataTime = millis(); // Update lastDataTime when a byte is received

    // Debug print for every byte received
    Serial.print("Received Byte: 0x");
    Serial.println(incomingByte, HEX);

    // First handle any ongoing IMMO sequence
    if (isIMMOSeq || incomingByte == IMMO_START_BYTE)
    {
        handleIMMOSequence(incomingByte);
        return;
    }

    // Proceed only if NormalOperation is flagged true
    if (NormalOperation)
    {
        // If the buffer is not full, add the incoming byte
        if (ECUBufferIndex < ECU_BUFFER_SIZE)
        {
            ECU_Buffer[ECUBufferIndex++] = incomingByte;
            Serial.print("Byte Added to ECU_BUFFER: 0x");
            Serial.println(incomingByte, HEX);
        }

        // Check if it's time to process or reset the data
        if (millis() - lastByteTime > FRAME_END_THRESHOLD)
        {
            Serial.println("FRAME_END_THRESHOLD occurred");

            if (ECUBufferIndex == ECU_BUFFER_SIZE)
            {
                // Buffer is full, process the data
                processECUData();
            }
            // Whether the buffer was full or not, reset the buffer index for new data
            ECUBufferIndex = 0;
            Serial.println("ECU_BUFFER Reset");
        }

        // Byte shifting logic to ensure only the last ECU_BUFFER_SIZE bytes are kept
        if (ECUBufferIndex == ECU_BUFFER_SIZE)
        {
            Serial.println("Bytes Shifted in ECU_BUFFER");
            // Shift all bytes in the buffer to the left by one position
            for (int i = 0; i < ECU_BUFFER_SIZE - 1; i++)
            {
                ECU_Buffer[i] = ECU_Buffer[i + 1];
            }

            // Insert the new byte at the end of the buffer
            ECU_Buffer[ECU_BUFFER_SIZE - 1] = incomingByte;
        }

        lastByteTime = millis(); // Update lastByteTime
    }
}


void loop()
{
    handleBikeOffCondition();
    ReadSerialData();
}
