
#include <Arduino.h>

// Define constants for time thresholds and timeouts
const unsigned long FRAME_END_THRESHOLD = 5;   // Threshold to detect end of a frame (5 ms)
const unsigned long DIAG_START_TIMEOUT = 2000; // Timeout for detecting 0xCD after IMMO sequence (2 seconds)
const unsigned long BIKE_OFF_TIMEOUT = 10000;  // 10 seconds BikeOff timer

// Define constants for magic numbers
const byte IMMO_START_BYTE = 0x3E;
const byte DIAG_START_BYTE = 0xCD;
const unsigned int IMMO_BUFFER_SIZE = 54; // Adjust this as necessary

// Define the size of the buffers used to store data
#define VEHICLE_SPEED_RAW_BUFFER_SIZE 8
#define ECU_BUFFER_SIZE 5

// Declare the buffers used to store data
byte Vehicle_Speed_Raw_Buffer[VEHICLE_SPEED_RAW_BUFFER_SIZE];
byte ECU_Buffer[ECU_BUFFER_SIZE];
byte IMMO_Buffer[IMMO_BUFFER_SIZE]; // IMMO sequence buffer

// Define the PIDs used to store data
uint16_t RPM_PID;
uint8_t Coolant_PID;
uint8_t Speed_PID;

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

// Global variable for enabling debugging
bool enableDebugging = true; // Set to false to disable debugging

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

// Function to handle the IMMO sequence and start of the diagnostic menu
void handleIMMOSequence(byte incomingByte)
{
  if (incomingByte == IMMO_START_BYTE)
  {
    isIMMOSeq = true;
  }

  if (isIMMOSeq)
  {
    IMMO_Buffer[IMMOIndex++] = incomingByte;
    if (IMMOIndex >= IMMO_BUFFER_SIZE)
    {
      immoSeqEndTime = millis();
      isIMMOSeq = false; // Reset isIMMOSeq as IMMO buffer is full
      IMMOIndex = 0;     // Reset the index for IMMO_Buffer

      // Start of diagnostic menu logic
      if (millis() - immoSeqEndTime <= DIAG_START_TIMEOUT)
      {
        if (incomingByte == DIAG_START_BYTE)
        {
          DiagIsStarting = true;
          if (enableDebugging)
          {
            Serial.println("Diag Menu Init:");
          }
        }
      }
      else
      {
        DiagIsStarting = false;
      }
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
    lastDataTime = 0;
    if (enableDebugging)
    {
      Serial.println("Bike seems to be turned off. Waiting for restart...");
    }
  }
}

// Function to read and process a single byte of serial data
void processSerialByte(byte incomingByte)
{
  lastDataTime = millis(); // Update lastDataTime when a byte is received

  if (isIMMOSeq || DiagIsStarting)
  {
    handleIMMOSequence(incomingByte);
  }
  else
  {
    // If the buffer is not full, add the incoming byte
    if (ECUBufferIndex < ECU_BUFFER_SIZE)
    {
      ECU_Buffer[ECUBufferIndex++] = incomingByte;
    }

    // Check if it's time to process or reset the data
    if (millis() - lastByteTime > FRAME_END_THRESHOLD)
    {
      if (ECUBufferIndex == ECU_BUFFER_SIZE)
      {
        // Buffer is full, process the data
        processECUData();
      }
      // Whether the buffer was full or not, reset the buffer index for new data
      ECUBufferIndex = 0;
    }

    // Byte shifting logic to ensure only the last ECU_BUFFER_SIZE bytes are kept
    if (ECUBufferIndex == ECU_BUFFER_SIZE)
    {
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

// Function to read and process serial data
void readAndProcessSerialData()
{
  while (Serial.available())
  {
    byte incomingByte = Serial.read();
    processSerialByte(incomingByte);
  }
}
