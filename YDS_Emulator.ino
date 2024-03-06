#include <Arduino.h>

#define STARTUP_BUFFER_SIZE 55
uint8_t Startup_buf[STARTUP_BUFFER_SIZE];

bool startupSent = false;
bool ecuRespondReady = false;

// Default values for RPM, Speed, and Coolant Temperature
uint8_t rpm = 0;
uint8_t speed = 0;
uint8_t errorCode = 0;
uint8_t coolantTemp = 0;

// Fake IMMO/ Normal Boot mode
void sendStartupMessages() {
  if (!startupSent) {
    // Fill Startup_buf with startup messages
    Startup_buf[0] = 0x3E; // 0x3E message

    // Fill IMMO sequence bytes (all but the last byte)
    for (int i = 1; i < STARTUP_BUFFER_SIZE - 1; ++i) {
      Startup_buf[i] = 0x00; // Default value set to 0x00
    }

    // Set the last byte to 0xFE
    Startup_buf[STARTUP_BUFFER_SIZE - 1] = 0xFE;

    // Send Startup_buf over Serial1
    Serial1.write(Startup_buf, STARTUP_BUFFER_SIZE);
    //Serial.println("Startup messages sent");
    startupSent = true;
    ecuRespondReady = true; // Set ECU response ready flag
  }
}

void ecuRespond() {
  static unsigned long lastResponseTime = 0;
  const unsigned long responseInterval = 7; // Interval for sending the ECU response in milliseconds

  unsigned long currentMillis = millis();
  // Check if enough time has elapsed since the last response and the ecuRespondReady flag is set
  if (ecuRespondReady && currentMillis - lastResponseTime >= responseInterval) {
    // Fill ecuRespond_buf with ECU response
    uint8_t ecuRespond_buf[6];
    ecuRespond_buf[0] = 0x01;
    ecuRespond_buf[1] = rpm; // Update RPM value
    ecuRespond_buf[2] = speed; // Update Speed value
    ecuRespond_buf[3] = errorCode; // Update Error Code value
    ecuRespond_buf[4] = coolantTemp; // Update Coolant Temperature value

    // Calculate checksum
    uint8_t checksum = ecuRespond_buf[1] + ecuRespond_buf[2] + ecuRespond_buf[3] + ecuRespond_buf[4];
    ecuRespond_buf[5] = checksum;

    // Send ecuRespond_buf over Serial1
    Serial1.write(ecuRespond_buf, sizeof(ecuRespond_buf));

    // Update lastResponseTime
    lastResponseTime = currentMillis;

    // Introduce a delay of 7 milliseconds
    delay(responseInterval);
  }
}

void setup() {
  Serial.begin(115200);
  // Print the message along with the initial values for RPM, Speed, Error Code, and Coolant Temp. 
  Serial.println("Hi, Pick your ECU Values, in decmimal with spaces between values then press enter to dynamically change them\n"
                 "RPM: 0\n"
                 "Speed: 0\n"
                 "Error Code: 0\n" // Added Error Code line
                 "Coolant Temp: 0");
  Serial1.begin(16040, SERIAL_8N1, 7, 8); // Using pins 7 (RX) and 8 (TX) for Serial1
}

void loop() {
  sendStartupMessages(); // Run once at the beginning
  ecuRespond(); // Run continuously after startup messages are sent

  if (Serial.available() > 0) {
    // Read user input from the serial monitor
    String userInput = Serial.readStringUntil('\n');

    // Convert the strings to integers
    int rpmInput, speedInput, errorCodeInput, coolantTempInput;
    sscanf(userInput.c_str(), "%d %d %d %d", &rpmInput, &speedInput, &errorCodeInput, &coolantTempInput);

    // Convert the values to uint8_t and update RPM, Speed, Error Code, and Coolant Temperature values
    rpm = (uint8_t)rpmInput;
    speed = (uint8_t)speedInput;
    errorCode = (uint8_t)errorCodeInput;
    coolantTemp = (uint8_t)coolantTempInput;

    // Print updated values
    Serial.print("RPM: ");
    Serial.println(rpm * 50);
    Serial.print("Speed: ");
    Serial.println(speed * 8);
    Serial.print("Error Code: ");
    Serial.println(errorCode);
    Serial.print("Coolant Temp: ");
    Serial.println(coolantTemp);
  }
}
