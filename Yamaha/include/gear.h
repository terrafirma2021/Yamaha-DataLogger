#pragma once
#include <vector>
#include <unordered_map>
#include <cmath>
#include <SPI.h>
#include <SPIFFS.h>

// Main
extern bool Gear_Speed_Ready;
extern bool Gear_RPM_Ready;
extern uint8_t gear_speed;
extern uint16_t gear_rpm;
extern uint8_t Gear_PID;
extern void sendResponse(const std::string & message);

bool gearLearning = false;
bool ratioReset = false;

// Global variables and constants
float currentRatio = 0.0f;
size_t closestIndex = 0;
constexpr int ratioArrayMax = 79;  // set odd to ensure always a gear mode
constexpr int MAX_GEARS = 5;       // Set 5 for xt660x
constexpr float constsDeviation = 6.0f;
constexpr float lookupDeviation = 6.0f;

std::vector <float> ratioArray;
std::vector <float> constRatios;

// Function prototypes
void gears();
void resetRATIOS();
void gearLearn();
void gearConsts(float currentRatio);
void writeGearConstantsToSPIFFS();
void gearLookup();


void gears(){
  resetRATIOS();
  gearLearn();
  gearLookup();
}

void resetRATIOS()
{
  if (!ratioReset)
  {
    //sendResponse("Ratio reset not enabled");
    return;
  }

  // Ensure the SPIFFS is initialized
  if (!SPIFFS.begin()) {
    sendResponse("Failed to mount SPIFFS");
    return;
  }

  std::string filePath = "/RATIOS.TXT";

  // Check if the file exists
  if (SPIFFS.exists(filePath.c_str()))
  {
    if (SPIFFS.remove(filePath.c_str()))
    {
      //sendResponse("Existing RATIOS.TXT removed successfully");
    }
    else
    {
      sendResponse("Failed to remove existing RATIOS.TXT");
    }
  }

  File file = SPIFFS.open(filePath.c_str(), "w");
  if (!file)
  {
    sendResponse("Failed to create new RATIOS.TXT");
    return;
  }
  file.close();
  sendResponse("New RATIOS.TXT created successfully");

  ratioReset = false;
  gearLearning = true;
  constRatios.empty();  // empty the constRatios
  gearLearn();
}

  void gearLearn() {
  // First, check if the gear speed and RPM data are ready.
  if (!Gear_Speed_Ready || !Gear_RPM_Ready || !gearLearning) {
    return;
  }

  // Next, check if the gear speed is too low or the RPM is zero.
  if (gear_speed < 10 || gear_rpm == 0) {
    sendResponse("Shift into Gear 1 now");
    ratioArray.clear(); // Clear the ratioArray only if this specific condition is met.
    constRatios.clear(); // Clear the consts ready for new ratios
    Gear_PID = 0; // Reset PID.
    
    Gear_Speed_Ready = Gear_RPM_Ready = false; // Reset flags after processing.
    return; // Exit the function after handling this condition.
  }

  // If the data is ready and valid (i.e., gear_speed >= 10 and gear_rpm != 0), proceed with the calculations.
  float currentRatio = static_cast<float>(gear_rpm) / static_cast<float>(gear_speed);
  ratioArray.push_back(currentRatio);

  // Reset flags for the next measurement, regardless of the condition outcomes above.
  Gear_Speed_Ready = Gear_RPM_Ready = false;

  // Check if the ratioArray has accumulated enough entries to process.
  if (ratioArray.size() >= ratioArrayMax) {
    std::unordered_map<float, int> counts;
    int maxCount = 0;
    float maxRatio = 0.0f;

    // Determine the most frequently occurring ratio in ratioArray.
    for (float ratio : ratioArray) {
      int count = ++counts[ratio];
      if (count > maxCount) {
        maxCount = count;
        maxRatio = ratio;
      }
    }

    currentRatio = maxRatio; // Update the actual ratio to the most common one found.
    ratioArray.clear(); // Clear the ratioArray after processing to start fresh for the next set of measurements.
    gearConsts(currentRatio); // Call another function to handle the actual ratio (not shown here).
  }
}

void gearConsts(float currentRatio)
{
  if (constRatios.size() >= MAX_GEARS)
  {
    sendResponse("Max Gears Reached. Gear learning has completed.");
    gearLearning = false;
    // Write gear constants to RATIOS.TXT
    writeGearConstantsToSPIFFS();
    return;
  }

  // Check if constRatios is empty, if so, add the current ratio as a new gear ratio
  if (constRatios.empty())
  {
    constRatios.push_back(currentRatio);
    sendResponse("Gear " + std::to_string(constRatios.size()) + " set");

    // Print the current gear ratios
    sendResponse("Current gear ratios:");
    sendResponse("Gear 1 set: " + std::to_string(currentRatio));
    return; // Exit the function
  }

  // Check if currentRatio is close to the last gear ratio
  if (std::fabs(constRatios.back() - currentRatio) <= constsDeviation)
  {
    sendResponse("Condition 1: Current ratio " + std::to_string(currentRatio) + " is close to Gear " + std::to_string(constRatios.size()) + ".");
    return; // Return without adding the ratio
  }

  // Check if currentRatio is larger than the last gear ratio
  if (currentRatio > constRatios.back())
  {
    sendResponse("Condition 2: Current ratio " + std::to_string(currentRatio) + " is larger than Gear " + std::to_string(constRatios.size()) + ".");
    return; // Return without adding the ratio
  }

  // Add the current ratio as a new gear ratio
  constRatios.push_back(currentRatio);
  sendResponse("Gear " + std::to_string(constRatios.size()) + " set");

  // Print the current gear ratios
  sendResponse("Current gear ratios:");
  for (size_t i = 0; i < constRatios.size(); ++i)
  {
    sendResponse("Gear " + std::to_string(i + 1) + ": " + std::to_string(constRatios[i]));
  }
}

void writeGearConstantsToSPIFFS() {
    // Open or create RATIOS.TXT for writing
    File ratiosFile = SPIFFS.open("/RATIOS.TXT", "w");
    if (!ratiosFile) {
        sendResponse("Failed to open RATIOS.TXT for writing.");
        return;
    }

    // Write gear constants to RATIOS.TXT
    for (size_t i = 0; i < constRatios.size(); ++i) {
        ratiosFile.println(constRatios[i]);
    }
    ratiosFile.close();

    sendResponse("Gear constants written to RATIOS.TXT successfully.");
}

void gearLookup() {
  if (ratioReset || gearLearning || constRatios.empty()) {
    return;
  }

  // Check for division by zero before calculating realRatio
  if (gear_speed < 7 || gear_rpm == 0) { // Less than 7 km/h assume Neutral
    Gear_PID = 0; // Set Gear_PID to 0 if there's no valid input
    return;
  }

  float realRatio = static_cast<float>(gear_rpm) / static_cast<float>(gear_speed);

  for (size_t i = 0; i < constRatios.size(); ++i) {
    if (std::fabs(constRatios[i] - realRatio) <= lookupDeviation) {
      closestIndex = i; // Set the zero-based index when a match is found
      Gear_PID = closestIndex + 1; // Convert zero-based index to one-based
      return; // Exit function since a gear is matched
    }
  }
}



