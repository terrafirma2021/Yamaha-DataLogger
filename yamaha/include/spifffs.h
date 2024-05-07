#pragma once
#include <SPI.h>
#include <SPIFFS.h>
#include <string>
#include <iostream>
#include <sstream>

// Function prototypes
void menu(std::string command);
void dir();
void renameFile(std::string oldName, std::string newName);
void printFileContents(std::string filename);
void deleteFile(std::string filename);
void deleteAll();
void copyFile(std::string filename, std::string newFilename);
void createFile(std::string filename);
extern void sendResponse(const std::string &message);
void showRATIOS();

// External constant vector declaration
extern std::vector <float> constRatios;


void dir() {
  sendResponse("Listing files:");
  File root = SPIFFS.open("/");

  if (!root) {
    sendResponse("Failed to open root directory.");
    return;
  }

  // Check if there are any files in the directory
  bool anyFiles = false;
  for (File file = root.openNextFile(); file; file = root.openNextFile()) {
    anyFiles = true;
    // Process each file by sending details about the file
    sendResponse(std::string(file.name()) + " - " + std::to_string(file.size()) + " bytes");
  }

  // If no files found, print a message
  if (!anyFiles) {
    sendResponse("No files found.");
  }
}


void renameFile(std::string oldName, std::string newName)
{
  // Ensure both oldName and newName start with a '/'
  if (oldName[0] != '/')
  {
    oldName = "/" + oldName;
  }
  if (newName[0] != '/')
  {
    newName = "/" + newName;
  }

  // Debug print
  sendResponse("Attempting to rename from " + oldName + " to " + newName);

  // Check if the old file exists
  if (!SPIFFS.exists(oldName.c_str()))
  {
    sendResponse("File does not exist: " + oldName);
    return;
  }

  // Attempt to rename
  if (SPIFFS.rename(oldName.c_str(), newName.c_str()))
  {
    sendResponse("File renamed successfully");
  }
  else
  {
    sendResponse("Failed to rename file");
  }
}

void printFileContents(std::string filename)
{
  // Ensure filename starts with a '/'
  if (filename[0] != '/')
  {
    filename = "/" + filename;
  }

  File file = SPIFFS.open(filename.c_str(), "r");
  if (!file)
  {
    sendResponse("Failed to open file for reading: " + filename);
    return;
  }

  sendResponse("\n\nFile content:");
  // Read and print the content line by line
  String line;
  while (file.available())
  {
    line = file.readStringUntil('\n');
    sendResponse(std::string(line.c_str()));
  }
  file.close();
}


void deleteFile(std::string filename)
{
  // Ensure filename starts with a '/'
  if (filename[0] != '/')
  {
    filename = "/" + filename;
  }

  if (SPIFFS.remove(filename.c_str()))
  {
    sendResponse("File deleted successfully: " + filename);
  }
  else
  {
    sendResponse("Failed to delete file: " + filename);
  }
}

void deleteAll() {
  sendResponse("Deleting all files from SPIFFS...");
  File root = SPIFFS.open("/");
  if (!root) {
    sendResponse("Failed to open root directory.");
    return;
  }
  for (File file = root.openNextFile(); file; file = root.openNextFile()) {
    String filePath = String("/") + file.name(); // Ensure the file path starts with '/'
    file.close(); // Close the file before attempting to delete
    if (SPIFFS.remove(filePath)) {
      Serial.println("Removed file: " + filePath); // Print the name of the file removed
    } else {
      Serial.println("Failed to remove file: " + filePath); // Print if the file removal failed
    }
  }
  sendResponse("All files deleted successfully.");
}




void copyFile(std::string filename, std::string newFilename)
{
  // Ensure both filename and newFilename start with a '/'
  if (filename[0] != '/')
  {
    filename = "/" + filename;
  }
  if (newFilename[0] != '/')
  {
    newFilename = "/" + newFilename;
  }

  File originalFile = SPIFFS.open(filename.c_str(), "r");
  if (!originalFile)
  {
    sendResponse("Failed to open original file for reading: " + filename);
    return;
  }

  File newFile = SPIFFS.open(newFilename.c_str(), "w");
  if (!newFile)
  {
    sendResponse("Failed to open new file for writing: " + newFilename);
    originalFile.close();
    return;
  }

  if (originalFile.available())
  {
    newFile.write(originalFile.read());
  }

  originalFile.close();
  newFile.close();

  sendResponse("File copied successfully from " + filename + " to " + newFilename);
}

void createFile(std::string filename)
{
  // Check if filename is provided and ensure it starts with a '/'
  if (filename.empty())
  {
    sendResponse("Please provide a filename.");
    return;
  }

  if (filename[0] != '/')
  {
    filename = "/" + filename;
  }

  // Create an empty file
  File file = SPIFFS.open(filename.c_str(), "w");
  if (!file)
  {
    sendResponse("Failed to create file: " + filename);
    return;
  }

  file.close();
  sendResponse("File created successfully: " + filename);
}

void showRATIOS() {
    sendResponse("\nRATIOS.TXT:");

    // Open RATIOS.TXT for reading
    File ratiosFile = SPIFFS.open("/RATIOS.TXT", "r");
    if (!ratiosFile) {
        sendResponse("Failed to open RATIOS.TXT.");
        return;
    }

    // Check if RATIOS.TXT is empty
    if (ratiosFile.size() == 0) {
        sendResponse("RATIOS.TXT is empty.");
        ratiosFile.close();
    } else {
        // Print contents of RATIOS.TXT
        size_t index = 0;
        sendResponse("Index: Ratio:");
        while (ratiosFile.available()) {
            std::ostringstream oss;
            oss << (index + 1) << ": " << ratiosFile.readStringUntil('\n').c_str();
            sendResponse(oss.str());
            index++;
        }
        ratiosFile.close();
    }

    sendResponse("\nconstRatios:");
    // Print contents of constRatios
    for (size_t i = 0; i < constRatios.size(); ++i) {
        std::ostringstream oss;
        oss << (i + 1) << ": " << constRatios[i];
        sendResponse(oss.str());
    }
}






void loadSpiffRatios() {
  sendResponse("Loading SPIFFS ratios...");

  // Open SPIFFS root directory
  File root = SPIFFS.open("/");
  if (!root) {
    sendResponse("Failed to open SPIFFS root directory.");
    return;
  }

  // Check if RATIOS.TXT exists
  if (!SPIFFS.exists("/RATIOS.TXT")) {
    // If file does not exist, create it
    sendResponse("RATIOS.TXT does not exist. Creating...");
    File ratiosFile = SPIFFS.open("/RATIOS.TXT", "w");
    if (!ratiosFile) {
      sendResponse("Failed to create RATIOS.TXT.");
      return;
    }
    // Close the file after creating
    ratiosFile.close();
    sendResponse("RATIOS.TXT created.");
    return; // No need to load ratios since the file is empty
  }

  // If RATIOS.TXT exists, open it for reading
  File ratiosFile = SPIFFS.open("/RATIOS.TXT", "r");
  if (!ratiosFile) {
    sendResponse("Failed to open RATIOS.TXT for reading.");
    return;
  }

  // Check if RATIOS.TXT is empty
  if (ratiosFile.size() == 0) {
    sendResponse("RATIOS.TXT is empty.");
    ratiosFile.close();
    return;
  }

  // Load contents of RATIOS.TXT into constRatios
  constRatios.clear(); // Clear existing ratios
  //sendResponse("Loading ratios:");
  size_t index = 0;
  while (ratiosFile.available()) {
    String ratioString = ratiosFile.readStringUntil('\n');
    float ratio = ratioString.toFloat();
    constRatios.push_back(ratio);
    // Debug print for each ratio value and index
    //sendResponse("Index: " + std::to_string(index) + ", Ratio: " + std::to_string(ratio));
    index++;
  }
  ratiosFile.close();

  // Print loaded ratios for debugging
  sendResponse("Loaded ratios:");
  for (size_t i = 0; i < constRatios.size(); ++i)
  {
    sendResponse("Index: " + std::to_string(i + 1) + ", Ratio: " + std::to_string(constRatios[i]));
  }

  sendResponse("SPIFFS ratios loaded successfully.");
}
