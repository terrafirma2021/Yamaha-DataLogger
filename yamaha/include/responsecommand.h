#pragma once
#include <string> // For std::string

extern void menu(std::string command);
extern void dir();
extern void renameFile(std::string oldName, std::string newName);
extern void printFileContents(std::string filename);
extern void deleteFile(std::string filename);
extern void deleteAll();
extern void copyFile(std::string filename, std::string newFilename);
extern void createFile(std::string filename);
extern void sendResponse(const std::string &message);
extern void showRATIOS();
extern void sendResponse(const std::string &message);
extern void toUpperCaseInPlace(std::string &str);
extern void trimInPlace(std::string &str);
extern void credits();
void handleActionWithArgs(const std::string& action, const std::string& args);

void menu(std::string command) {
  // Menu Header
  sendResponse("\n\n**** Yamaha Datalogger ****\n");

  // Set Gear Ratios
  sendResponse("Enter a command :");

  
  // Gear Ratios Options
sendResponse("\n**** Gear Ratio Control ****\n");
  sendResponse("Start the learn process whilst bike is on the stand!");
sendResponse("1. Ratios - Display Ratios stored in RATIOS.TXT");
sendResponse("2. Ratio Reset - wipe all saved gear ratios. WARNING!!");
sendResponse("3. Gears - Start gear training");

// Spiffs Commands
sendResponse("\n**** Spiffs Commands ****\n");
sendResponse("4. Dir - List files");
sendResponse("5. Rename <oldName> <newName> - Rename a file");
sendResponse("6. Print <filename> - Print the contents of a file");
sendResponse("7. Delete <filename> - Delete a file");
sendResponse("8. Delete All - Delete all files from SPIFFS");
sendResponse("9. Copy <filename> <newFilename> - Copy a file");
sendResponse("10. Create <filename> - Create a new file");
sendResponse("11. Q or Quit - Exit");

// Debug Functions
sendResponse("\n**** Debug Functions ****\n");
sendResponse("12. Debug Off - Turn off debug mode");
sendResponse("13. Debug Rx - Turn on debug mode for receiving data");
sendResponse("14. Debug Tx - Turn on debug mode for transmitting data");
sendResponse("15. Debug Pid - Turn on debug mode for PID calculations");
sendResponse("16. Bike On - Disable bike timer, all flags will not reset");
sendResponse("17. Bike Off - Enable bike timer, enable normal startup");
sendResponse("18. Menu - Print a list of all commands");
sendResponse("19. Reset - Restart the ESP");
sendResponse("20. Credits - Thanks");
}

void receiveResponse(std::string message)
{
    trimInPlace(message);
    toUpperCaseInPlace(message);

    // Attempt to match and handle simple commands that do not require arguments
    if (message == "MENU") {
        menu("MENU");
    } else if (message == "DIR") {
        dir();
    } else if (message == "RATIOS") {
        showRATIOS();
    } else if (message == "Q" || message == "QUIT") {
        sendResponse("Quitting...");
    } else if (message == "DEBUG OFF" || message == "DEBUG 0") {
        sendResponse("Command Received: Debug Off");
        Debug_RX = false;
        Debug_TX = false;
        Debug_PIDS = false;
        Debug_YAM = false;
        DisableBikeOff_Flag = false;
    } else if (message == "DEBUG RX") {
        sendResponse("Command Received: Debug RX");
        Debug_RX = true;
        Debug_TX = false;
        Debug_PIDS = false;
    } else if (message == "DEBUG TX") {
        sendResponse("Command Received: Debug TX");
        Debug_TX = true;
        Debug_RX = false;
        Debug_PIDS = false;
    } else if (message == "DEBUG PID" || message == "DEBUG PIDS") {
        sendResponse("Command Received: Debug PIDS");
        Debug_PIDS = true;
        Debug_RX = false;
        Debug_TX = false;
    } else if (message == "DEBUG YAM") {
        sendResponse("Command Received: Debug YAM");
        Debug_YAM = true;
        Debug_RX = false;
        Debug_TX = false;
        Debug_PIDS = false;
    } else if (message == "BIKE ON") {
        sendResponse("Command Received: Bike timer disabled");
        DisableBikeOff_Flag = false;
    } else if (message == "BIKE OFF") {
        sendResponse("Command Received: Bike timer enabled");
        DisableBikeOff_Flag = true;
    } else if (message == "RATIO RESET") {
        sendResponse("Command Received: Ratio reset test");
        ratioReset = true;
    } else if (message == "GEARS") {
        sendResponse("Command Received: GEARS, Starting Gear training");
        gearLearning = true;
    } else if (message == "RESET") {
        sendResponse("Command Received: Bye!");
        ESP.restart();
    } else if (message == "CREDITS") {
        credits();
    } else {
        // Determine the action and arguments for more complex commands
        size_t spaceIndex = message.find(' ');
        std::string action = (spaceIndex != std::string::npos) ? message.substr(0, spaceIndex) : message;
        std::string args = (spaceIndex != std::string::npos) ? message.substr(spaceIndex + 1) : "";

        // Handle commands that expect both actions and arguments
        handleActionWithArgs(action, args);
    }
}

void handleActionWithArgs(const std::string& action, const std::string& args) {
    if (action == "RENAME") {
        // Additional argument handling for "rename"
        size_t spacePos = args.find(' ');
        if (spacePos == std::string::npos) {
            sendResponse("Invalid RENAME command format. Usage: RENAME <oldName> <newName>");
        } else {
            std::string oldName = args.substr(0, spacePos);
            std::string newName = args.substr(spacePos + 1);
            renameFile(oldName, newName);
        }
    } else if (action == "PRINT") {
        printFileContents(args);
    } else if (action == "DELETE") {
        if (args == "ALL") {
            deleteAll();
        } else {
            deleteFile(args);
        }
    } else if (action == "COPY") {
        size_t spacePos = args.find(' ');
        if (spacePos == std::string::npos) {
            sendResponse("Invalid COPY command format. Usage: COPY <filename> <newFilename>");
        } else {
            std::string filename = args.substr(0, spacePos);
            std::string newFilename = args.substr(spacePos + 1);
            copyFile(filename, newFilename);
        }
    } else if (action == "CREATE") {
        createFile(args);
    } else {
        sendResponse("Invalid command. Please try again.");
    }
}




void credits() {
  sendResponse("\n\n**** Yamaha Datalogger 2024 ****\n");
  sendResponse("Big thanks to:\n");
  sendResponse("    TriB : For the L9637D's and LOTS of wise advice\n    https://github.com/HerrRiebmann\n");
  sendResponse("    The Dude Discord: @skydiving123 : For Schooling me alot \n");
  sendResponse("    Unexpected Maker : for the Feather S3 \n    https://unexpectedmaker.com/\n");
  sendResponse("    Jani @RealDash : For a great dash implementation  \n    https://realdash.net/index.php\n");
  sendResponse("    Kevin Hope : for a large contribution to the XT660 Scene\n    https://www.facebook.com/kevin.hope.549\n");
  sendResponse("    Race Chrono : for the best Data Logger \n    https://racechrono.com/\n");
  sendResponse("    xt660x/r/z owners : All of you!\n    https://www.facebook.com/groups/1479421705713602\n");
  sendResponse("    Torque App Android\n    https://torque-bhp.com/\n");
  sendResponse("    For XDF help\n    https://www.motorcycle-tuning.com/\n");
  sendResponse("    Espressif team : for the esp32 s3! \n    https://www.espressif.com/\n");
  sendResponse("    Arduino team : For the Community\n    https://forum.arduino.cc/\n");
  sendResponse("    Sigrok team : for the software to decode the Yamaha\n    https://sigrok.org\n");
  sendResponse("    And a special thanks to the ChatGPT team at OpenAI\n");
}
