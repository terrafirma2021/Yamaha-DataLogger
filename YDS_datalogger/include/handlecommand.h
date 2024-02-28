#include <string>
#include <BLE.h>

// Define the PIDs used to store data
uint16_t RPM_PID;
uint8_t Coolant_PID;
uint8_t Speed_PID;
uint16_t Gear_PID;

std::string handleCommand(const std::string &command) {
    std::string response;

    if (command == "ATI" || command == "AT@1") {
        response = "ELM327 v2.1";
    } else if (command == "ATS1" || command == "AT S1") {
        Device::getInstance().handleATSCommand = true; // Turn Spaces On
        response = "OK";
    } else if (command == "ATS0" || command == "AT S0") {
        Device::getInstance().handleATSCommand = false; // Turn Spaces Off
        response = "OK";
    } else if (command == "ATH1" || command == "AT H1") {
        Device::getInstance().handleATHCommand = true;
        Device::getInstance().setATH1Active(true); // Set ATH1 active
        response = "OK";
    } else if (command == "ATH0" || command == "AT H0") {
        Device::getInstance().handleATHCommand = false;
        Device::getInstance().setATH1Active(false); // Reset ATH1 active
        response = "OK";
    } else if (command == "ATZ") { // Reset
        Device::getInstance().handleATHCommand = false;
        Device::getInstance().setATH1Active(false);
        Device::getInstance().handleATSCommand = false;
        response = "OK";
    } else if (command == "ATE0" || command == "ATPC" || command == "ATM0" ||
               command == "ATL0" || command == "ATST62" || command == "ATSP0" ||
               command == "ATSP0" || command == "ATAT1" || command == "ATAT2" ||
               command == "ATAT2" || command == "ATSP6" || command == "ATSPA6") {
        response = "OK";
    } else if (command == "ATDPN") {
        response = "6"; // Protocol Number 6 CAN bus
    } else if (command == "0100") { // PID 1 - Supported PIDs [01-20]
        response = "41 00 08 18 00 01";
    } else if (command == "0120") { // PID 2 - Supported PIDs [21-40]
        response = "41 20 00 00 00 01";
    } else if (command == "0140") { // PID 3 - Supported PIDs [41-60]
        response = "41 40 00 00 00 01";
    } else if (command == "0160") { // PID 4 - Supported PIDs [61-80]
        response = "41 60 00 00 00 01";
    } else if (command == "0180") { // PID 5 - Supported PIDs [81-A0]
        response = "41 80 00 00 00 00";
    } else if (command == "01A0") { // PID 6 - Supported PIDs [A1-C0]
        response = "41 A0 10 00 00 00 00";
    } else if (command == "01C0") { // PID 7 - Supported PIDs [C1-E0]
        response = "41 C0 NO DATA";
    } else if (command == "0105") { // Engine Coolant Temperature (PID 0105)
        response = "41 05 " + std::to_string(Coolant_PID);
    } else if (command == "010C") { // RPM (PID 010C)
        response = "41 0C " + std::to_string(RPM_PID);
    } else if (command == "010D") { // Vehicle Speed (PID 010D)
        response = "41 0D " + std::to_string(Speed_PID);
    } else if (command == "01A4") { // Transmission Actual Gear (PID 01A4)
        response = "41 0D 0" + std::to_string(Gear_PID);
    } else if (command == "0902") { // VIN (PID 0902)
        response = "49 02 00 00 59 41 4D 41 48 41 45 53 50 33 32 4F 44 42";
    } else if (command == "0904") { // Calibration ID (PID 0904)
        response = "49 04 00 00 00 00";
    } else if (command == "090A") { // ECU Name (PID 090A)
        response = "49 0A 45 53 50 33 32 37 45 6D 75 6C 61 74 6F 72 00 00 00 00 00 00";
    } else if (command == "01009") { // ???
        response = "41 009 NO DATA";
    } else {
#if DEBUG_LEVEL >= 1
        Serial.println(("Unknown command or length mismatch. Received: " + command).c_str());
#endif
    }

    return response;
}
