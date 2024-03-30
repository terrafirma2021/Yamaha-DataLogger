#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>


// PIDS
extern uint16_t RPM_PID;       // RPM * 50 = RAW
extern uint8_t Speed_PID;      // RAW km/h
extern uint8_t Coolant_PID;    // Temp = RAW
extern uint8_t Error_PID;      // Error code
extern uint8_t Gear_PID;       // RAW 00-05
extern uint8_t Temp_PID;      // MCU Temp
extern uint8_t CPU_PID;       // CPU Freq mhz
extern uint8_t RAM_Free_PID;  // Free Ram
extern uint8_t Max_Speed_PID; // Max Speed Reached
extern uint16_t MCU_Uptime_PID;


// Function to convert a PID value to hexadecimal string
template <typename T>
std::string hexToString(T value)
{
    std::ostringstream stream;
    // Determine the number of bytes in the type T
    int numBytes = sizeof(T);
    // Loop through each byte
    for (int i = numBytes - 1; i >= 0; --i)
    {
        // Extract the current byte
        uint8_t byte = (value >> (i * 8)) & 0xFF;
        // Append the byte as hexadecimal to the stream
        stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    return stream.str();
}


std::string handleCommand(const std::string &command)
{
    if (command == "ATI" || command == "AT@1")
        return "ELM327 v2.1";
    else if (command == "ATS1" || command == "AT S1")
        // Implementation for ATS1
        return "OK";
    else if (command == "ATS0" || command == "AT S0")
        // Implementation for ATS0
        return "OK";
    else if (command == "ATH1" || command == "AT H1")
        // Implementation for ATH1
        return "OK";
    else if (command == "ATH0" || command == "AT H0")
        // Implementation for ATH0
        return "OK";
    else if (command == "ATZ")
        // Implementation for ATZ
        return "OK";
    else if 
        (
        command == "ATE0" ||
        command == "ATD" ||
        command == "ATPC" ||
        command == "ATM0" ||
        command == "ATL0" ||
        command == "ATST62" ||
        command == "ATSP0" ||
        command == "ATSP0" ||
        command == "ATAT1" ||
        command == "ATAT2" ||
        command == "ATAT2" ||
        command == "ATSP6" ||
        command == "ATSPA6"
        )
        return "OK";
    else if (command == "ATDPN")
        return "6";         // Protocol Number 6 CAN bus
    else if (command == "0100") // PID 1 - Supported PIDs [01-20]
        return "41 00 08 18 00 01";
    else if (command == "0120") // PID 2 - Supported PIDs [21-40]
        return "41 20 00 00 00 01";
    else if (command == "0140") // PID 3 - Supported PIDs [41-60]
        return "41 40 00 00 00 01";
    else if (command == "0160") // PID 4 - Supported PIDs [61-80]
        return "41 60 00 00 00 01";
    else if (command == "0180") // PID 5 - Supported PIDs [81-A0]
        return "41 80 00 00 00 00";
    else if (command == "01A0") // PID 6 - Supported PIDs [A1-C0]
        return "41 A0 10 00 00 00 00";
    else if (command == "01C0") // PID 7 - Supported PIDs [C1-E0]
        return "41 C0 NO DATA";
    else if (command == "0105") // Engine Coolant Temperature (PID 0105)
        return "41 05 " + hexToString(Coolant_PID);
    else if (command == "0105 1") // Engine Coolant Temperature (PID 0105)  // Race chrono +1 response
        return "41 05 " + hexToString(Coolant_PID);
    else if (command == "010C") // RPM (PID 010C)
        return "41 0C " + hexToString(RPM_PID);
    else if (command == "010C 1") // RPM (PID 010C) // race chrono +1 response
        return "41 0C " + hexToString(RPM_PID);
    else if (command == "010D") // Vehicle Speed (PID 010D)
        return "41 0D " + hexToString(Speed_PID);
    else if (command == "010D 1") // Vehicle Speed (PID 010D) // race chrono +1 response
        return "41 0D " + hexToString(Speed_PID);
    else if (command == "01A4") // Transmission Actual Gear (PID 01A4)
        return "41 A4 " + hexToString(Gear_PID);
    else if (command == "01A4 1") // Transmission Actual Gear (PID 01A4) // +1
        return "41 A4 " + hexToString(Gear_PID);
    else if (command == "0902") // VIN (PID 0902)
        return "49 02 00 00 59 41 4D 41 48 41 45 53 50 33 32 4F 44 42";
    else if (command == "0904") // Calibration ID (PID 0904)
        return "49 04 00 00 00 00";
    else if (command == "090A") // ECU Name (PID 090A)
        return "49 0A 45 53 50 33 32 37 45 6D 75 6C 61 74 6F 72 00 00 00 00 00 00";
    else if (command == "01009") // ???
        return "41 009 NO DATA";

    // custom PIDS

    else if (command == "1001 1") // Error code ( Custom PID 0901) // +1
        return "41 02 " + hexToString(Error_PID);
    else if (command == "1001") // Error code ( Custom PID 0901)
        return "41 02 " + hexToString(Error_PID);
    else if (command == "1002 1") // MCU Temp C ( Custom PID 0903) // +1
        return "41 02 " + hexToString(Temp_PID);
    else if (command == "1002") // MCU Temp C ( Custom PID 0903)
        return "41 02 " + hexToString(Temp_PID);
    else if (command == "1003 1") // CPU Freq mhz ( Custom PID 0904) // +1
        return "41 02 " + hexToString(CPU_PID);
    else if (command == "1003") // CPU Freq mhz ( Custom PID 0904)
        return "41 02 " + hexToString(CPU_PID);
    else if (command == "1004 1") // Free Ram ( Custom PID 0905) // +1
        return "41 02 " + hexToString(RAM_Free_PID);
    else if (command == "1004") // Free Ram ( Custom PID 0905)
        return "41 02 " + hexToString(RAM_Free_PID);
    else if (command == "1005 1") // Max Speed ( Custom PID 0907) // +1
        return "41 02 " + hexToString(Max_Speed_PID);
    else if (command == "1005") // Max Speed ( Custom PID 0907)
        return "41 02 " + hexToString(Max_Speed_PID);
    else if (command == "1006 1") // Max Speed ( Custom PID 0907) // +1
        return "41 02 " + hexToString(MCU_Uptime_PID);
    else if (command == "1006") // Max Speed ( Custom PID 0907)
        return "41 02 " + hexToString(MCU_Uptime_PID);
    else {

        if (!command.empty())
        {
            Serial.print("Unknown command or length mismatch. Received: ");
            Serial.println(command.c_str());
        }
    }

    return "";
}