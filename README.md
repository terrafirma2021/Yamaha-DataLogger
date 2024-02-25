### Yamaha DataLogger
ESP32 s3 BLE Yamaha Datalogger

#### Description:
The Yamaha DataLogger is an ESP32-based device designed to convert data received from the k-line into Canbus ODB2 standard. This functionality enables compatibility with various apps like RaceChrono, Realdash, and Torque. Additionally, the project plans to support Gear information in the future.

#### Supported PIDs:
- RPM
- Speed
- Coolant Temp
- Gear
- Voltage ( todo)

#### Requirements:
To build this project, you will need the following components:
- L9637D
- 510 pull-up resistor
- Level conversion components

#### Additional Information:
- It's possible to pozi-tap the loom for other sensor data not sent via the k-line.

