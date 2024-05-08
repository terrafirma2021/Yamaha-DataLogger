### Yamaha DataLogger
ESP32 s3 BLE Yamaha Datalogger

#### Description:
- The Yamaha DataLogger is an ESP32-S3 based device designed to convert k-line data into Canbus ODB2 standard, using ELM327 emulation. 
 Enabling compatibility with apps like RaceChrono, Realdash, and Torque. 
- Now displays PIDS via OLED screen. 
- Now handles automatic gear ratio tuning via BLE Serial "Gears", (using gipro gear indicators learn function approach).

#### updates:
- Migrated NVS to spiffs
- gear ratios persist on cold boot / auto load on startup
- added simple temrinal menu via serial/ble
- All serial/ble serial commands are simultaneously mirrored.
- General tidy up
- Migrated send/response commands



#### Added Files:
- Added Ecu Emulator for testing
- Added Torque app PID.csv
- Wokwi PCB Design
- My custom PCB build
- L9637d pinout
- Realdash XML (Plug and play)
- racechrono.rcz file
- 3d Printed Case STL + Photos
- 3D printed case with switch mount (switch can be found below):
- https://www.amazon.co.uk/dp/B01N367QLZ


#### Added OLED Support:
- 0.96" I2C



#### Supported PIDs:
- RPM
- Speed
- Error code
- Coolant Temp
- Gear
- MCU Temp
- CPU Freq
- Ram Free
- Top Speed
- MCU Uptime seconds

#### Requirements:
To build this project, you will need the following components:
- L9637D
- <strike>510 pull-up resistor</strike> Not needed for RX
- Level conversion components
- Due to ESP32 startup time, an external power supply switch will be required to enable esp32 to start before the ecu! This saves re-writing the codebase.

#### Additional Information:
- It's possible to pozi-tap the loom for other sensor data not sent via the k-line.
- Wokwi circuit design: https://wokwi.com/projects/391712730296045569
- Thingverse link here: https://www.thingiverse.com/thing:6560376

#### Big thanks to:
- [TriB](https://github.com/HerrRiebmann) For the L9637D's and LOTS of wise advice from an ECU VET :) 
- The Dude Discord: @skydiving123 For Schooling me alot :P  \o/
- [Unexpected Maker:](https://esp32s3.com) for the Feather S3 and support
- [RealDash](https://realdash.net/index.php)
- [Race Chrono](https://racechrono.com)

![Screenshot](https://github.com/terrafirma2021/Yamaha-DataLogger/blob/main/3d%20printed%20case/20240402_010515.jpg)
![Screenshot](https://raw.githubusercontent.com/terrafirma2021/Yamaha-DataLogger/main/Screenshot_20240416_134657_Torque.webp)
