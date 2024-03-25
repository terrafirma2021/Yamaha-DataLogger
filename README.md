### Yamaha DataLogger
ESP32 s3 BLE Yamaha Datalogger

#### Description:
- The Yamaha DataLogger is an ESP32-S3 based device designed to convert k-line data into Canbus ODB2 standard, using ELM327 emulation. 
 Enabling compatibility with apps like RaceChrono, Realdash, and Torque. 
- Now displays PIDS via OLED screen. 
- Now handles automatic gear ratio tuning via BLE Serial "gear learn", (using gipro gear indicators learn function approach).


#### Added Files:
- Added Ecu Emulator for testing
- Added Torque app PID.csv
- Wokwi PCB Design
- My custom PCB build
- L9637d pinout


#### Added OLED Support:
0.96" I2C



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
- MCU Uptime seconds (Wraps around after 65,536 18h.20m)

#### Requirements:
To build this project, you will need the following components:
- L9637D
- <strike>510 pull-up resistor</strike> Not needed for RX
- Level conversion components

#### Additional Information:
- It's possible to pozi-tap the loom for other sensor data not sent via the k-line.
- Wokwi circuit design: https://wokwi.com/projects/391712730296045569

#### Big thanks to:
- [TriB](https://github.com/HerrRiebmann) For the L9637D's and LOTS of wise advice from an ECU VET :) 
- The Dude Discord: @skydiving123 For Schooling me alot :P  \o/
- [Unexpected Maker:](https://esp32s3.com) for the Feather S3 and support
- [RealDash](https://realdash.net/index.php)
- [Race Chrono](https://racechrono.com)


![Screenshot](https://github.com/terrafirma2021/Yamaha-DataLogger/blob/main/Newpids_n.jpg)
