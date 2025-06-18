# ESP32 Doorbell
### ESP32 with an Accelerometer and Screen intended to detect door knocks and send signals over LAN to other devices.

## Overview
This project uses the ESP-IDF framework from PlatformIO on an ESP32 Dev Board. I've also written a small and basic SSD1306 driver to display uppercase characters to the screen. Once a vibration that passes the designated threshold is detected, the screen displays a message and a UDP message is broadcasted over WiFi on the 17388 port. This UDP message is intended to be captured by another device.

I wrote this to work in tandem with my [base station](https://github.com/KaiserMighty/ESP32-Base-Station), which would play an audio cue. Intended to allow me to hear people knocking on my outer door while I am inside my room with headphones on.

## Organization
This device assumes that the ESP32 has two I2C devices:

* MPU6050 based accelerometer/gyroscope
* SSD1306 based display

The I2C SCL and SDA lines are assumed to be connected to GPIO pins 18 and 19 respectively.  
The WiFi SSID and Password are editable within `main.c`.